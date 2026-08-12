#include "can_driver/can_send.hpp"

#include <atomic>
#include <cassert>
#include <chrono>
#include <thread>
#include <vector>

#include <sys/socket.h>
#include <unistd.h>

int main()
{
  int sockets[2] = {-1, -1};
  assert(socketpair(AF_UNIX, SOCK_DGRAM, 0, sockets) == 0);

  can_driver::CanSend sender;
  sender.setControlFramePeriodMs(20);

  can_frame old_frame{};
  old_frame.can_id = 0x201;
  old_frame.can_dlc = 8;
  old_frame.data[0] = 1;
  can_frame latest_frame = old_frame;
  latest_frame.data[0] = 2;
  sender.push({old_frame, latest_frame});

  std::atomic<bool> running{true};
  std::thread thread(&can_driver::CanSend::sendTask, &sender, sockets[0], std::ref(running));

  can_frame event_one{};
  event_one.can_id = 0x301;
  event_one.can_dlc = 8;
  event_one.data[0] = 3;
  can_frame event_two = event_one;
  event_two.can_id = 0x401;
  event_two.data[0] = 4;
  sender.push({event_one, event_two});

  const auto start = std::chrono::steady_clock::now();
  const auto deadline = start + std::chrono::milliseconds(145);
  std::vector<std::chrono::steady_clock::time_point> control_times;
  std::vector<canid_t> event_ids;
  while (std::chrono::steady_clock::now() < deadline) {
    timeval timeout{};
    timeout.tv_usec = 30000;
    setsockopt(sockets[1], SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));
    can_frame frame{};
    if (read(sockets[1], &frame, sizeof(frame)) == sizeof(frame)) {
      if (frame.can_id == 0x201) {
        assert(frame.data[0] == (control_times.size() < 3 ? 2 : 5));
        control_times.push_back(std::chrono::steady_clock::now());
        if (control_times.size() == 3) {
          can_frame replacement = latest_frame;
          replacement.data[0] = 5;
          sender.setLatestControlFrame(replacement);
        }
      } else {
        event_ids.push_back(frame.can_id);
      }
    }
  }

  running = false;
  thread.join();
  close(sockets[0]);
  close(sockets[1]);

  assert(control_times.size() >= 6);
  for (std::size_t index = 1; index < control_times.size(); ++index) {
    const auto interval = std::chrono::duration_cast<std::chrono::milliseconds>(
      control_times[index] - control_times[index - 1]);
    assert(interval >= std::chrono::milliseconds(10));
    assert(interval <= std::chrono::milliseconds(35));
  }
  assert((event_ids == std::vector<canid_t>{0x301, 0x401}));
  return 0;
}

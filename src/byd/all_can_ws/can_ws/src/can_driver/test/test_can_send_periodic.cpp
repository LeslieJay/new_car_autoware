#include "can_driver/can_send.hpp"

#include <atomic>
#include <cassert>
#include <chrono>
#include <thread>

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

  const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(125);
  int received = 0;
  while (std::chrono::steady_clock::now() < deadline) {
    timeval timeout{};
    timeout.tv_usec = 30000;
    setsockopt(sockets[1], SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));
    can_frame frame{};
    if (read(sockets[1], &frame, sizeof(frame)) == sizeof(frame)) {
      assert(frame.can_id == 0x201);
      assert(frame.data[0] == 2);
      ++received;
    }
  }

  running = false;
  thread.join();
  close(sockets[0]);
  close(sockets[1]);

  assert(received >= 5);
  return 0;
}

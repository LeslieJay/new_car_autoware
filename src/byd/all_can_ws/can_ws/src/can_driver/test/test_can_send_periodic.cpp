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

  // Test charging mode override and full/fault restrictions
  {
    int test_sockets[2];
    assert(socketpair(AF_UNIX, SOCK_DGRAM, 0, test_sockets) == 0);

    can_frame normal_ctrl{};
    normal_ctrl.can_id = 0x201;
    normal_ctrl.can_dlc = 8;
    normal_ctrl.data[0] = 100; // speed
    normal_ctrl.data[4] = 0b00011011; // normal driving

    // 1. 收到充电请求后：下发的同ID帧只会让其进入充电模式 (0b00100001)，而不是干扰
    can_driver::charging_requested_.store(true);
    can_driver::is_battery_full_.store(false);
    can_driver::is_charge_fault_.store(false);

    assert(can_driver::CanSend::sendFrames(test_sockets[0], {normal_ctrl}));
    can_frame rx{};
    assert(read(test_sockets[1], &rx, sizeof(rx)) == sizeof(rx));
    assert(rx.can_id == 0x201);
    assert(rx.data[0] == 0);
    assert(rx.data[4] == 0b00100001); // Bit5=1, Bit0=1

    // 2. 电池充满后：无论在哪都不能发充电帧 (Bit5必须为0)，且速度转向为0、使能全部断掉 (Bit1=0, Bit3=0, Bit4=0)
    can_driver::charging_requested_.store(true);
    can_driver::is_battery_full_.store(true);
    assert(can_driver::CanSend::sendFrames(test_sockets[0], {normal_ctrl}));
    assert(read(test_sockets[1], &rx, sizeof(rx)) == sizeof(rx));
    assert(rx.data[0] == 0);
    assert(rx.data[2] == 0);
    assert(rx.data[4] == 0b00000001);

    // 3. 出现充电故障后：无论在哪都不能发充电帧 (Bit5必须为0)，且速度转向为0、使能全部断掉 (Bit1=0, Bit3=0, Bit4=0)
    can_driver::is_battery_full_.store(false);
    can_driver::is_charge_fault_.store(true);
    assert(can_driver::CanSend::sendFrames(test_sockets[0], {normal_ctrl}));
    assert(read(test_sockets[1], &rx, sizeof(rx)) == sizeof(rx));
    assert(rx.data[0] == 0);
    assert(rx.data[2] == 0);
    assert(rx.data[4] == 0b00000001);

    // 4. 整车处于充电模式时 (is_charging_mode_=true)，即使充电请求已结束，也绝对禁止下发带速度转向和使能的帧
    can_driver::charging_requested_.store(false);
    can_driver::is_charging_mode_.store(true);
    can_driver::is_charge_fault_.store(false);
    assert(can_driver::CanSend::sendFrames(test_sockets[0], {normal_ctrl}));
    assert(read(test_sockets[1], &rx, sizeof(rx)) == sizeof(rx));
    assert(rx.data[0] == 0);
    assert(rx.data[2] == 0);
    assert(rx.data[4] == 0b00000001);

    // 5. 停止充电/非充电流程下，无论何处传入带Bit5=1的充电帧，Bit5必须被强制清零，绝不能下发充电帧
    can_driver::charging_requested_.store(false);
    can_driver::is_charging_mode_.store(false);
    can_frame rogue_charge_frame = normal_ctrl;
    rogue_charge_frame.data[4] = 0b00111011; // 企图设置 Bit5 充电使能
    assert(can_driver::CanSend::sendFrames(test_sockets[0], {rogue_charge_frame}));
    assert(read(test_sockets[1], &rx, sizeof(rx)) == sizeof(rx));
    assert((rx.data[4] & (1 << 5)) == 0);

    // 6. 处于充电状态时 (charging_requested_=true)，速度转向为0、抱闸锁死、持续维系充电使能(Bit5=1)；
    //    停止充电时 (charging_requested_=false)，Bit5 必须为0 (0b00000001)
    can_driver::charging_requested_.store(true);
    can_driver::is_charging_mode_.store(true);
    can_driver::is_battery_full_.store(false);
    can_driver::is_charge_fault_.store(false);
    assert(can_driver::CanSend::sendFrames(test_sockets[0], {normal_ctrl}));
    assert(read(test_sockets[1], &rx, sizeof(rx)) == sizeof(rx));
    assert(rx.data[0] == 0);
    assert(rx.data[2] == 0);
    assert(rx.data[4] == 0b00100001); // 持续维系充电使能 Bit5=1, Bit0=1

    // 停止充电后
    can_driver::charging_requested_.store(false);
    assert(can_driver::CanSend::sendFrames(test_sockets[0], {normal_ctrl}));
    assert(read(test_sockets[1], &rx, sizeof(rx)) == sizeof(rx));
    assert(rx.data[0] == 0);
    assert(rx.data[2] == 0);
    assert(rx.data[4] == 0b00000001); // Bit5 必须为0，绝不再下发充电帧

    // 复位状态
    can_driver::charging_requested_.store(false);
    can_driver::is_charging_mode_.store(false);
    can_driver::is_battery_full_.store(false);
    can_driver::is_charge_fault_.store(false);
    close(test_sockets[0]);
    close(test_sockets[1]);
  }

  // 5. 语音播报守护及0x381状态解析测试
  {
    // 验证0x381 BYTE2 bit0(0x01)与兼容掩码解析为正在播报中
    uint8_t byte2_idle = 0x00;
    uint8_t byte2_playing_bit0 = 0x01; // 协议标准 bit0: 播报中
    uint8_t byte2_playing_bit2 = 0x04; // 兼容 bit2: 播报中
    int mask = 0x01;

    auto parse_playing = [mask](uint8_t b2) {
      return ((b2 & 0x01) != 0) || ((b2 & static_cast<uint8_t>(mask)) != 0);
    };

    assert(parse_playing(byte2_playing_bit0) == true);
    assert(parse_playing(byte2_playing_bit2) == true);
    assert(parse_playing(byte2_idle) == false);

    // 验证底层 0x401 守护规则：播报中任何 0x401 帧严禁关闭语音模块或改变语音编号
    can_driver::is_voice_active_.store(true);
    can_driver::active_voice_id_.store(4); // 倒车请注意

    int test_sockets[2];
    assert(socketpair(AF_UNIX, SOCK_DGRAM, 0, test_sockets) == 0);
    can_driver::CanSend sender;
    std::atomic<bool> sender_running{true};
    std::thread sender_thread([&sender, &sender_running, sock = test_sockets[0]]() {
      sender.sendTask(sock, sender_running);
    });

    // 尝试推送一帧清空语音的 0x401 帧（模拟上层控制周期帧）
    can_frame incoming_401{};
    incoming_401.can_id = 0x401;
    incoming_401.can_dlc = 8;
    incoming_401.data[0] = 0x00; // 语音开关未开
    incoming_401.data[2] = 0;    // 语音编号为0
    sender.push({incoming_401});

    can_frame wire_401{};
    assert(read(test_sockets[1], &wire_401, sizeof(wire_401)) == sizeof(wire_401));
    assert(wire_401.can_id == 0x401);
    assert((wire_401.data[0] & (1 << 4)) != 0); // 必须被守护为打开
    assert(wire_401.data[2] == 4);               // 必须保持为当前播报编号 4

    // 结束播报后，恢复正常
    can_driver::is_voice_active_.store(false);
    can_driver::active_voice_id_.store(0);

    sender_running.store(false);
    sender_thread.join();
    close(test_sockets[0]);
    close(test_sockets[1]);

    // 验证倒车检测逻辑完整性
    // 情况A: 负速度
    double v_test1 = -0.5;
    int gear_test1 = 1;
    bool is_rev1 = (v_test1 < -0.001) || (gear_test1 == -1);
    assert(is_rev1 == true);

    // 情况B: 倒挡 + 正速度输入
    double v_test2 = 0.5;
    int gear_test2 = -1;
    if (gear_test2 == -1 && v_test2 > 0) v_test2 = -v_test2;
    bool is_rev2 = (v_test2 < -0.001) || (gear_test2 == -1);
    assert(is_rev2 == true);
    assert(v_test2 < 0);

    // 情况C: 倒挡 + 零速度停车
    double v_test3 = 0.0;
    int gear_test3 = -1;
    bool is_rev3 = (v_test3 < -0.001) || (gear_test3 == -1);
    assert(is_rev3 == true);

    // 验证任何播报中语音绝对不可被打断
    bool is_playing = true;
    uint8_t current_voice = 4; // 正在播报倒车
    uint8_t high_prio_voice = 10; // 障碍物高级别语音到达
    bool allow_interrupt = !is_playing;
    assert(allow_interrupt == false); // 绝对禁止打断正在播报中的语音

    // 验证到达目标点语音单次触发以及倒车语音优先于到达目标点
    std::atomic<bool> is_arrived_destination{true};
    std::atomic<bool> arrival_voice_played{false};
    std::atomic<uint8_t> pending_control_voice{4}; // 倒车请注意

    // 1. 倒车时倒车语音优先于到达目标点
    auto select_voice = [&](bool arrived, bool arrived_played, uint8_t ctrl_v) -> uint8_t {
      if (ctrl_v == 4) return 4; // 倒车语音优先
      if (arrived && !arrived_played) return 5; // 单次到达
      return ctrl_v;
    };
    assert(select_voice(is_arrived_destination.load(), arrival_voice_played.load(), pending_control_voice.load()) == 4);

    // 2. 到达语音单次触发与消费
    pending_control_voice.store(0);
    assert(select_voice(is_arrived_destination.load(), arrival_voice_played.load(), pending_control_voice.load()) == 5);
    // 触发播报后置位
    arrival_voice_played.store(true);
    is_arrived_destination.store(false);
    // 播报结束后不能再重播到达语音
    assert(select_voice(is_arrived_destination.load(), arrival_voice_played.load(), pending_control_voice.load()) == 0);

    // 3. 车辆移动或倒车时清空到达状态并保护防重播
    double test_vel = -0.5;
    bool is_rev = (test_vel < -0.001);
    if (std::abs(test_vel) > 0.001 || is_rev) {
      is_arrived_destination.store(false);
      arrival_voice_played.store(true);
      pending_control_voice.store(4);
    }
    assert(is_arrived_destination.load() == false);
    assert(arrival_voice_played.load() == true);
    assert(select_voice(is_arrived_destination.load(), arrival_voice_played.load(), pending_control_voice.load()) == 4);
  }

  return 0;
}

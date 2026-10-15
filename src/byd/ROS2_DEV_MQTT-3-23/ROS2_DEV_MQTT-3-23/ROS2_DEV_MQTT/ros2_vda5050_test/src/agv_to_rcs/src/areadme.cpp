git用法：
1.git add .
2.git commit -m "project can
这两步会记录本地的变更，下拉云端时不会覆盖这些
3.git pull --rebase origin main下拉云端覆盖本地和自己改动无关的部分
4.git push把融合自己代码的部分推上去
升起
cansend can0 401#0000000001000000
下降
cansend can0 401#0000000002000000
candump -L can1,603:7FF | pv -l -i 1 > /dev/null报文频率
python3 src/byd/scripts/monitor_hook_can.py
sudo apt update
sudo apt install -y mosquitto-clients
mosquitto_sub -h agv.byd.com -p 1883   -u 'root' -P 'Root1234#$'   -t 'uagv/v1/BYD/qqa0001/#'   -v
如果mqtt服务器设置了开启了 ACL（权限控制），订阅被静默拒绝，拒绝授权，不会报错退出，也不会打印任何错误日志，而是默默挂起等待
QoS 0 报文会被服务端直接丢弃，并且客户端也不会收到任何错误，手动发也收不到
sudo tcpdump -i any -nn -A -l 'port 1883 and dst host 202.103.149.205' | grep --line-buffered -o '{.*}'
它会把底层的 TCP 头等乱码全部过滤掉，在终端上以 10Hz 实时滚屏打印出你的 AGV 每一帧发出的完整 JSON 数据，效果和 mosquitto_sub 完全一样。
0x181的byte4的bit6=1为起升到位，bit7=1为下降到位

到达
ros2 topic pub /byd/autoware/state autoware_system_msgs/msg/AutowareState "{stamp: {sec: 0, nanosec: 0}, state: 6}" --once
货插客户端没动作，就是还没走到位置，光有6state也没用



目前在control_execute_action.cpp中，会根据LOAD，UNLOAD和pick，drop的动作类型，强制重置高度，高度为1是升起挂钩，高度为2是下降挂钩
HARDACTION带动作的点是不能抢占的，必须要等到上一个点执行完毕了，才能接受下一个任务
if_reach_point和agv_driver_control->get_flag_driving()这个由state=6触发的有什么区别，为什么要用两个，if_reach_point什么条件为1
满足这两个条件，才能
if_reach_point或许是由精度决定的，因为之前state=6不触发货插，只有到了目标点，并且state=6才触发货插


下发目标点如果can驱动没启动,发布任务也不会下发目标点，是因为挂钩驱动卡住了吗？
卸货，挂钩下降，目前发来的命令里面是反的
ros2 topic pub --once /uagv/v1/BYD/qqa0001/order vda5050_interfaces/msg/AGVOrder "{header_id: 1, timestamp: '2026-07-20T14:04:11.185981+08:00', version: v1, manufacturer: BYD, serial_number: qqa0001, order_id: 'task-p2p-20260720-O51P', order_update_id: 0, zone_set_id: map_floor_1, nodes: [{node_id: '1#电池卸料点1', sequence_id: 0, released: true, node_position: {x: 299.183135986328, y: -61.0916442871094, theta: -1.07944893836975, map_id: '', map_description: '', allowed_deviation_xy: 0.5, allowed_deviation_theta: 5.0}, actions: []}, {node_id: '1#电池卸料点2', sequence_id: 2, released: true, node_position: {x: 294.183135986328, y: -61.0916442871094, theta: -1.07944893836975, map_id: '', map_description: '', allowed_deviation_xy: 0.5, allowed_deviation_theta: 5.0}, actions: [{action_type: UNLOAD, action_id: 'drop-action-001', action_description: '', blocking_type: HARD, action_parameters: [{key: height, value: {array_value: [], boolean_value: false, number_value: 1.0, string_value: ''}}]}]}], edges: [{edge_id: 'edge_to_2', sequence_id: 1, edge_description: '', released: true, start_node_id: '1#电池卸料点1', end_node_id: '1#电池卸料点2', max_speed: 1.0, max_height: 0.0, orientation: 0.0, orientation_type: TANGENTIAL, direction: '', rotation_allowed: false, max_rotation_speed: 0.0, length: 0.0, obstacle_avoidance_channel: 0, trajectory: {degree: 1, knot_vector: [], control_points: []}, actions: []}]}"
取货
ros2 topic pub --once /uagv/v1/BYD/qqa0001/order vda5050_interfaces/msg/AGVOrder "{header_id: 1, timestamp: '2026-07-15T16:27:46.909346+08:00', version: v1, manufacturer: BYD, serial_number: qqa0001, order_id: 'task-p2p-20260715-EL05', order_update_id: 0, zone_set_id: map_floor_1, nodes: [{node_id: '料点', sequence_id: 0, released: true, node_position: {x: 167.678558349609, y: -194.128631591797, theta: -2.69616770744324, map_id: '', map_description: '', allowed_deviation_xy: 0.5, allowed_deviation_theta: 5.0}, actions: []}, {node_id: '2#后段卸料点', sequence_id: 2, released: true, node_position: {x: 167.678558349609, y: -194.128631591797, theta: -2.69616770744324, map_id: '', map_description: '', allowed_deviation_xy: 0.5, allowed_deviation_theta: 5.0}, actions: [{action_type: LOAD, action_id: 'pick-action-001', action_description: '', blocking_type: HARD, action_parameters: [{key: height, value: {array_value: [], boolean_value: false, number_value: 1.0, string_value: ''}}]}]}], edges: [{edge_id: 'edge_01_料点', sequence_id: 1, edge_description: '', released: true, start_node_id: '01', end_node_id: '2#后段卸料点', max_speed: 1.0, max_height: 0.0, orientation: 0.0, orientation_type: TANGENTIAL, direction: '', rotation_allowed: false, max_rotation_speed: 0.0, length: 0.0, obstacle_avoidance_channel: 0, trajectory: {degree: 1, knot_vector: [], control_points: []}, actions: []}]}"


// 暂停
ros2 topic pub --once /uagv/v1/BYD/qqa0001/instantActions vda5050_interfaces/msg/AGVInstantActions "{header_id: 1001, timestamp: '2026-07-23T10:00:00Z', version: 'v1', manufacturer: 'BYD', serial_number: 'qqa0001', actions: [{action_type: 'startPause', action_id: 'b34ec9c6-28fc-4b12-8db0-000000000001', action_description: 'agv暂停', blocking_type: 'HARD', action_parameters: []}]}"
// 恢复
ros2 topic pub --once /uagv/v1/BYD/qqa0001/instantActions vda5050_interfaces/msg/AGVInstantActions "{header_id: 1001, timestamp: '2026-07-23T10:00:00Z', version: 'v1', manufacturer: 'BYD', serial_number: 'qqa0001', actions: [{action_type: 'stopPause', action_id: 'b34ec9c6-28fc-4b12-8db0-000000000001', action_description: 'agv恢复', blocking_type: 'HARD', action_parameters: []}]}"


取消任务,通用
ros2 topic pub --once /uagv/v1/BYD/qqa0001/instantActions vda5050_interfaces/msg/AGVInstantActions "{header_id: 2, timestamp: '2026-09-16T10:45:00.000000+08:00', version: v1, manufacturer: BYD, serial_number: qqa0001, actions: [{action_type: cancelOrder, action_id: cancel_001, blocking_type: HARD}]}"


充电需要两条命令配合，先给一个带动作的order走到动作位置，然后再给一个instantaction，改变状态到充电
ros2 topic pub --once /uagv/v1/BYD/qqa0001/order vda5050_interfaces/msg/AGVOrder "{header_id: 0, timestamp: '2026-08-05T10:00:00+08:00', version: 'v1', manufacturer: 'BYD', serial_number: 'qqa0001', order_id: 'b32d7a42-2b1e-4b12-9531-31b6f226cee1', order_update_id: 0, zone_set_id: '22', nodes: [{node_id: '22_54', sequence_id: 0, released: true, actions: [], node_position: {x: 167.678558349609, y: -194.128631591797, theta: -2.69616770744324, map_id: '22', allowed_deviation_xy: 1.5, allowed_deviation_theta: 0.5}}, {node_id: '22_53', sequence_id: 2, released: true, actions: [{action_type: 'CHARGE', action_id: '5d962a1c-40e4-4a7d-9006-700720711bc6', blocking_type: 'HARD', action_parameters: []}], node_position: {x: 167.678558349609, y: -194.128631591797, theta: -2.69616770744324, map_id: '22', allowed_deviation_xy: 1.5, allowed_deviation_theta: 0.5}}], edges: [{edge_id: '22_54-22_53', sequence_id: 1, released: true, start_node_id: '22_54', end_node_id: '22_53', actions: [], max_speed: 1.0, orientation: 0.0, orientation_type: 'TANGENTIAL', rotation_allowed: true, trajectory: {degree: 1, knot_vector: [], control_points: [{x: 167.678558349609, y: -194.128631591797, weight: 1.0}, {x: 167.678558349609, y: -194.128631591797, weight: 1.0}]}, obstacle_avoidance_channel: 0}]}"

第一条命令到达目标点后，进入充电函数，但是函数中进入阻塞态，只有instantActions才会改变标志位，从阻塞中唤醒，开始尝试联系充电服务端
ros2 topic pub /uagv/v1/BYD/qqa0001/instantActions vda5050_interfaces/msg/AGVInstantActions "{header_id: 1, timestamp: '2026-08-05T10:02:00.000000', version: 'v1', manufacturer: 'BYD', serial_number: 'qqa0001', actions: [{action_type: 'start_charge', action_id: 'f9459907-a47a-49ca-a19e-c1aa7c5f0862', action_description: '', blocking_type: 'HARD', action_parameters: []}]}" --once
如果充电服务端没有启动，发布这条命令就会报错


唐
ros2 topic pub --once /uagv/v1/BYD/qqa0001/order vda5050_interfaces/msg/AGVOrder "{header_id: 1, timestamp: '2026-07-20T14:04:11.185981+08:00', version: v1, manufacturer: BYD, serial_number: qqa0001, order_id: 'task-p2p-20260720-O51P', order_update_id: 0, zone_set_id: map_floor_1, nodes: [{node_id: '1#电池卸料点1', sequence_id: 0, released: true, node_position: {x: 299.183135986328, y: -61.0916442871094, theta: -1.07944893836975, map_id: '', map_description: '', allowed_deviation_xy: 0.5, allowed_deviation_theta: 5.0}, actions: []}, {node_id: '1#电池卸料点2', sequence_id: 2, released: true, node_position: {x: 294.183135986328, y: -61.0916442871094, theta: -1.07944893836975, map_id: '', map_description: '', allowed_deviation_xy: 0.5, allowed_deviation_theta: 5.0}, actions: [{action_type: UNLOAD, action_id: 'drop-action-001', action_description: '', blocking_type: HARD, action_parameters: [{key: height, value: {array_value: [], boolean_value: false, number_value: 1.0, string_value: ''}}]}]}], edges: [{edge_id: 'edge_to_2', sequence_id: 1, edge_description: '', released: true, start_node_id: '1#电池卸料点1', end_node_id: '1#电池卸料点2', max_speed: 1.0, max_height: 0.0, orientation: 0.0, orientation_type: TANGENTIAL, direction: '', rotation_allowed: false, max_rotation_speed: 0.0, length: 0.0, obstacle_avoidance_channel: 0, trajectory: {degree: 1, knot_vector: [], control_points: []}, actions: []}]}"


// 挂钩bug
// [INFO] [1788229679.124633095] [can_action_server]: 收到挂钩目标: signal=1, target_height=102
// [INFO] [1788229679.124912100] [can_action_server]: 接受挂钩目标，开始在新线程中执行
// [INFO] [1788229679.125095266] [can_action_server]: 开始执行挂钩: 目标高度=102.00, 信号=1
// [ERROR] [1788229679.125184993] [can_action_server]: RCS挂钩指令异常: 不支持的目标高度 102



[INFO] [1788397600.267339126] [can_action_server]: 收到挂钩目标: signal=1, target_height=1
[INFO] [1788397600.267791635] [can_action_server]: 接受挂钩目标，开始在新线程中执行
[INFO] [1788397600.268217713] [can_action_server]: 开始执行挂钩: 目标高度=1.00, 信号=1
[INFO] [1788397600.268342032] [can_action_server]: 下达挂钩升降到位置1命令 (CAN ID:0x401, data[4]=1)
[INFO] [1788397600.268363504] [can_action_server]: 挂钩CAN帧已成功发送到发送队列


倒车
order要求rotation_allowed必须为true，orientation为3.141592653589793
ros2 topic pub --once /uagv/v1/BYD/qqa0001/order vda5050_interfaces/msg/AGVOrder "{header_id: 1, timestamp: '2026-07-20T14:04:11.185981+08:00', version: v1, manufacturer: BYD, serial_number: qqa0001, order_id: 'task-p2p-20260720-O51P', order_update_id: 0, zone_set_id: map_floor_1, nodes: [{node_id: '1#电池卸料点1', sequence_id: 0, released: true, node_position: {x: 299.183135986328, y: -61.0916442871094, theta: -1.07944893836975, map_id: '', map_description: '', allowed_deviation_xy: 0.5, allowed_deviation_theta: 5.0}, actions: []}, {node_id: '1#电池卸料点2', sequence_id: 2, released: true, node_position: {x: 294.183135986328, y: -61.0916442871094, theta: -1.07944893836975, map_id: '', map_description: '', allowed_deviation_xy: 0.5, allowed_deviation_theta: 5.0}, actions: [{action_type: UNLOAD, action_id: 'drop-action-001', action_description: '', blocking_type: HARD, action_parameters: [{key: height, value: {array_value: [], boolean_value: false, number_value: 1.0, string_value: ''}}]}]}], edges: [{edge_id: 'edge_to_2', sequence_id: 1, edge_description: '', released: true, start_node_id: '1#电池卸料点1', end_node_id: '1#电池卸料点2', max_speed: 1.0, max_height: 0.0, orientation: 3.141592653589793, orientation_type: TANGENTIAL, direction: '', rotation_allowed: true, max_rotation_speed: 0.0, length: 0.0, obstacle_avoidance_channel: 0, trajectory: {degree: 1, knot_vector: [], control_points: []}, actions: []}]}"
必须启动倒车服务端
ros2 launch agv_high_precision_reverse_controller agv_high_precision_reverse_controller.launch.py
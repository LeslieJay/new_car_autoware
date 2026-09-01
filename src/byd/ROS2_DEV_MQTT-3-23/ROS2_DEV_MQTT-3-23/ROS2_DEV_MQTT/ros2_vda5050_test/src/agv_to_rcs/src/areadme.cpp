ros2 topic pub /byd/autoware/state autoware_system_msgs/msg/AutowareState "{stamp: {sec: 0, nanosec: 0}, state: 6}" --once


HARDACTION带动作的点是不能抢占的，必须要等到上一个点执行完毕了，才能接受下一个任务
if_reach_point和agv_driver_control->get_flag_driving()这个由state=6触发的有什么区别，为什么要用两个，if_reach_point什么条件为1
满足这两个条件，才能
if_reach_point或许是由精度决定的，因为之前state=6不触发货插，只有到了目标点，并且state=6才触发货插
// 连续测试


ros2 topic pub --once /uagv/v1/BYD/qqa0001/order vda5050_interfaces/msg/AGVOrder "{header_id: 1, timestamp: '2026-07-20T14:04:11.185981+08:00', version: v1, manufacturer: BYD, serial_number: qqa0001, order_id: 'task-p2p-20260720-O51P', order_update_id: 0, zone_set_id: map_floor_1, nodes: [{node_id: '1#电池卸料点1', sequence_id: 0, released: true, node_position: {x: 299.183135986328, y: -61.0916442871094, theta: -1.07944893836975, map_id: '', map_description: '', allowed_deviation_xy: 0.5, allowed_deviation_theta: 5.0}, actions: []}, {node_id: '1#电池卸料点2', sequence_id: 2, released: true, node_position: {x: 294.183135986328, y: -61.0916442871094, theta: -1.07944893836975, map_id: '', map_description: '', allowed_deviation_xy: 0.5, allowed_deviation_theta: 5.0}, actions: [{action_type: UNLOAD, action_id: 'drop-action-001', action_description: '', blocking_type: HARD, action_parameters: [{key: height, value: {array_value: [], boolean_value: false, number_value: 1.0, string_value: ''}}]}]}], edges: [{edge_id: 'edge_to_2', sequence_id: 1, edge_description: '', released: true, start_node_id: '1#电池卸料点1', end_node_id: '1#电池卸料点2', max_speed: 1.0, max_height: 0.0, orientation: 0.0, orientation_type: TANGENTIAL, direction: '', rotation_allowed: false, max_rotation_speed: 0.0, length: 0.0, obstacle_avoidance_channel: 0, trajectory: {degree: 1, knot_vector: [], control_points: []}, actions: []}]}"

ros2 topic pub --once /uagv/v1/BYD/qqa0001/order vda5050_interfaces/msg/AGVOrder "{header_id: 1, timestamp: '2026-07-15T16:27:46.909346+08:00', version: v1, manufacturer: BYD, serial_number: qqa0001, order_id: 'task-p2p-20260715-EL05', order_update_id: 0, zone_set_id: map_floor_1, nodes: [{node_id: '料点', sequence_id: 0, released: true, node_position: {x: 167.678558349609, y: -194.128631591797, theta: -2.69616770744324, map_id: '', map_description: '', allowed_deviation_xy: 0.5, allowed_deviation_theta: 5.0}, actions: []}, {node_id: '2#后段卸料点', sequence_id: 2, released: true, node_position: {x: 167.678558349609, y: -194.128631591797, theta: -2.69616770744324, map_id: '', map_description: '', allowed_deviation_xy: 0.5, allowed_deviation_theta: 5.0}, actions: [{action_type: LOAD, action_id: 'pick-action-001', action_description: '', blocking_type: HARD, action_parameters: [{key: height, value: {array_value: [], boolean_value: false, number_value: 1.0, string_value: ''}}]}]}], edges: [{edge_id: 'edge_01_料点', sequence_id: 1, edge_description: '', released: true, start_node_id: '01', end_node_id: '2#后段卸料点', max_speed: 1.0, max_height: 0.0, orientation: 0.0, orientation_type: TANGENTIAL, direction: '', rotation_allowed: false, max_rotation_speed: 0.0, length: 0.0, obstacle_avoidance_channel: 0, trajectory: {degree: 1, knot_vector: [], control_points: []}, actions: []}]}"

// 暂停
ros2 topic pub --once /uagv/v1/BYD/qqa0001/instantActions vda5050_interfaces/msg/AGVInstantActions "{header_id: 1001, timestamp: '2026-07-23T10:00:00Z', version: 'v1', manufacturer: 'BYD', serial_number: 'qqa0001', actions: [{action_type: 'startPause', action_id: 'b34ec9c6-28fc-4b12-8db0-000000000001', action_description: 'agv暂停', blocking_type: 'HARD', action_parameters: []}]}"
// 恢复
ros2 topic pub --once /uagv/v1/BYD/qqa0001/instantActions vda5050_interfaces/msg/AGVInstantActions "{header_id: 1001, timestamp: '2026-07-23T10:00:00Z', version: 'v1', manufacturer: 'BYD', serial_number: 'qqa0001', actions: [{action_type: 'stopPause', action_id: 'b34ec9c6-28fc-4b12-8db0-000000000001', action_description: 'agv恢复', blocking_type: 'HARD', action_parameters: []}]}"


充电需要两条命令配合，先给一个带动作的order走到动作位置，然后再给一个instantaction，改变状态到充电
ros2 topic pub --once /uagv/v1/BYD/qqa0001/order vda5050_interfaces/msg/AGVOrder "{header_id: 0, timestamp: '2026-08-05T10:00:00+08:00', version: 'v1', manufacturer: 'BYD', serial_number: 'qqa0001', order_id: 'b32d7a42-2b1e-4b12-9531-31b6f226cee1', order_update_id: 0, zone_set_id: '22', nodes: [{node_id: '22_54', sequence_id: 0, released: true, actions: [], node_position: {x: 167.678558349609, y: -194.128631591797, theta: -2.69616770744324, map_id: '22', allowed_deviation_xy: 1.5, allowed_deviation_theta: 0.5}}, {node_id: '22_53', sequence_id: 2, released: true, actions: [{action_type: 'CHARGE', action_id: '5d962a1c-40e4-4a7d-9006-700720711bc6', blocking_type: 'HARD', action_parameters: []}], node_position: {x: 167.678558349609, y: -194.128631591797, theta: -2.69616770744324, map_id: '22', allowed_deviation_xy: 1.5, allowed_deviation_theta: 0.5}}], edges: [{edge_id: '22_54-22_53', sequence_id: 1, released: true, start_node_id: '22_54', end_node_id: '22_53', actions: [], max_speed: 1.0, orientation: 0.0, orientation_type: 'TANGENTIAL', rotation_allowed: true, trajectory: {degree: 1, knot_vector: [], control_points: [{x: 167.678558349609, y: -194.128631591797, weight: 1.0}, {x: 167.678558349609, y: -194.128631591797, weight: 1.0}]}, obstacle_avoidance_channel: 0}]}"

第一条命令到达目标点后，进入充电函数，但是函数中进入阻塞态，只有instantActions才会改变标志位，从阻塞中唤醒，开始尝试联系充电服务端
ros2 topic pub /uagv/v1/BYD/qqa0001/instantActions vda5050_interfaces/msg/AGVInstantActions "{header_id: 1, timestamp: '2026-08-05T10:02:00.000000', version: 'v1', manufacturer: 'BYD', serial_number: 'qqa0001', actions: [{action_type: 'start_charge', action_id: 'f9459907-a47a-49ca-a19e-c1aa7c5f0862', action_description: '', blocking_type: 'HARD', action_parameters: []}]}" --once
如果充电服务端没有启动，发布这条命令就会报错


唐
// 挂钩bug
// [INFO] [1788229679.124633095] [can_action_server]: 收到挂钩目标: signal=1, target_height=102
// [INFO] [1788229679.124912100] [can_action_server]: 接受挂钩目标，开始在新线程中执行
// [INFO] [1788229679.125095266] [can_action_server]: 开始执行挂钩: 目标高度=102.00, 信号=1
// [ERROR] [1788229679.125184993] [can_action_server]: RCS挂钩指令异常: 不支持的目标高度 102


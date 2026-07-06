pose:
    position:
      x: 248.853203697923
      y: -35.102312131853495
      z: 89.00000000004242
    orientation:
      x: -0.001393567368610207
      y: 0.006568142568843616
      z: 0.052438236549295615
      w: 0.9986015966444199
nvidia@nvidia-desktop:~/autoware$ ros2 topic echo /autoware_orientation --once
header:
  stamp:
    sec: 1782974977
    nanosec: 941000000
  frame_id: imu_link
orientation:
  orientation:
    x: -0.00028018181173282593
    y: 0.007183233189816486
    z: 0.22027517445613193
    w: 0.975411282576439
  rmse_rotation_x: 0.0
  rmse_rotation_y: 0.0
  rmse_rotation_z: 0.0
---
nvidia@nvidia-desktop:~/autoware$ ros2 topic echo /localization/kinematic_state --once
header:
  stamp:
    sec: 1782975012
    nanosec: 479866456
  frame_id: map
child_frame_id: base_link
pose:
  pose:
    position:
      x: 209.4239128112549
      y: -53.053671213201106
      z: 89.32000118707353
    orientation:
      x: -0.0002720825361506866
      y: 0.007269028948839746
      z: 0.21665391991236238
      w: 0.976221371501281
  covariance:
  - 0.00017669837385682346
  - 8.235681693847575e-05
  - 0.0
  - 0.0
  - 0.0
  - 3.5067997404763605e-09
  - 8.235681693847573e-05
  - 3.875322072565291e-05
  - 0.0
  - 0.0
  - 0.0
  - -7.511492667922728e-09
  - 0.0
  - 0.0
  - 0.007269154915859539
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0004456758209912252
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0004456758209912252
  - 0.0
  - 3.506799740476363e-09
  - -7.51149266792272e-09
  - 0.0
  - 0.0
  - 0.0
  - 5.367106550667068e-05
twist:
  twist:
    linear:
      x: 0.0
      y: 0.0
      z: 0.0
    angular:
      x: 0.0
      y: 0.0
      z: 0.0
  covariance:
  - 0.07629468764108821
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - -8.132760850702669e-21
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - -8.132760849993084e-21
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.010930846184597293
---
nvidia@nvidia-desktop:~/autoware$ ros2 topic echo /autoware_orientation --once
header:
  stamp:
    sec: 1782975234
    nanosec: 878000000
  frame_id: imu_link
orientation:
  orientation:
    x: -0.00029940444591489623
    y: 0.0072683538545670195
    z: 0.22027514916691066
    w: 0.9754106520069985
  rmse_rotation_x: 0.0
  rmse_rotation_y: 0.0
  rmse_rotation_z: 0.0
---
nvidia@nvidia-desktop:~/autoware$ ros2 topic echo /localization/kinematic_state --once
header:
  stamp:
    sec: 1782975278
    nanosec: 79660923
  frame_id: map
child_frame_id: base_link
pose:
  pose:
    position:
      x: 209.38516930067254
      y: -53.06102971209197
      z: 89.27999999999959
    orientation:
      x: -0.00027220232597496396
      y: 0.007269423407063818
      z: 0.21662337271267124
      w: 0.9762281474039814
  covariance:
  - 0.0001410074332582632
  - 6.578609571636674e-05
  - 0.0
  - 0.0
  - 0.0
  - 1.1137848550598436e-11
  - 6.578609571636673e-05
  - 3.0787660978421304e-05
  - 0.0
  - 0.0
  - 0.0
  - -2.38598229551574e-11
  - 0.0
  - 0.0
  - 0.007257962398944151
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.00044521544137978445
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.00044521544137978445
  - 0.0
  - 1.1137848550597685e-11
  - -2.3859822955155796e-11
  - 0.0
  - 0.0
  - 0.0
  - 4.0726629832631134e-05
twist:
  twist:
    linear:
      x: 0.0
      y: 0.0
      z: 0.0
    angular:
      x: 0.0
      y: 0.0
      z: 0.0
  covariance:
  - 0.07578727347636041
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 2.263076608199865e-21
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 2.2630766062762914e-21
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.011584242206234037
---

点1：
ros2 topic pub --once /planning/mission_planning/goal geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: 'map'}, pose: {position: {x: 243.26363433552328, y: -37.141902093574636, z: 89.28999999999957}, orientation: {x: -0.0033794693663022498, y: 0.003372215997922873, z: 0.24660542673218327, w: 0.9691042105224306}}}"
  
倒车点1：
ros2 service call /reverse_parking_planner/set_goal_pose \
  reverse_parking_planner/srv/SetGoalPose \
  "{goal_pose: {header: {frame_id: 'map'}, pose: {position: {x: 242.385266, y: -37.619886, z: 89.290000}, orientation: {x: -0.0033794693663022498, y: 0.003372215997922873, z: 0.24660542673218327, w: 0.9691042105224306}}}}"
  
点2：
ros2 topic pub --once /planning/mission_planning/goal geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: 'map'}, pose: {position: {x: 261.444300524526, y: -28.175441045963222, z: 89.1799999991811}, orientation: {x: -0.0033794693663022498, y: 0.003372215997922873, z: 0.24660542673218327, w: 0.9691042105224306}}}"

倒车点2：
ros2 service call /reverse_parking_planner/set_goal_pose \
  reverse_parking_planner/srv/SetGoalPose \
  "{goal_pose: {header: {frame_id: 'map'}, pose: {position: {x: 260.538544, y: -28.599240, z: 89.180000}, orientation: {x: -0.003919807370038459, y: 0.013610314398294736, z: 0.21705392966784673, w: 0.9760568559607102}}}}"

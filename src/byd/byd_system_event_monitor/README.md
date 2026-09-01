# BYD system event monitor

This node publishes `byd_vehicle_msgs/msg/EventTrigger` on `/system/event_trigger` for:

- `abnormal_stop`: the vehicle remains near zero speed in full autonomous mode while it is not
  at the goal and no fresh planned-stop reason exists;
- `autonomous_to_manual`: full autonomous mode reaches stable manual mode directly or through a
  configured two-second intermediate transition window.

Both detectors are edge-triggered. A continuous stop or manual state emits one event, not a
periodic stream.

The default `byd_event_rosbag_recorder` trigger policy accepts exactly these two event types and
keeps diagnostic and manual-service triggers disabled. That policy is configurable in the
recorder's `triggers` parameters.

```bash
ros2 launch byd_system_event_monitor system_event_monitor.launch.py
```

The top-level BYD bringup keeps the monitor disabled by default. Enable it together with the event
recorder to retain the configured pre/post-event bag window:

```bash
enable_system_event_monitor:=true enable_event_rosbag_recorder:=true
```

Before vehicle deployment, verify that the configured central stop-reason topic publishes at
least once per `freshness.stop_reason_seconds`; stale required inputs deliberately suspend
abnormal-stop detection and appear as `WARN` on `/diagnostics`.

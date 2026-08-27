# BYD event rosbag recorder

`event_rosbag_recorder_node` keeps a rolling window of configured ROS 2 topics and writes an
MCAP bag when an explicit event or matching diagnostic error occurs. The default event window is
30 seconds before the first trigger through 30 seconds after the last merged trigger.

## Start

```bash
ros2 launch byd_event_rosbag_recorder event_rosbag_recorder.launch.py \
  event_rosbag_recorder_param_file:=/path/to/event_rosbag_recorder.param.yaml
```

The BYD top-level bringup also accepts:

```bash
enable_event_rosbag_recorder:=true
event_rosbag_recorder_param_file:=/path/to/event_rosbag_recorder.param.yaml
```

The recorder is disabled by default in top-level bringup.

## Trigger

Publish `byd_vehicle_msgs/msg/EventTrigger` on `/system/event_trigger`, publish a matching
`ERROR` or `STALE` status on `/diagnostics`, or call:

```bash
ros2 service call /event_rosbag_recorder/trigger \
  byd_vehicle_msgs/srv/TriggerEvent \
  "{event_type: manual_test, severity: 2, description: operator_test}"
```

Completed event directories contain rosbag2 metadata, an MCAP file, and `event.json`. Directories
ending in `.inprogress` are never treated as completed events; remnants found at startup are
renamed with a `.corrupt` suffix for inspection.

## Resource bound

Serialized messages are bounded by both the configured pre-trigger duration and
`recording.queue_capacity_messages`. If the message-count bound is reached, the oldest message is
dropped and the drop counter is published in `/event_rosbag_recorder/status`. Size the bound from
measured aggregate topic rate before enabling high-bandwidth image or point-cloud topics.

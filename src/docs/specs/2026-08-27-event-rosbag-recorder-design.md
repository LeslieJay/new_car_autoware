# Event-triggered ROS Bag Recorder Design

## Goal

Add a ROS 2 recorder that continuously maintains a bounded pre-event buffer and, when an event
occurs, saves configured topics from 30 seconds before the first event through 30 seconds after
the last merged event. The recorder is an observability component: recording failures must be
reported without affecting vehicle control.

The implementation belongs to a new package:

```text
src/byd/byd_event_rosbag_recorder
```

The default launch configuration keeps the component disabled until its storage and performance
have been validated on the vehicle.

## Event Sources

The recorder accepts two event sources.

### Explicit events

Subscribe to `/system/event_trigger` using a new `byd_vehicle_msgs/msg/EventTrigger` interface:

```text
std_msgs/Header header
string event_id
string event_type
uint8 severity
string description

uint8 SEVERITY_INFO=0
uint8 SEVERITY_WARN=1
uint8 SEVERITY_ERROR=2
uint8 SEVERITY_STALE=3
```

Every received explicit event triggers recording. An empty `event_id` is accepted; the recorder
generates a UUID for metadata while preserving the original empty value.

### Diagnostic events

Subscribe to `/diagnostics` using `diagnostic_msgs/msg/DiagnosticArray`. A status triggers when:

1. its name matches at least one configured include expression;
2. its name matches no configured exclude expression; and
3. its level is at least the configured minimum level.

The defaults include all names, exclude the recorder's own diagnostic name, and trigger on
`ERROR` and `STALE`. `WARN` is opt-in through `min_level`.

Track the last level independently for every diagnostic status name. A transition from a level
below the threshold to a level at or above it creates an event. Repeated abnormal reports for
the same name extend an active event but do not create additional event records unless the
diagnostic message or level changes. A return below the threshold rearms the name.

The first observed status after startup triggers if it is already at or above the threshold.
If the pre-event buffer does not yet cover 30 seconds, mark the shortened window in event
metadata.

## Recording State Machine

Use the node's ROS clock for event and message-window decisions, respecting `use_sim_time`. Use
wall-clock UTC only for human-readable directory names.

The recorder has four externally observable states:

- `BUFFERING`: the rolling buffer is healthy and there is no active event.
- `CAPTURING`: at least one event is active and the post-event deadline has not passed.
- `FINALIZING`: buffered segments are being filtered into the final bag.
- `DEGRADED`: the recorder cannot guarantee capture because of configuration, storage, or writer
  failure.

The first trigger at ROS time `T0` changes `BUFFERING` to `CAPTURING` and fixes the requested
window start at `T0 - pre_trigger_seconds`. Each subsequent trigger at `Tn` is appended to the
same event manifest and moves the requested end to `Tn + post_trigger_seconds`.

When the end time passes, close the segment containing the end boundary and enter `FINALIZING`.
Read the retained source segments and write only messages whose receive timestamp is in the
inclusive interval:

```text
[T0 - pre_trigger_seconds, Tn + post_trigger_seconds]
```

Return to `BUFFERING` after the final bag and manifest are durable. Triggers received during
`FINALIZING` start a new logical capture backed by the still-running rolling writer; finalization
must not pause ingestion.

Cap one capture at `max_event_duration_seconds`, defaulting to 600 seconds. On reaching the cap,
close and finalize it. Diagnostic names that remain abnormal stay disarmed until they recover;
new explicit events and newly abnormal diagnostic names may start another capture.

## Rolling Buffer

Continuously serialize selected messages into MCAP segments under the configured temporary
directory. The default segment duration is five seconds. Keep enough closed segments to cover
`pre_trigger_seconds + segment_seconds`; retain the currently open segment as well.

Segment retention is reference based:

- ordinary closed segments older than the rolling horizon may be removed;
- segments intersecting an active or finalizing capture remain pinned;
- finalization releases its pins only after the final bag and metadata are durable.

The rolling writer and finalizer run on separate worker threads with bounded queues. Subscription
callbacks only timestamp and enqueue serialized messages. Queue overflow increments a per-topic
drop counter, emits a recorder diagnostic, and is recorded in the event manifest. It never blocks
vehicle callbacks indefinitely.

Use the rosbag2 receive timestamp as the authoritative filter timestamp. Preserve the serialized
message and original topic type without deserializing configured data topics.

## Topic Discovery and QoS

The topic selection configuration supports exact names plus include and exclude ECMAScript
regular expressions. Exact required topics are unioned with regex results, then exclusions are
applied. The following evidence topics are always selected and cannot be excluded:

```text
/system/event_trigger
/diagnostics
```

Discover types from the ROS graph and create generic subscriptions. Topics absent at startup
remain pending and are reconsidered on graph changes. A newly discovered matching topic begins
recording immediately. Record its effective start time in the manifest when it appears during a
capture.

A topic must resolve to exactly one type. If the same name advertises multiple types, or its type
changes after recording begins, reject that topic, retain recording for other topics, and publish
an error diagnostic.

Use a configured per-topic QoS override when present. Otherwise derive a compatible profile from
current publishers, preferring best-effort when publishers disagree on reliability and volatile
when they disagree on durability. Recreate a pending subscription when graph discovery first
provides enough QoS information; do not silently change an active subscription during a capture.

## Storage Format and Event Layout

Require `rosbag2_storage_mcap` and use MCAP chunk compression with Zstd at a low compression
level. Validate plugin availability before entering `BUFFERING`; do not silently fall back to
SQLite3 or another format.

Write each event to an `.inprogress` directory and atomically rename it after all files and the
directory entry have been synced:

```text
<output_directory>/
  20260827_143052_ERROR_brake_fault.inprogress/
    metadata.yaml
    event.json
    event_0.mcap
```

The final directory omits `.inprogress`. Sanitize the severity and event-type filename components
to ASCII letters, digits, `_`, and `-`; replace other runs with `_`. Resolve collisions by adding
a numeric suffix.

`metadata.yaml` is standard rosbag2 metadata. `event.json` contains:

- schema version;
- recorder package version and ROS distribution;
- first and last trigger times;
- requested and actual message-window bounds;
- whether the pre- or post-window is truncated and why;
- every merged explicit and diagnostic event in arrival order;
- selected topics, discovered types, QoS profiles, and effective recording intervals;
- per-topic received, written, and dropped counts;
- storage format and compression settings;
- finalization or recovery warnings.

The bag may naturally lack a message exactly at either boundary. Verification permits one normal
publication period of apparent edge error. Messages outside the requested interval are not
written merely because they shared a source segment.

## Recovery

On startup, inspect only paths owned by the recorder beneath the configured output and temporary
directories.

- Remove unpinned rolling segments left by the previous process after recording their discovery
  in diagnostics.
- Attempt to open and finalize each `.inprogress` event using its manifest and available MCAP
  data.
- Atomically rename a recoverable event to its final name.
- Rename an unrecoverable event to `.corrupt`, preserving its contents and adding the recovery
  failure to `event.json` when possible.

Recovery completes before the node enters `BUFFERING`. Failure to recover one old event does not
prevent new recording when the rolling buffer itself can start safely.

## Disk Governance

Before opening the rolling writer, require both a writable directory and at least
`minimum_free_space_gb`. During operation, monitor free space and total finalized-event usage.

When either limit is crossed, delete the oldest completed event directories until both limits are
satisfied. Active, `.inprogress`, `.corrupt`, and rolling-buffer paths are protected. Resolve and
validate every deletion candidate as a direct child of the configured output directory.

If cleanup cannot restore the safety margin, close the rolling writer, enter `DEGRADED`, and emit
an `ERROR` diagnostic. Periodically retry capacity checks and resume `BUFFERING` with a new
rolling segment when space becomes sufficient. Recorder degradation never terminates or commands
another vehicle node.

## ROS Interfaces and Observability

Publish the recorder's detailed health through `/diagnostics`, including state, selected and
missing topics, queue drops, buffer coverage, active event deadline, plugin status, free space,
and last writer error.

Also add these interfaces to `byd_vehicle_msgs`:

```text
# RecorderStatus.msg, published on /event_rosbag_recorder/status
std_msgs/Header header
uint8 state
string state_message
float64 buffer_coverage_seconds
uint32 selected_topic_count
uint32 missing_topic_count
uint64 dropped_message_count
float64 free_space_gb
string active_event_id

# ActiveEvent.msg, published on /event_rosbag_recorder/active_event
std_msgs/Header header
bool active
string primary_event_id
builtin_interfaces/Time window_start
builtin_interfaces/Time expected_window_end
uint32 merged_event_count

# TriggerEvent.srv, served at /event_rosbag_recorder/trigger
string event_type
uint8 severity
string description
---
bool accepted
string event_id
string message
```

Use transient-local reliable QoS for the two status topics so a late observer receives current
state. The trigger service creates the same internal event representation as
`/system/event_trigger`.

## Configuration

Install a parameter file with this shape and defaults:

```yaml
event_rosbag_recorder_node:
  ros__parameters:
    recording:
      pre_trigger_seconds: 30.0
      post_trigger_seconds: 30.0
      segment_seconds: 5.0
      max_event_duration_seconds: 600.0
      queue_capacity_messages: 10000

    topics:
      names: []
      include_regex: []
      exclude_regex: []

    diagnostics:
      min_level: 2
      include_names: [".*"]
      exclude_names: ["^event_rosbag_recorder($|:)"]
      trigger_on_transition: true

    storage:
      output_directory: /data/event_bags
      temporary_directory: /data/event_bags/.buffer
      minimum_free_space_gb: 20.0
      maximum_event_storage_gb: 200.0
      cleanup_policy: oldest_first
      storage_id: mcap
      compression: zstd
```

Represent QoS overrides in a separate installed YAML map keyed by exact topic name, passed as a
launch argument, because ROS parameter names cannot safely encode arbitrary topic names. Reject
invalid durations, regexes, paths, thresholds, unsupported cleanup policies, and malformed QoS
entries during construction. Configuration changes take effect after restart; runtime parameter
mutation is outside the first version.

## Launch Integration

Install a standalone launch file and add the recorder to `src/byd/launch/bringup.launch.py` behind
these arguments:

```text
enable_event_rosbag_recorder:=false
event_rosbag_recorder_param_file:=<installed default parameter file>
event_rosbag_recorder_qos_file:=<installed default QoS file>
```

The launch integration must leave current bringup behavior unchanged when the enable flag is
false.

## Verification

Automated verification must cover:

1. Exact-name and regex topic selection, required-topic inclusion, late discovery, ambiguous
   types, and QoS parsing.
2. Diagnostic threshold, include/exclude filtering, startup-abnormal behavior, recovery rearming,
   explicit triggers, and self-diagnostic exclusion.
3. State transitions, merged events, moving post-event deadline, triggers during finalization,
   and the ten-minute cap.
4. A deterministic integration test that publishes timestamped messages before and after a
   trigger, opens the resulting MCAP, and proves topic membership and inclusive time filtering.
5. Truncated pre-window behavior when triggering less than 30 seconds after startup.
6. Per-topic accounting and manifest contents when the ingestion queue overflows.
7. Unwritable directories, missing MCAP plugin, low disk space, cleanup ordering, protected paths,
   and recovery of `.inprogress` data.
8. Launch behavior with the enable flag both false and true.
9. A repeatable high-bandwidth test publisher for image or point-cloud-sized serialized messages,
   reporting ingestion rate, write rate, CPU, memory, disk throughput, and drops without requiring
   vehicle hardware.

Build and test at least `byd_vehicle_msgs`, `byd_event_rosbag_recorder`, and `byd_launch`. A test
run is complete only when the generated bag can be opened by rosbag2 tooling and all automated
tests pass.

## Non-goals

- Detecting domain-specific faults from raw sensor or vehicle data inside the recorder.
- Changing vehicle behavior, restarting failed nodes, or publishing control commands.
- Runtime hot-reload of topic, QoS, storage, or diagnostic configuration.
- Recording existing camera streams as MP4 or modifying `multi_recorder`.
- Uploading event bags to remote storage.
- Guaranteeing pre-trigger history that predates recorder startup or was never published.

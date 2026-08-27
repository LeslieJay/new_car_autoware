std_msgs/Header header
uint8 state
string state_message
float64 buffer_coverage_seconds
uint32 selected_topic_count
uint32 missing_topic_count
uint64 dropped_message_count
float64 free_space_gb
string active_event_id

uint8 STATE_BUFFERING=0
uint8 STATE_CAPTURING=1
uint8 STATE_FINALIZING=2
uint8 STATE_DEGRADED=3

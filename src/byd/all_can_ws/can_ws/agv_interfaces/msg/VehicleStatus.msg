# VehicleStatus.msg
# 小车状态消息，用于RCS系统实时监控小车状态

# 消息头部信息
int32 header_id
string timestamp
string version
string manufacturer
string serial_number

# 小车运行状态
# 0: OFFLINE(离线/灰灯), 1: ONLINE(在线/绿灯), 2: ERROR(错误/红灯), 3: WARNING(警告/红灯)
int32 status

# 错误代码和信息
int32 error_code        # 错误代码 (0表示无错误)
string error_message    # 错误描述信息
string warning_message  # 警告信息

# 系统状态详情
bool is_connected       # 是否连接到RCS
bool is_charging        # 是否正在充电
bool is_moving          # 是否正在移动
bool is_emergency_stop  # 是否急停状态

# 附加状态信息
float64 battery_level   # 电池电量百分比 (0-100)
string current_task     # 当前任务ID
string location         # 当前位置信息

# 时间戳（系统时间）
int64 system_timestamp  # 系统时间戳（毫秒）
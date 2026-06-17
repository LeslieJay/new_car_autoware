#!/usr/bin/env python3

import gi
gi.require_version('Gst', '1.0')
from gi.repository import Gst, GLib

import rclpy
from rclpy.node import Node
import time
import os
from autoware_vehicle_msgs.msg import  VelocityReport # 速度
from autoware_vehicle_msgs.msg import ControlModeReport  # 控制模式
from autoware_vehicle_msgs.msg import GearReport # 档位
from autoware_vehicle_msgs.msg import SteeringReport # 偏转角
from autoware_vehicle_msgs.msg import TurnIndicatorsReport # 转向灯
from rclpy.signals import SignalHandlerOptions # 新增引入
import signal
import threading
from datetime import datetime

import pyds

# ================= 全局状态 =================

class VehicleState:
    def __init__(self):
        self.mode = "UNKNOWN" 
        self.gear = "N" 
        self.speed = 0.0
        self.steer = 0.0
        self.turn = "NONE"      
        self.time = 0.0                     # 存储消息里的 ROS 时间戳

state = VehicleState()

# ================= OSD缓存 =================
cached_text = ""
last_osd_update = 0
cached_time_str = ""
last_time_update = 0

# ================= ROS2 Node =================

class RecorderNode(Node):
    def __init__(self):
        super().__init__('recorder_node')
        
        # --- 声明并获取参数 ---
        self.declare_parameter('video_device', '/dev/video0')
        self.declare_parameter('camera_id', 0)
        self.declare_parameter('data_dir', '/home/byd/vehicle_video_recorder/data')
        self.declare_parameter('width', 1920)
        self.declare_parameter('height', 1080)

        self.video_device = self.get_parameter('video_device').value
        self.camera_id = self.get_parameter('camera_id').value
        self.data_dir = self.get_parameter('data_dir').value
        self.width = self.get_parameter('width').value
        self.height = self.get_parameter('height').value


        # --- 为每路相机创建独立文件夹 ---
        self.output_dir = self.data_dir
        if not os.path.exists(self.output_dir):
            os.makedirs(self.output_dir, exist_ok=True)

        # --- 订阅车辆状态 ---
        # 订阅车辆控制模式
        self.create_subscription(ControlModeReport, '/vehicle/status/control_mode', self.mode_cb, 10)
        # 订阅档位
        self.create_subscription(GearReport, '/vehicle/status/gear_status', self.gear_cb, 10)
        # 订阅方向盘转角
        self.create_subscription(SteeringReport, '/vehicle/status/steering_status', self.steer_cb, 10)
        # 订阅速度
        self.create_subscription(VelocityReport, '/vehicle/status/velocity_status', self.vel_cb, 10)
        # 订阅转向灯状态
        self.create_subscription(TurnIndicatorsReport, '/vehicle/status/turn_indicators_status',self.turn_cb, 10)

        self.get_logger().info(f"Recorder Node Started for Camera {self.camera_id}")

    def mode_cb(self, msg):
        mapping = {
            1: "AUTO",   # 自动驾驶
            2: "AUTO_STEER",
            3: "AUTO_VEL",
            4: "MANUAL",  # 手动驾驶
            5: "DISENGAGED",
            6: "NOT_READY"
        }
        state.mode = mapping.get(msg.mode, "UNKNOWN")

    def gear_cb(self, msg):
        if msg.report == 2:
            state.gear = "D"   # 前进档
        elif msg.report == 20:
            state.gear = "R"   # 倒车档
        elif msg.report == 22:
            state.gear = "P"   # 停车档
        else:
            state.gear = "N"   # 空档   

    def steer_cb(self, msg):
        state.steer = msg.steering_tire_angle

    def vel_cb(self, msg):
        state.speed = msg.longitudinal_velocity * 3.6  # m/s 转 km/h
        # 使用速度消息的時間戳作為全局同步時間
        state.time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9


    def turn_cb(self, msg):
        if msg.report == 2:
            state.turn = "LEFT"
        elif msg.report == 3:
            state.turn = "RIGHT"
        else:
            state.turn = "NONE"

# ================= OSD =================

def osd_probe(pad, info, u_data):
    """
    单行底部水印（透明背景、白色小字体、居中）：
    顺序：模式 mode | 速度 Speed | 档位 Gear | 转向灯 Turn | 方向盘转角 Steer | 日期时间 Time
    """
    global cached_text, last_osd_update, cached_time_str, last_time_update

    try:
        print("[OSD] probe called")
    except Exception:
        pass

    gst_buffer = info.get_buffer()
    if not gst_buffer:
        print("[OSD] no gst_buffer")
        return Gst.PadProbeReturn.OK

    # 安全获取 batch_meta
    try:
        batch_meta = pyds.gst_buffer_get_nvds_batch_meta(hash(gst_buffer))
    except Exception as e:
        print(f"[OSD] gst_buffer_get_nvds_batch_meta exception: {e}")
        return Gst.PadProbeReturn.OK
    
    if not batch_meta:
        print("[OSD] no batch_meta")
        return Gst.PadProbeReturn.OK
    
    now = time.time()

    # ===== OSD 降频（10Hz）=====
    if now - last_osd_update > 0.1:

        # 更新时间（1Hz）
        if now - last_time_update > 1:
            cached_time_str = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
            last_time_update = now

        speed = state.speed
        gear = state.gear
        turn = state.turn
        steer = state.steer
        mode = state.mode

        cached_text = (
            f"{mode} | {speed:.0f}km/h | {gear} | {turn} | "
            f"{steer:.2f}° | {cached_time_str}"
        )

        last_osd_update = now
        try:
            print(f"[OSD] updated cached_text: {cached_text}")
        except Exception:
            pass

    # 遍历 batch 中每帧        
    l_frame = batch_meta.frame_meta_list
    if not l_frame:
        print("[OSD] batch_meta.frame_meta_list empty")
        return Gst.PadProbeReturn.OK

    
    while l_frame:
        try:
            frame_meta = pyds.NvDsFrameMeta.cast(l_frame.data)
        except Exception as e:
            print(f"[OSD] NvDsFrameMeta.cast exception: {e}")
            l_frame = l_frame.next
            continue

        if not frame_meta:
            l_frame = l_frame.next
            continue

        # 申请 display_meta（来自 pool）
        try:
            display_meta = pyds.nvds_acquire_display_meta_from_pool(batch_meta)
        except Exception as e:
            print(f"[OSD] acquire_display_meta_from_pool exception: {e}")
            l_frame = l_frame.next
            continue

        if not display_meta:
            print("[OSD] failed to acquire display_meta")
            l_frame = l_frame.next
            continue

        display_meta.num_labels = 1

        # 保护访问 text_params[0]
        try:
            txt = display_meta.text_params[0]
        except Exception as e:
            # text_params 访问失败：打印并跳过当前帧（常见于 pyds 绑定不一致）
            print(f"[OSD] text_params access exception: {e}")
            l_frame = l_frame.next
            continue

        txt.display_text = cached_text

        # ===== 动态字体（适配1080P/4K）=====
        frame_h = frame_meta.source_frame_height    # 读取当前帧的高度（像素）
        frame_w = frame_meta.source_frame_width     # 读取当前帧的宽度（像素）
        font_size = int(frame_h * 0.025)            # 根据帧高按比例计算字体大小（约占画面高度的 2.5%），并转换为整数。

        txt.font_params.font_name = "Sans"          # 设置字体名称为 Sans。
        txt.font_params.font_size = font_size       # 将上面计算的字号应用到 text 参数
        txt.font_params.font_color.set(1, 1, 1, 1)  # 设置字体颜色为不透明白色（RGBA 全 1）。

        txt.set_bg_clr = 1                          # 试图启用背景色；注意：pyds 绑定可能需要不同字段/方法来启用背景
        # txt.text_bg_clr.set(0, 0, 0, 0.35)        # 背景颜色设置，半透明黑背景。

        # 居中
        estimated_width = int(len(cached_text) * font_size * 0.55)  # 粗略估算文字宽度，用字符数 * 字号 * 比例因子（0.55）估算
        txt.x_offset = max(0, (frame_w - estimated_width) // 2)     # 使文本在水平方向居中（基于估算宽度）
        bottom_margin = int(frame_h * 0.03) # 预留 3% 的底部边距
        txt.y_offset = frame_h - font_size - bottom_margin                     # 设定文本纵坐标：位于底部，距底边上移 font_size+10 像素

        pyds.nvds_add_display_meta_to_frame(frame_meta, display_meta)

        l_frame = l_frame.next

    return Gst.PadProbeReturn.OK

# ================= Bus =================
def bus_call(bus, message, loop):
    t = message.type

    if t == Gst.MessageType.ERROR:
        err, debug = message.parse_error()
        print(f"[GStreamer ERROR] {err} | {debug}")
        loop.quit()

    elif t == Gst.MessageType.EOS:
        print("[GStreamer] EOS")
        loop.quit()

    return True

# ================= GStreamer =================
def build_pipeline(video_device, output_dir, width, height):
    """
    构建 GStreamer 视频录制流水线（GMSL2 相机适配）
    v4l2src → nvvidconv(UYVY→I420→NV12) → nvdsosd → nvv4l2h265enc → h265parse → splitmuxsink
    
    使用 v4l2src 读取 GMSL2 相机数据，经 nvvidconv 转换为 NVMM 格式，
    通过 nvdsosd 叠加车辆状态，H.265 编码分段录制。

    Returns:
        Gst.Pipeline: 配置好的 GStreamer 流水线
    """
    pipeline_str = f"""
    v4l2src device={video_device} !  
    video/x-raw, format=UYVY, width={width}, height={height}, framerate=30/1 !
    videorate drop-only=true !
    video/x-raw, framerate=10/1 !
    nvvidconv !
    video/x-raw(memory:NVMM), format=NV12 !
    m.sink_0 nvstreammux name=m batch-size=1 width={width} height={height} live-source=true !
    nvdsosd name=osd !
    nvv4l2h265enc bitrate=4000000 insert-sps-pps=true iframeinterval=10 !
    h265parse !
    splitmuxsink location={output_dir}/video_%05d.mp4
                 muxer=qtmux
                 max-size-time=10000000000
                 max-files=5
                 send-keyframe-requests=true
    """

    pipeline = Gst.parse_launch(pipeline_str)
    if not pipeline:
        raise RuntimeError("Pipeline create failed")

    return pipeline

# ================= main =================

def main():
    # 1. 必须在所有 ROS2 操作前禁用信号处理
    from rclpy.signals import SignalHandlerOptions
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    
    node = RecorderNode()

    # 从节点获取参数
    video_device = node.video_device
    output_dir = node.output_dir
    width, height = node.width, node.height

    Gst.init(None)
    try:
        pipeline = build_pipeline(video_device, output_dir, width, height)
    except Exception as e:
        print(f"Pipeline construction failed: {e}")
        return

    osd = pipeline.get_by_name("osd")
    if osd:
        pad = osd.get_static_pad("sink")
        pad.add_probe(Gst.PadProbeType.BUFFER, osd_probe, None)

    loop = GLib.MainLoop()

    # 2. 信号处理函数定义
    def signal_handler(sig, frame):
        print("\n[INFO] 正在安全停止录制并写入视频索引...")
        # 停止 GLib 主循环
        if loop.is_running():
            loop.quit()
        # pipeline.send_event(Gst.Event.new_eos())

    # 绑定信号
    signal.signal(signal.SIGINT, signal_handler)

    # 3. 启动总线监听
    bus = pipeline.get_bus()
    bus.add_signal_watch()
    bus.connect("message", bus_call, loop)

    # 4. 启动 ROS 线程
    ros_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    ros_thread.start()

    # 5. 启动流水线
    pipeline.set_state(Gst.State.PLAYING)
    print("[INFO] 开始录制，按下 Ctrl+C 停止...")

    try:
        loop.run()
    finally:
        # 6. 优雅清理逻辑
        print("[INFO] 正在释放资源...")
        
        # 发送 EOS 确保分段视频正常结尾 (选做，对 MP4 录制更友好)
        pipeline.send_event(Gst.Event.new_eos())
        time.sleep(1.0) # 给 1 秒时间处理 EOS
        
        pipeline.set_state(Gst.State.NULL)
        node.destroy_node()
        rclpy.shutdown()
        print("[INFO] 已退出")

    # try:
    #     loop.run() # 等待 bus_call 收到 EOS 后自动退出 loop
    # finally:
    #     # 6. 优雅清理逻辑
    #     print("[INFO] 流水线已停止，正在释放 ROS 资源...")
    #     pipeline.set_state(Gst.State.NULL)
    #     node.destroy_node()
    #     if rclpy.ok():
    #         rclpy.shutdown()
    #     print("[INFO] 已完全退出")

if __name__ == "__main__":
    main()

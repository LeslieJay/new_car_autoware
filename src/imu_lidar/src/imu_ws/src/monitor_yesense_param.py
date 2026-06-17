#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.client import AsyncParametersClient

class YesenseParamMonitor(Node):
    def __init__(self):
        super().__init__('yesense_param_monitor')
        self.client = AsyncParametersClient(self, 'yesense_pub')
        self.last_value = None
        self.get_logger().info("Monitoring /yesense_pub.use_sim_time ...")

        # 启动定时器，高频检查（1ms 间隔）
        self.timer = self.create_timer(0.001, self.check_param)  # 1ms

    def check_param(self):
        future = self.client.get_parameters(['use_sim_time'])
        rclpy.spin_until_future_complete(self, future, timeout_sec=0.001)
        if future.done():
            try:
                param = future.result()
                if param and isinstance(param[0], Parameter):
                    value = param[0].value
                    if value != self.last_value:
                        self.get_logger().warn(f"use_sim_time changed to: {value}")
                        self.last_value = value
                else:
                    self.get_logger().error("Failed to get parameter (node may have crashed)")
                    self.get_logger().error("YESSENSE NODE IS GONE!")
                    rclpy.shutdown()
            except Exception as e:
                self.get_logger().error(f"Exception: {e}")
                self.get_logger().error("NODE CRASHED or unreachable!")
                rclpy.shutdown()
        # else: 超时未完成，说明节点无响应 → 可能已崩溃
        elif not future.done():
            self.get_logger().error("NO RESPONSE from yesense_pub — NODE CRASHED!")
            rclpy.shutdown()

def main():
    rclpy.init(args=None)
    node = YesenseParamMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
#!/usr/bin/env python3
import rclpy
from asl_tb3_lib.control import BaseController 
from asl_tb3_msgs.msg import TurtleBotControl

class PerceptionController(BaseController):
    def __init__(self):
        super().__init__('perception_controller')
        self.declare_parameter('active', True)
        self._stop_until=None

    @property
    def active(self)->bool:
        return self.get_parameter('active').value


    def compute_control(self)->TurtleBotControl:
        msg=TurtleBotControl()
        now=self.get_clock().now().nanoseconds/1e9
        if self.active:
            self._stop_until=None
            msg.v=0.0
            msg.omega=0.5
            return msg
        if self._stop_until is None:
            self._stop_until=now+5.0

        if now<self._stop_until:
            msg.v=0.0
            msg.omega=0.0
            return msg
        else:
            self.set_parameters([rclpy.Parameter('active',rclpy.Parameter.Type.BOOL,True)])
            self._stop_until=None
            msg.v=0.0
            msg.omega=0.5
            return msg
        
if __name__ == "__main__":
    rclpy.init()
    controller = PerceptionController()
    rclpy.spin(controller)
    rclpy.shutdown()

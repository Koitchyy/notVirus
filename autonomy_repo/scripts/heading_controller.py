#!/usr/bin/env python3

import rclpy
from asl_tb3_lib.control import BaseHeadingController
from asl_tb3_lib.math_utils import wrap_angle
from asl_tb3_msgs.msg import TurtleBotControl, TurtleBotState

class HeadingController(BaseHeadingController):
    def __init__(self):
        super().__init__()
        self.declare_parameter("kp", 2.0)

    @property
    def kp(self) -> float:
        """Get real-time parameter value of max velocity
        
        Returns:
            float: latest parameter value of max velocity
        """
        return self.get_parameter("kp").value


    def compute_control_with_goal(self,
        state: TurtleBotState,
        goal: TurtleBotState
    ) -> TurtleBotControl:
        error = wrap_angle(goal.theta - state.theta)
        om = self.kp * error

        return TurtleBotControl(omega=om)

if __name__ == "__main__":
    rclpy.init()
    controller = HeadingController()
    rclpy.spin(controller)
    rclpy.shutdown()


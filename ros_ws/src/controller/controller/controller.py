# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from geometry_msgs.msg import Twist, Vector3, Pose

import numpy as np

class Controller(Node):

    def __init__(self):
        super().__init__('controller')

        self.publisher_ = self.create_publisher(Twist, '/setpoint_euler', 10)

        self.subscription = self.create_subscription(Pose, '/setpoint_pose', self.listener_callback, 10)
        self.subscription  # prevent unused variable warning

    def publisher_callback(self, homo_trans):

        alpha = np.arctan2(homo_trans[1,0], homo_trans[0,0])
        beta = np.arctan2(-homo_trans[2,0], np.sqrt( homo_trans[2,1]**2 + homo_trans[2,2]**2 ))
        gamma = np.arctan2(homo_trans[2,1], homo_trans[2,2])

        msg = Twist(
            linear = Vector3(x = homo_trans[0,3], y = homo_trans[1,3], z = homo_trans[2,3]),
            angular = Vector3(x = gamma, y = beta, z = alpha)
            )

        self.publisher_.publish(msg)


    def listener_callback(self, msg):

        homo_trans = np.zeros((4,4))

        ni = msg.orientation.w
        e_x = msg.orientation.x
        e_y = msg.orientation.y
        e_z = msg.orientation.z

        homo_trans[0,0] = 2 * (ni*ni + e_x*e_x) - 1
        homo_trans[0,1] = 2 * (e_x*e_y - ni*e_z)
        homo_trans[0,2] = 2 * (e_x*e_z + ni*e_y)

        homo_trans[1,0] = 2 * (e_x*e_y + ni*e_z)
        homo_trans[1,1] = 2 * (ni*ni + e_y*e_y) - 1
        homo_trans[1,2] = 2 * (e_y*e_z - ni*e_x)

        homo_trans[2,0] = 2 * (e_x*e_z - ni*e_y)
        homo_trans[2,1] = 2 * (e_y*e_z + ni*e_x)
        homo_trans[2,2] = 2 * (ni*ni + e_z*e_z) - 1

        homo_trans[0,3] = msg.position.x
        homo_trans[1,3] = msg.position.y
        homo_trans[2,3] = msg.position.z
        homo_trans[3,3] = 1

        self.get_logger().info('Homogeneous Transformation Matrix')
        self.get_logger().info('"%.2f" "%.2f" "%.2f" "%.2f"' %(homo_trans[0,0], homo_trans[0,1], homo_trans[0,2], homo_trans[0,3]))
        self.get_logger().info('"%.2f" "%.2f" "%.2f" "%.2f"' %(homo_trans[1,0], homo_trans[1,1], homo_trans[1,2], homo_trans[1,3]))
        self.get_logger().info('"%.2f" "%.2f" "%.2f" "%.2f"' %(homo_trans[2,0], homo_trans[2,1], homo_trans[2,2], homo_trans[2,3]))
        self.get_logger().info('"%.2f" "%.2f" "%.2f" "%.2f"' %(homo_trans[3,0], homo_trans[3,1], homo_trans[3,2], homo_trans[3,3]))

        self.publisher_callback(homo_trans)

def main(args=None):
    rclpy.init(args=args)

    controller = Controller()

    rclpy.spin(controller)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

"""
Demonstration of using the nav2 action interface. This node navigates to a goal pose
provided on the command line.  This code also include a demonstration of interacting
with OccupancyGrid messages through the map_util.Map class.

Author: Nathan Sprague
Version: 10/29/2020
"""
import argparse
import time
import numpy as np

import rclpy
import rclpy.node
from rclpy.action.client import ActionClient
from rclpy.task import Future

from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus

from rclpy.qos import QoSProfile, QoSDurabilityPolicy

from nav_msgs.msg import OccupancyGrid
from jmu_ros2_util import map_utils, transformations


class NavNode(rclpy.node.Node):

    def __init__(self, x, y, theta, timeout):
        super().__init__('nav_demo')

        #self.create_timer(.1, self.timer_callback)

        latching_qos = QoSProfile(depth=1,
                durability=QoSDurabilityPolicy.RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL)
        self.create_subscription(OccupancyGrid, 'map', self.map_callback, 
                qos_profile=latching_qos)

        # Create the action client and wait until it is active
        #self.ac = ActionClient(self, NavigateToPose, '/NavigateToPose')
        self.ac = ActionClient(self, NavigateToPose, '/navigate_to_pose')


        # Set up the goal message
        self.goal = NavigateToPose.Goal()
        self.goal.pose.header.frame_id = 'map'  # SEEMS TO BE IGNORED!
        self.goal.pose.pose.position.x = x
        self.goal.pose.pose.position.y = y
        # We need to convert theta to a quaternion....
        quaternion = transformations.quaternion_from_euler(0, 0, theta, 'rxyz')
        self.goal.pose.pose.orientation.x = quaternion[0]
        self.goal.pose.pose.orientation.y = quaternion[1]
        self.goal.pose.pose.orientation.z = quaternion[2]
        self.goal.pose.pose.orientation.w = quaternion[3]

        self.timeout = timeout

    def send_goal(self):
        self.get_logger().info("WAITING FOR NAVIGATION SERVER...")
        self.ac.wait_for_server()
        self.get_logger().info("NAVIGATION SERVER AVAILABLE...")
        self.get_logger().info("SENDING GOAL TO NAVIGATION SERVER...")
        self.start_time = time.time()

        self.cancel_future = None
        self.is_done = False
        self.map = None
        self.goal_future = self.ac.send_goal_async(self.goal)

        self.create_timer(.1, self.timer_callback)
        self.future_event = Future()

        return self.future_event

    def map_callback(self, map_msg):
        """Process the map message.  This doesn't really do anything useful, it is
        purely intended as an illustration of the Map class.

        """
        if self.map is None:  # No need to do this every time map is published.

            self.map = map_utils.Map(map_msg)

            # Use numpy to calculate some statistics about the map:
            total_cells = self.map.width * self.map.height
            pct_occupied = np.count_nonzero(self.map.grid == 100) / total_cells * 100
            pct_unknown = np.count_nonzero(self.map.grid == -1) / total_cells * 100
            pct_free = np.count_nonzero(self.map.grid == 0) / total_cells * 100
            map_str = "Map Statistics: occupied: {:.1f}% free: {:.1f}% unknown: {:.1f}%"
            self.get_logger().info(map_str.format(pct_occupied, pct_free, pct_unknown))

            # Here is how to access map cells to see if they are free:
            # x = 2.06
            # y = -1.2
            x = self.goal.pose.pose.position.x
            y = self.goal.pose.pose.position.y
            val = self.map.get_cell(x, y)
            if val == 100:
                free = "occupied"
            elif val == 0:
                free = "free"
            else:
                free = "unknown"
            self.get_logger().info("HEY! Map position ({:.2f}, {:.2f}) is {}".format(x, y, free))

    def done(self):
        """ This allows the node to act as a future object. """
        return self.is_done

    def timer_callback(self):
        """ Periodically check in on the progress of navigation. """

        if not self.goal_future.done():
            self.get_logger().info("NAVIGATION GOAL NOT YET ACCEPTED")

        elif self.cancel_future is not None:  # We've cancelled and are waiting for ack.
            if self.cancel_future.done():
                self.ac.destroy()
                self.is_done = True
                self.future_event.set_result(None)

        else:

            if self.goal_future.result().status == GoalStatus.STATUS_SUCCEEDED:
                self.get_logger().info("NAVIGATION SERVER REPORTS SUCCESS. EXITING!")
                self.ac.destroy()
                self.is_done = True
                self.future_event.set_result(None)

            if self.goal_future.result().status == GoalStatus.STATUS_ABORTED:
                self.get_logger().info("NAVIGATION SERVER HAS ABORTED. EXITING!")
                self.ac.destroy()
                self.is_done = True
                self.future_event.set_result(None)

            elif time.time() - self.start_time > self.timeout:
                self.get_logger().info("TAKING TOO LONG. CANCELLING GOAL!")
                self.cancel_future = self.goal_future.result().cancel_goal_async()
                self.future_event.set_result(None)

def main():
    parser = argparse.ArgumentParser(formatter_class=argparse.ArgumentDefaultsHelpFormatter,
                                     description="Navigate to a designated location.")
    parser.add_argument('x', default=0.0, type=float,
                        help='X-coordinate of goal')
    parser.add_argument('y', default=0.0, type=float,
                        help='Y-coordinate of goal')
    parser.add_argument('theta', default=0.0, type=float,
                        help='Orientation of goal')
    parser.add_argument('--timeout', default=float('inf'), type=float,
                        help='How long to wait before cancelling navigation.')
    args = parser.parse_args()

    rclpy.init()

    node = NavNode(args.x, args.y, args.theta, args.timeout)

    future = node.send_goal()

    rclpy.spin_until_future_complete(node,future)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from geometry_msgs.msg import PoseStamped
import rclpy

def make_pose(navigator, x, y):
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.header.stamp = navigator.get_clock().now().to_msg()
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.orientation.w = 1.0
    return pose

def main():
    rclpy.init()
    navigator = BasicNavigator()
    navigator.waitUntilNav2Active(localizer='slam_toolbox')

    waypoints = [
        make_pose(navigator, 1.7585004921374887, -0.3866338038824857),
        make_pose(navigator, 2.3947220232624513, 0.26344221734774775)
    ]

    navigator.followWaypoints(waypoints)

    while not navigator.isTaskComplete():
        feedback = navigator.getFeedback()
        if feedback:
            print(f'Executing waypoint {feedback.current_waypoint + 1} of {len(waypoints)}')

    print(navigator.getResult())
    rclpy.shutdown()

if __name__ == '__main__':
    main()

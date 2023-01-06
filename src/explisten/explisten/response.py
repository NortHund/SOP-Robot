import rclpy
import os
import re
from rclpy.node import Node

from std_msgs.msg import String
from vision2_msgs.msg import Faces, Face, Point2, FaceImage, FaceImages


class Vision2ActionMsgs(Node):

    def __init__(self):
        print("init")
        super().__init__('vision2_action_msgs')
        self.subscription = self.create_subscription(
            Faces,
            '/face_details',
            self.handle_action_msgs,
            10)
        self.subscription  # prevent unused variable warning

    def handle_action_msgs(self, faces_details):
        print("handle")
        print(faces_details)
        print(type(faces_details))
        detailstr = str(faces_details)
        #wlist = detailstr.split()
        wlist = re.findall(r'\w+', detailstr)
        print(wlist)
        
        for i in wlist:
            if i == "Happy":
                print("yes")
                os.system("ros2 action send_goal /jaw_controller/follow_joint_trajectory control_msgs/action/FollowJointTrajectory \"{ trajectory: { joint_names: [head_jaw_joint], points: [ { positions: [0.1], time_from_start: { sec: 1, nanosec: 0 } } ] } }\" ");
                os.system("canberra-gtk-play -f /workspace/sounds/hello.ogg");
                os.system("ros2 action send_goal /jaw_controller/follow_joint_trajectory control_msgs/action/FollowJointTrajectory \"{ trajectory: { joint_names: [head_jaw_joint], points: [ { positions: [0.0], time_from_start: { sec: 1, nanosec: 0 } } ] } }\" ");

            if i == "Angry":
                print("no")
                os.system("ros2 action send_goal /head_controller/follow_joint_trajectory control_msgs/action/FollowJointTrajectory \"{ trajectory: { joint_names: [head_pan_joint, head_tilt_right_joint, head_tilt_left_joint, head_tilt_vertical_joint], points: [ { positions: [0.7, 0.0, 0.0, 0.0], time_from_start: { sec: 1, nanosec: 0 } } ] } }\" ");
                os.system("canberra-gtk-play -f /workspace/sounds/helpme.ogg");
                os.system("ros2 action send_goal /head_controller/follow_joint_trajectory control_msgs/action/FollowJointTrajectory \"{ trajectory: { joint_names: [head_pan_joint, head_tilt_right_joint, head_tilt_left_joint, head_tilt_vertical_joint], points: [ { positions: [0.0, 0.0, 0.0, 0.0], time_from_start: { sec: 1, nanosec: 0 } } ] } }\" ");
    

        #self.send_goal(1)


def main(args=None):
    print("starting")
    rclpy.init(args=args)

    exp_listener = Vision2ActionMsgs()

    rclpy.spin(exp_listener)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    exp_listener.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

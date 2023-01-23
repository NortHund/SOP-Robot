import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from std_msgs.msg import String
from vision2_msgs.msg import Faces, Face, Point2, FaceImage, FaceImages

#import actionlib

from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from trajectory_msgs.msg import JointTrajectory
from builtin_interfaces.msg import Duration

import time
import re



class Vision2ActionMsgs(Node):
    
    
    def __init__(self):
        super().__init__('vision2_action_msgs')

        self._action_client = ActionClient(
            self, FollowJointTrajectory, '/head_controller/follow_joint_trajectory')

        self.subscriber = self.create_subscription(
            Faces,
            "/face_details",
            self.handle_action_msgs,
            10,
        )
        global globalstatus
        globalstatus = 0
        global time1
        time1 = 0
        global time2
        time2 = 0

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def feedback_callback(self, feedback):
        self.get_logger().info('Received feedback: {0}'.format(feedback.feedback))

    def get_result_callback(self, future):
        result = future.result().result
        status = future.result().status
        global globalstatus 
        globalstatus = future.result().status
        self.get_logger().info('Result: {0}'.format(result))
        self.get_logger().info('status: {0}'.format(status))
       

    




    def handle_action_msgs(self, faces_details):
        # todo... 
        # parse message (face_details)...
        #time.sleep(5)
        print(faces_details)
        print(type(faces_details))
        detailstr = str(faces_details)
        #wlist = detailstr.split()
        wlist = re.findall(r'\w+', detailstr)
        print(wlist)
        
        goal = [0.0, 0.0, 0.0, 0.0]
        
        xvalue = 0
        xarray = []
        facesizes = []
        faceexp = 0
        emotionarray = []
        for i, value in enumerate(wlist):
            if value == "x":
                xarray.append(int(wlist[i+1]))
            if value == "emotion":
                emotionarray.append(wlist[i+1])

        print("xarray")
        print(xarray)
        
        print("emotionarray")
        print(emotionarray)

        for i in range(int(len(xarray) / 2)):
            facesizes.append(xarray[i*2 + 1] - xarray[i*2])
            
        print("facesizes")
        print(facesizes)

        maxface = max(facesizes)

        print("maxface")
        print(maxface)

        maxfaceindex = facesizes.index(maxface)
        print("maxfaceindex")
        print(maxfaceindex)
        
        maxfaceemotion = emotionarray[maxfaceindex]
        
        print("maxfaceemotion")
        print(maxfaceemotion)

        xvalue = (xarray[maxfaceindex * 2] + xarray[maxfaceindex * 2 + 1]) / 2
        
        print("x-centre-value")
        print(xvalue)
        
        if maxfaceemotion == "Happy":
            faceexp = 1
            #wait(2)
        elif maxfaceemotion == "Angry":
            faceexp = 2
            #wait(2)
        
        
        if faceexp == 1:
            print("Happy face detected")
            xvalscaled = float(xvalue) / 1100
            xvalscaled = (xvalscaled - 0.6)*(-1)
            goal = [xvalscaled, 0.0, 0.0, 0.0]
        elif faceexp == 2:
            print("Angry face detected")
            if xvalue > 600:
                goal = [0.6, 0.0, 0.0, 0.0]
            else:
                goal = [-0.6, 0.0, 0.0, 0.0]
        
        # Calculate closest person
        # calculate projected goal position based on expression.. 
        # happy look towards? neutral reset position? angry -> lookaway?
        # goal is https://docs.ros2.org/foxy/api/trajectory_msgs/msg/JointTrajectoryPoint.html "positions"
        # print(faces_details)
        time1 = time.perf_counter()
        
        self.send_goal(goal)
        #self.wait_for_result(15)
        #time.sleep(15)
        
        

    def send_goal(self, goal):

        global time1 
        global time2
        time1 = time.perf_counter()
        
        if time1 - time2 > 3:
            head_joints = ['head_pan_joint', 
                            'head_tilt_right_joint', 
                            'head_tilt_left_joint', 
                            'head_tilt_vertical_joint']

            # "correct" duration might be in lecture videos
            duration = Duration(sec=1,nanosec=0)
            msg1 = [JointTrajectoryPoint(positions=goal,time_from_start=duration)]
            msg2 = JointTrajectory(joint_names=head_joints,points=msg1)
            #self.get_logger().info('Waiting for action server...')
            self._action_client.wait_for_server()
            #self.get_logger().info('Sending goal request...')
            goal_msg = FollowJointTrajectory.Goal()
            goal_msg.trajectory = msg2
            
            
            self._send_goal_future = self._action_client.send_goal_async(
                goal_msg,
                feedback_callback=self.feedback_callback)
            #time.sleep(10)
            self._send_goal_future.add_done_callback(self.goal_response_callback)
            serverstatus = globalstatus
            
            time2 = time.perf_counter()
            print(serverstatus)
        if self._action_client.server_is_ready():
            #self._action_client.send_goal_async(goal_msg)
            print("server availabel goal")
        
        return 1
        #return self._action_client.send_goal(goal_msg)
        
        #return self._action_client.get_result()

        #return self._action_client.send_goal_async(goal_msg)

def main(args=None):
    rclpy.init(args=args)

    vision2_action_msgs = Vision2ActionMsgs()

    rclpy.spin(vision2_action_msgs)
    

    vision2_action_msgs.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

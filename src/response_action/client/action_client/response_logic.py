# Copyright 2018 Open Source Robotics Foundation, Inc.
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

from action_msgs.msg import GoalStatus

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from std_msgs.msg import String
from vision2_msgs.msg import Faces, Face, Point2, FaceImage, FaceImages
from response_msgs.msg import response_goal


class MinimalActionClient(Node):

    def __init__(self):
        self.subscriber = self.create_subscription(
            Faces,
            "/face_details",
            self.handle_action_msgs,
            10,
        )


    def handle_action_msgs(self, faces_details):
        print(faces_details)
        print(type(faces_details))
        detailstr = str(faces_details)
        wlist = re.findall(r'\w+', detailstr)
        print(wlist)
        
        goal = [0.0, 0.0, 0.0, 0.0]
        
        global currentxcor
        
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
                
        self.face_details_publisher.publish(Goals(head=goal))

def main(args=None):
    rclpy.init(args=args)

    action_client = MinimalActionClient()

    rclpy.spin(action_client)


if __name__ == '__main__':
    main()

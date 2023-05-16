from re import I
import rclpy
from rclpy.node import Node
import cv2

from .yunet import YuNet
import os
import numpy as np

from ament_index_python.packages import get_package_share_directory

from std_msgs.msg import String
from sensor_msgs.msg import Image

from vision2_msgs.msg import Faces, Face, Point2, FaceImage, FaceImages

from cv_bridge import CvBridge, CvBridgeError

bridge = CvBridge()
'''For haarcascade
face_cascade_name = cv2.CascadeClassifier(
    cv2.data.haarcascades + 'haarcascade_frontalface_default.xml')
'''


class Vision2(Node):

    def __init__(self):
        super().__init__('vision')

        model = (
            self.declare_parameter(
                "classifier", "face_detection_yunet_2022mar.onnx")
            .get_parameter_value()
            .string_value
        )

        self.model = YuNet(modelPath=os.path.join(
            get_package_share_directory("vision2"),
            "classifier",
            model,
        ),
            inputSize=[320, 320],
            confThreshold=0.9,
            nmsThreshold=0.3,
            topK=5000,
            backendId=0,
            targetId=0)

        image_topic = (
            self.declare_parameter("image_topic", "/image_raw")
            .get_parameter_value()
            .string_value
        )

        self.subscriber = self.create_subscription(
            Image,
            image_topic,
            self.detect_face,
            3,
        )

        face_image_topic = (
            self.declare_parameter(
                "face_image_topic", "image_face"
            )
            .get_parameter_value()
            .string_value
        )

        faces_image_topic = (
            self.declare_parameter(
                "faces_image_topic", "faces_image_array"
            )
            .get_parameter_value()
            .string_value
        )

        face_topic = (
            self.declare_parameter(
                "face_topic", "faces"
            )
            .get_parameter_value()
            .string_value
        )

        self.face_publisher = self.create_publisher(Faces, face_topic, 10)

        self.face_img_publisher = self.create_publisher(
            Image, face_image_topic, 2)

        self.faces_img_publisher = self.create_publisher(
            FaceImages, faces_image_topic, 10)

    def detect_face(self, img: Image):

        cv2_bgr_img = bridge.imgmsg_to_cv2(img, "bgr8")
        #cv2_gray_img = bridge.imgmsg_to_cv2(img, "mono8")

        '''For haarcascade
            # Get face coordinates
            face = face_cascade_name.detectMultiScale(
                cv2_gray_img, scaleFactor=1.2, minNeighbors=7, minSize=(35, 35))
            '''

        h, w, _ = cv2_bgr_img.shape
        self.model.setInputSize([w, h])
        face = self.model.infer(cv2_bgr_img)
        msg_faces = []
        msg_face_imgs = []

        '''For haarcascade
            #for x1, y1, w, h in face:
            '''
        id_tmp = 0
        try:
            for det2 in face:
                det = (np.rint(det2)).astype(int)
                x1 = det[0]
                y1 = det[1]
                w = det[2]
                h = det[3]

                new_face = cv2_bgr_img[y1:y1 + h, x1:x1 + w]

                cv2.rectangle(cv2_bgr_img, (x1, y1),
                              (x1+w, y1+h), (0, 120, 255), 2)

                new_face = cv2.resize(new_face, (48, 48))

                msg_face = Face(top_left=Point2(x=int(x1), y=int(y1)),
                                bottom_right=Point2(x=int(x1+w), y=int(y1+h)), id=id_tmp)

                msg_faces.append(msg_face)

                msg_face_img = FaceImage(face_image=(
                    bridge.cv2_to_imgmsg(new_face, "bgr8")))

                msg_face_imgs.append(msg_face_img)
                id_tmp = id_tmp+1
        except:
            pass

        self.face_publisher.publish(Faces(faces=msg_faces))

        self.face_img_publisher.publish(
            bridge.cv2_to_imgmsg(cv2_bgr_img, "bgr8"))

        self.faces_img_publisher.publish(FaceImages(
            face_images=msg_face_imgs, face_info=msg_faces))


def main(args=None):
    rclpy.init(args=args)

    visions = Vision2()

    rclpy.spin(visions)

    visions.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

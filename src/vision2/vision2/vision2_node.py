import rclpy
from rclpy.node import Node
import cv2

from ament_index_python.packages import get_package_share_directory

from std_msgs.msg import String
from sensor_msgs.msg import Image

from vision2_msgs.msg import Faces, Face, Point2, FaceImage, FaceImages

from cv_bridge import CvBridge, CvBridgeError

# for sensor_msg to image and back
bridge = CvBridge()
# using haarcascade
face_cascade_name = cv2.CascadeClassifier(
    cv2.data.haarcascades + 'haarcascade_frontalface_default.xml')


class Vision2(Node):

    def __init__(self):
        super().__init__('vision')

        # for raw img feed
        image_topic = (
            self.declare_parameter("image_topic", "/image_raw")
            .get_parameter_value()
            .string_value
        )

        # for raw img feed
        self.subscriber = self.create_subscription(
            Image,
            image_topic,
            self.detect_face,
            10,
        )

        # For visualization
        face_image_topic = (
            self.declare_parameter(
                "face_image_topic", "image_face"
            )
            .get_parameter_value()
            .string_value
        )

        # For faceimg array
        faces_image_topic = (
            self.declare_parameter(
                "faces_image_topic", "faces_image_array"
            )
            .get_parameter_value()
            .string_value
        )
        # For face coordinates for object tracker
        # todo -> create vision2_msgs and give 42x42 face pictures
        # with coordinates to expression_tracker_node
        face_topic = (
            self.declare_parameter(
                "face_topic", "faces"
            )
            .get_parameter_value()
            .string_value
        )

        # publish coordinates for object tracker
        self.face_publisher = self.create_publisher(Faces, face_topic, 10)

        # publish image for visualization
        self.face_img_publisher = self.create_publisher(
            Image, face_image_topic, 10)

        # publish face images for expression detection node
        self.faces_img_publisher = self.create_publisher(
            FaceImages, faces_image_topic, 10)

    def detect_face(self, img: Image):
        try:
            cv2_bgr_img = bridge.imgmsg_to_cv2(img, "bgr8")
            cv2_gray_img = bridge.imgmsg_to_cv2(img, "mono8")

            # Get face coordinates
            face = face_cascade_name.detectMultiScale(
                cv2_gray_img, scaleFactor=1.2, minNeighbors=7, minSize=(35, 35))

            # Array for coordinate messages
            msg_faces = []
            # Array for face images
            msg_face_imgs = []

            # Rotate every face from the picture
            # Todo: create also 48x48 picture from face and
            # Include it with message, or create another message for
            # expression tracker
            for x1, y1, w, h in face:

                # Lines for getting face

                new_face = cv2_bgr_img[y1:y1 + h, x1:x1 + w]

                # For visualization draw a box around the face in original picture
                cv2.rectangle(cv2_bgr_img, (x1, y1),
                              (x1+w, y1+h), (0, 0, 255), 2)

                # resize picture for classifier
                new_face = cv2.resize(new_face, (48, 48))

                # add coordinates to message for object tracker
                msg_face = Face(top_left=Point2(x=int(x1), y=int(y1)),
                                bottom_right=Point2(x=int(x1+w), y=int(y1+h)))
                # add face coordinates to msg array
                msg_faces.append(msg_face)

                # add faceimg to msg
                msg_face_img = FaceImage(face_image=(
                    bridge.cv2_to_imgmsg(new_face, "bgr8")))

                # add img to array
                msg_face_imgs.append(msg_face_img)

            # publish coordinate messages
            self.face_publisher.publish(Faces(faces=msg_faces))

            # publish image for visualisation
            self.face_img_publisher.publish(
                bridge.cv2_to_imgmsg(cv2_bgr_img, "bgr8"))

            # self.face_img_publisher.publish(
            # bridge.cv2_to_imgmsg(new_face, "bgr8"))

            # img array publisher for expression detection
            self.faces_img_publisher.publish(FaceImages(
                face_images=msg_face_imgs, face_info=msg_faces))

        except:
            print("error")


def main(args=None):
    rclpy.init(args=args)

    visions = Vision2()

    rclpy.spin(visions)

    visions.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

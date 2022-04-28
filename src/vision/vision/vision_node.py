#from cgitb import grey
import rclpy
from rclpy.node import Node
import cv2
#import json
import numpy
import os

#from keras.preprocessing import img_to_array

from ament_index_python.packages import get_package_share_directory

from std_msgs.msg import String
from sensor_msgs.msg import Image
from face_tracker_msgs.msg import Faces, Face, Point2
from keras.models import load_model
from keras.preprocessing.image import img_to_array

from cv_bridge import CvBridge, CvBridgeError

j = 0
bridge = CvBridge()
face_cascade_name = cv2.CascadeClassifier(
    cv2.data.haarcascades + 'haarcascade_frontalface_default.xml')


class Vision(Node):

    def __init__(self):
        super().__init__('vision')

        emotionface_image_topic = (
            self.declare_parameter(
                "emotionface_image_topic", "image_emotionface"
            ) 
            .get_parameter_value()
            .string_value
        )

        classifier = (
            self.declare_parameter("classifier", "fer_2013.h5")
            .get_parameter_value()
            .string_value
        )

        self.classifier = load_model(
            os.path.join(
                get_package_share_directory("vision"),
                "classifier",
                classifier,
            )
        )

        self.subscriber = self.create_subscription(
            Image,
            "/image_raw",
            self.detect_face,
            10,
        )
        '''
        face_image_topic = (
            self.declare_parameter(
                "face_image_topic", "faces"
            )  # non-absolute paths are inside the current node namespace
            .get_parameter_value()
            .string_value
        )
        '''
        self.face_img_publisher = self.create_publisher(
            Image, emotionface_image_topic, 10)
        

        #TODOOO
        #self.face_cropped_publisher = self.create_publisher(Image,face_image_topic,10)

    def detect_face(self, img: Image):
        try:
            cv2_bgr_img = bridge.imgmsg_to_cv2(img, "bgr8")
            cv2_gray_img = bridge.imgmsg_to_cv2(img, "mono8")

            face = face_cascade_name.detectMultiScale(
                cv2_gray_img, scaleFactor=1.2, minNeighbors=7, minSize=(35, 35))

            faces_array = []
            i = 0

            for x, y, w, h in face:

                new_face = cv2_bgr_img[y:y + h, x:x + w] 
                new_face2 = cv2_gray_img[y:y + h, x:x + w]
                faces_array.append(new_face)

                cv2.rectangle(cv2_bgr_img, (x, y), (x+w, y+h), (0, 0, 255), 2)
                temp = cv2.resize(new_face2, (48, 48))

                pixels = img_to_array(temp)
                pixels = numpy.expand_dims(pixels, axis=0)

                predictions = self.classifier.predict(pixels)

                index = numpy.argmax(predictions[0])
                print(predictions)
                emotions = ("Angry", "Disgust", "Fear", "Happy",
                            "Neutral", "Sad", "Surprised")

                predicted_emotion = emotions[index]

                cv2.putText(faces_array[i], predicted_emotion, (int(20), int(
                    20)), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
                print(predicted_emotion)
                faces_array[i] = faces_array[i]
                i = i+1

            #blank_image = numpy.zeros((480,640,3), numpy.uint8)
            dim = (96, 128)
            i = 1
            x_offset = 0
            y_offset = 0
            for face in faces_array:
                fixed = cv2.resize(face, dim)

                cv2_bgr_img[y_offset:y_offset+fixed.shape[0],
                            x_offset:x_offset+fixed.shape[1]] = fixed
                # print(i)
                i = i+1
                x_offset = x_offset + 96
                if x_offset > 512:
                    y_offset = y_offset+128
                    x_offset = 0

            self.face_img_publisher.publish(
                bridge.cv2_to_imgmsg(cv2_bgr_img, "bgr8"))
            #self.face_img_publisher.publish(bridge.cv2_to_imgmsg(blank_image, "bgr8"))

        except:
            print("error")


def main(args=None):
    rclpy.init(args=args)

    visions = Vision()

    rclpy.spin(visions)

    visions.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

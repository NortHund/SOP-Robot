import rclpy
from rclpy.node import Node
import cv2
import numpy
import os


from ament_index_python.packages import get_package_share_directory

from std_msgs.msg import String
from sensor_msgs.msg import Image
import message_filters

from vision2_msgs.msg import Faces, Face, Point2, FaceImage, FaceImages
from keras.models import load_model
from keras.preprocessing.image import img_to_array

from cv_bridge import CvBridge, CvBridgeError

bridge = CvBridge()

# Need to fix everything here after vision2_msgs are working

class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = '%s' % self.i
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1


class ExpressionDetection(Node):

    def __init__(self):
        super().__init__('vision')

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

        # for recieving face img array + coord array
        self.faces_sub = message_filters.Subscriber(
            self,
            FaceImages,
            "/faces_image_array",  # when not using namespaces
        )
        # for recieving face coords with tracker ids
        self.image_sub = message_filters.Subscriber(
            self,
            Faces,
            "/face_ids",
        )

        self.sync = message_filters.ApproximateTimeSynchronizer(
            (self.faces_sub, self.image_sub), 4, 0.1, allow_headerless=True)

        self.sync.registerCallback(self.detect_expression)

    def detect_expression(self, msg_face_imgs,msg_ids):
        #try:
            # todo: handle msgs, detect expressions, send coords, expression, id array to next step (ai and vizualisation_node)

            # get face images
            msg_images = msg_face_imgs.face_images
            # get face info
            msg_face_info = msg_face_imgs.face_info
            # init array for face images
            faces_img_array = []

            # get face images to array
            for img in msg_images:
                #print(img)
                faces_img_array.append(bridge.imgmsg_to_cv2(img.face_image, "mono8"))
            
            # init array for face info
            face_info_array = []

            # get face info to array
            for info in msg_face_info:
                face_info_array.append(((info.top_left.x, info.top_left.y),
                         (info.bottom_right.x, info.bottom_right.y)))
            
            # 

            print(face_info_array)
            msg_faces_details = []

            for face in faces_img_array:

                temp = img_to_array(face)
                temp = numpy.expand_dims(temp, axis=0)

                predictions = self.classifier.predict(temp)

                index = numpy.argmax(predictions[0])
                emotions = ("Angry", "Disgust", "Fear", "Happy",
                            "Neutral", "Sad", "Surprised")

                predicted_emotion = emotions[index]
                print(predicted_emotion)
                #face_info_array[int(faces_img_array.index(face))]
                #msg_face_details = Face(top_left=Point2(x=int(x1), y=int(y1)),
                                #bottom_right=Point2(x=int(x1+w), y=int(y1+h)))

                #need to build messages + include ids




            #self.face_img_publisher.publish(bridge.cv2_to_imgmsg(blank_image, "bgr8"))

        #except:
           # print("error")


def main(args=None):
    rclpy.init(args=args)

    expression_detection = ExpressionDetection()

    rclpy.spin(expression_detection)

    expression_detection.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

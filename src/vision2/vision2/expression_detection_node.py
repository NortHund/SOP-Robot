from json.encoder import INFINITY
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

expression_list = []

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

        # for publisher which publish everything
        faces_details_topic = (
            self.declare_parameter(
                "face_details_topic", "face_details"
            )
            .get_parameter_value()
            .string_value
        )

        self.sync = message_filters.ApproximateTimeSynchronizer(
            (self.faces_sub, self.image_sub), 4, 0.1, allow_headerless=True)

        self.sync.registerCallback(self.detect_expression)

        # Publish expression, coordinates, tracker ids
        self.face_details_publisher = self.create_publisher(
            Faces, faces_details_topic, 10)

    def detect_expression(self, msg_face_imgs, msg_ids):
        # try:
        # todo: optimize for multiple faces
        # get face images
        msg_images = msg_face_imgs.face_images
        # get face info
        msg_face_info = msg_face_imgs.face_info
        # get face ids
        msg_face_ids = msg_ids.faces

        # init array for face images
        faces_img_array = []

        # get face images to array
        for img in msg_images:
            faces_img_array.append(
                bridge.imgmsg_to_cv2(img.face_image, "mono8"))

        # init array for face coord
        face_coord_array = []
        # get face coord to array
        for info in msg_face_info:
            face_coord_array.append(((info.top_left.x, info.top_left.y),
                                     (info.bottom_right.x, info.bottom_right.y)))
        # init array for face ids
        face_ids_array = []

        # get face ids to array
        for info in msg_face_ids:
            face_ids_array.append(((info.top_left.x, info.top_left.y),
                                   (info.bottom_right.x, info.bottom_right.y), info.id))

        face_coord_center = []

        for face_coords in face_coord_array:
            temp1, temp2 = face_coords
            face_coord_center.append(
                ((temp1[0]+0.5*temp2[0]), (temp1[1]+0.5*temp2[1])))

        face_ids_center = []

        for face_id in face_ids_array:
            temp1, temp2, temp3 = face_id
            face_ids_center.append(
                ((temp1[0]+0.5*temp2[0]), (temp1[1]+0.5*temp2[1]), temp3))

        #sorted_ids_coords = face_ids_array
        sorted_ids_coords = []

        for face_id_position in face_ids_center:
            dist = INFINITY
            new_index = 0
            for face_coord_position in face_coord_center:
                if numpy.sqrt((face_coord_position[0]-face_id_position[0])**2+(face_coord_position[1]-face_id_position[1])**2) < dist:
                    new_index = face_coord_center.index(
                        face_coord_position)
                    dist = numpy.sqrt((face_coord_position[0]-face_id_position[0])**2+(face_coord_position[1]-face_id_position[1])**2)

            try:
                # sorted_ids_coords[new_index] = face_ids_array[face_ids_center.index(
                # face_id_position)]
                sorted_ids_coords.append(face_ids_array[new_index])
                # except IndexError:
            except:
                pass

        msg_faces_details = []

        #print(sorted_ids_coords)


        #print("faces_img_array len: ", len(faces_img_array))
        #print("sorted_ids_coords len: ", len(sorted_ids_coords))

        i = 0
        if (len(faces_img_array) <= len(sorted_ids_coords)):
            detected_round = False
            for face in faces_img_array:
                is_on_list = False
                #check if expression list is null
                #***********************************************
                if len(expression_list) > 0:
                    for item in expression_list:
                        if item[0] == sorted_ids_coords[i][2]:
                            is_on_list = True
                    if not is_on_list:
                        for item in expression_list:
                            #print("muut falseksi")
                            item[1] = False
                        expression_list.append([sorted_ids_coords[i][2], True, ""])
                else:
                    expression_list.append([sorted_ids_coords[i][2], True, ""])
                    #print("lisatty ensimmainen listalle")

                #print(expression_list)

                #*********************************************

                pop_these = len(expression_list) - len(faces_img_array)

                while pop_these > 0:
                    try:
                        expression_list.pop()
                        pop_these = pop_these -1
                        #print("popped")
                    except:
                        pass

                #**********************************************

                try:
                    temp_msg_det = sorted_ids_coords[i]
                    i = i+1
                except IndexError:
                    # except:
                    pass
                j = 0
                needs_detection = False
                while j < len(expression_list):
                    #print(j)
                    #print(i)
                    #print("kek")
                    if sorted_ids_coords[i-1][2] == expression_list[j][0]:
                        needs_detection = expression_list[j][1]
                        break
                    j = j+1

                if needs_detection and not detected_round:
                    temp = img_to_array(face)
                    temp = numpy.expand_dims(temp, axis=0)
                    predictions = self.classifier.predict(temp)
                    index = numpy.argmax(predictions[0])
                    emotions = ("Angry", "Disgust", "Fear", "Happy",
                            "Neutral", "Sad", "Surprised")
                    predicted_emotion = emotions[index]

                    expression_list[j][2] = predicted_emotion
                    print("detected emotion for: ", j)
                    detected_round = True

                    if(len(expression_list)==1):
                        pass
                    else:
                        expression_list[j][1] = False
                        if j == 0:
                            expression_list[len(expression_list)-1][1] = True
                        else:
                            expression_list[j-1][1] = True

                else:
                    predicted_emotion = expression_list[j][2]



                

                #try:
                #print("hep)")
                face_id_msg = Face(top_left=Point2(x=int(temp_msg_det[0][0]), y=int(temp_msg_det[0][1])), bottom_right=Point2(
                    x=int(temp_msg_det[1][0]), y=int(temp_msg_det[1][1])), id=int(temp_msg_det[2]), emotion=predicted_emotion)

                msg_faces_details.append(face_id_msg)
                #except UnboundLocalError:
                    #print("error")
                    #pass
                #i = i + 1
            self.face_details_publisher.publish(Faces(faces=msg_faces_details))
            #print("Send msg to face_details topic")
            print(msg_faces_details)

        # except:
        # print("error")


def main(args=None):
    rclpy.init(args=args)

    expression_detection = ExpressionDetection()

    rclpy.spin(expression_detection)

    expression_detection.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

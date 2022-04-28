import rclpy
from rclpy.node import Node

import dlib
import cv2

from std_msgs.msg import String
from sensor_msgs.msg import Image

from vision2_msgs.msg import Faces, Face, Point2

from cv_bridge import CvBridge, CvBridgeError
import message_filters

# for sensor_msg to image and back
bridge = CvBridge()
# init tracker list
# there should be some better way with services
trackers = []
# need to fix ids when there is vision2_msgs
ids = []


class ObjectTracker(Node):

    def __init__(self):
        super().__init__("object_tracker")

        # for visualization (testing)
        object_tracker_image_topic = (
            self.declare_parameter(
                "object_tracker_image_topic", "image_tracked"
            )
            .get_parameter_value()
            .string_value
        )
        # for tracking face positions
        self.faces_sub = message_filters.Subscriber(
            self,
            Faces,
            # "face_tracker/faces",
            # "vision2/faces", #for vision2 tracking when its done
            "/faces",  # when not using namespaces
        )
        # for raw img feed (could use also face feature coordinates
        # if we take that approach)
        self.image_sub = message_filters.Subscriber(
            self,
            Image,
            "/image_raw",
        )
        # for face tracker id + coordinates publisher
        faces_id_topic = (
            self.declare_parameter(
                "face_topic", "face_ids"
            )
            .get_parameter_value()
            .string_value
        )

        # need image + coordinates msgs for tracking
        # need to add msg.header.stamp = node.get_clock().now() to face topic to avoid
        # headerless messages.
        self.sync = message_filters.ApproximateTimeSynchronizer(
            (self.faces_sub, self.image_sub), 4, 0.1, allow_headerless=True)
        #
        self.sync.registerCallback(self.track_object)
        # visualize tracking
        self.face_img_publisher = self.create_publisher(
            Image, object_tracker_image_topic, 10)
        # coordinates + tracker ids
        self.face_id_publisher = self.create_publisher(
            Faces, faces_id_topic, 10)

    # Handles tracking

    def track_object(self, faces_msg, img_msg):
        # convert msg to image
        cv2_bgr_img = bridge.imgmsg_to_cv2(img_msg, "bgr8")
        # get msgs
        msg1 = faces_msg.faces
        # init array for msgs in msg
        msg2 = []
        for msg in msg1:
            msg2.append(((msg.top_left.x, msg.top_left.y),
                         (msg.bottom_right.x, msg.bottom_right.y)))
        # init final array for msgs in msg
        faces = []
        # get face coordinates from msgs
        for msg in msg2:
            faces.append((msg))
        # init array for index numbers
        trackers_delete = []
        faces_ids_msg = []
        i = 0
        try:
            # loop every tracker
            for tracker in trackers:
                # update every tracker and get tracking quality
                quality = tracker.update(cv2_bgr_img)
                # if quality under value, we stop tracking
                if quality < 8:
                    # append index number of tracker we dont want anymore
                    trackers_delete.append(trackers.index(tracker))

                else:
                    # if we continue tracking we get position of each tracker
                    t_pos = tracker.get_position()

                    t_x = int(t_pos.left())
                    t_y = int(t_pos.top())
                    t_w = int(t_pos.width())
                    t_h = int(t_pos.height())
                    # we draw rextangles of tracker to converted image
                    cv2.rectangle(cv2_bgr_img, (t_x,  t_y),
                                  (t_x+t_w, t_y+t_h), (0, 0, 255), 2)
                    # also include id of tracker as identification
                    cv2.putText(cv2_bgr_img, ("ID: " + str(ids[trackers.index(tracker)])),
                                (t_x, t_y), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)

                    # create coord msg with tracker id
                    face_id_msg = Face(top_left=Point2(x=int(t_x), y=int(t_y)), bottom_right=Point2(
                        x=int(t_x+t_w), y=int(t_y+t_h)), id=int((ids[trackers.index(tracker)])))
                    # append msg list
                    faces_ids_msg.append(face_id_msg)

                # last we pop unwanted trackers
            for index_to_remove in trackers_delete:
                trackers.pop(index_to_remove-i)
                ids.pop(index_to_remove-i)
                i = i+1
        except:
            print("error")
        # first we check if we have any new faces to track
        if len(faces) > len(trackers):
            # loop every face coordinate
            for face in faces:

                (topleft_xy, bottomright_xy) = face
                topleft_x, topleft_y = topleft_xy
                bottomright_x, bottomright_y = bottomright_xy

                w = (bottomright_x-topleft_x)
                h = (bottomright_y-topleft_y)
                # get middle of every coordinate pair
                x_mid = topleft_x + 0.5 * w
                y_mid = topleft_y + 0.5 * h

                tracked = 0
                # loop every tracker
                for tracker in trackers:
                    t_pos = tracker.get_position()

                    t_x = int(t_pos.left())
                    t_y = int(t_pos.top())
                    t_w = int(t_pos.width())
                    t_h = int(t_pos.height())
                    # get middle point of every tracker
                    t_x_mid = t_x + 0.5 * t_w
                    t_y_mid = t_y + 0.5 * t_h
                    # if middlepoint is lockated in tracking zone
                    # or any middlepoint is lockated in face zone
                    # we determine that we are already tracking this face position
                    if ((t_x <= x_mid <= (t_x + t_w)) and
                        (t_y <= y_mid <= (t_y + t_h)) and
                        (topleft_x <= t_x_mid <= (bottomright_x)) and
                            (topleft_y <= t_y_mid <= (bottomright_y))):
                        tracked = +1
                # if we are not tracking
                if tracked == 0:
                    # we initialize new tracker
                    tracker = dlib.correlation_tracker()
                    # we start tracking new face
                    tracker.start_track(cv2_bgr_img, dlib.rectangle(
                        topleft_x-10, topleft_y-30, bottomright_x+5, bottomright_y+5))
                    # we append our trakcer list

                    trackers.append(tracker)
                    if(len(ids) == 0):
                        ids.append(1)
                    else:
                        ids.append(ids[len(ids)-1]+1)
        # print(cv2_bgr_img.shape)

        # we publish img for vizualisation
        self.face_img_publisher.publish(
            bridge.cv2_to_imgmsg(cv2_bgr_img, "bgr8"))

        self.face_id_publisher.publish(Faces(faces=faces_ids_msg))

        # todo: we need to info as msg and also handle vizualisation elsewhere
        # where we also include data from other nodes.
        # also need to scale down pictures for performance and think about way
        # how to handle tracking for example every 5th or 10th frame


def main(args=None):

    rclpy.init(args=args)
    tracker = ObjectTracker()

    rclpy.spin(tracker)

    tracker.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

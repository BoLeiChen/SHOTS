#!/home/ros/anaconda2/envs/mediapipe/bin python3
import cv2
import mediapipe as mp
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
# import multiprocessing
import threading
import time
import queue

mp_drawing = mp.solutions.drawing_utils
mp_objectron = mp.solutions.objectron

img_buffer = queue.Queue()

class object_detection:
    def __init__(self) -> None:
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/camera/color/image_raw", Image, self.img_callback, queue_size=1)


    def img_callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)
        
        img_buffer.put(cv_image)
        
def img_buffer_handle():
    with mp_objectron.Objectron(static_image_mode=False,
                            max_num_objects=5,
                            min_detection_confidence=0.3,
                            min_tracking_confidence=0.88,
                            model_name='Cup') as objectron:
        while True:
            # time.sleep(0.1)
            image = img_buffer.get()
            image.flags.writeable = False
            image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
            results = objectron.process(image)

            # Draw the box landmarks on the image.
            image.flags.writeable = True
            image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
            if results.detected_objects:
                for detected_object in results.detected_objects:
                    mp_drawing.draw_landmarks(
                    image, detected_object.landmarks_2d, mp_objectron.BOX_CONNECTIONS)
                    mp_drawing.draw_axis(image, detected_object.rotation,
                                        detected_object.translation)
            # Flip the image horizontally for a selfie-view display.
            cv2.imshow('MediaPipe Objectron', image)
            # cv2.imshow("frame", cv_image)
            # cv2.waitKey(3)
            

if __name__ == '__main__':
    rospy.init_node("object_detection")
    rospy.loginfo("Starting object_detection node")
    object_detection()
    # object_process = multiprocessing.Process(target=object_detection)
    img_buffer_handle_process = threading.Thread(target=img_buffer_handle)
    img_buffer_handle_process.start()
    rospy.spin()

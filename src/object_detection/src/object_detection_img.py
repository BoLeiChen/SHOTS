#!/home/ros/anaconda2/envs/mediapipe/bin python3
import cv2
import mediapipe as mp
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
# import time
mp_drawing = mp.solutions.drawing_utils
mp_objectron = mp.solutions.objectron

class object_detection:
    def __init__(self) -> None:
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/camera/color/image_raw", Image, self.img_callback, queue_size=1)


    def img_callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)
            
        with mp_objectron.Objectron(static_image_mode=True,
                            max_num_objects=5,
                            min_detection_confidence=0.5,
                            model_name='Cup') as objectron:
            
            # Convert the BGR image to RGB and process it with MediaPipe Objectron.
            # start_t = time.time()
            results = objectron.process(cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB))  # 0.05s左右
            # end_t = time.time()
            # print(end_t-start_t)

            # Draw box landmarks.
            if not results.detected_objects:
                print(f'No box landmarks detected on')
            else:  
                annotated_image = cv_image.copy()
                for detected_object in results.detected_objects:
                    mp_drawing.draw_landmarks(
                        annotated_image, detected_object.landmarks_2d, mp_objectron.BOX_CONNECTIONS)
                    mp_drawing.draw_axis(annotated_image, detected_object.rotation,
                                detected_object.translation)
                    cv_image = annotated_image
                    
            cv2.imshow("frame", cv_image)
            cv2.waitKey(3)
            
        # For static images:
        # IMAGE_FILES = []
        # with mp_objectron.Objectron(static_image_mode=True,
        #                     max_num_objects=5,
        #                     min_detection_confidence=0.5,
        #                     model_name='Shoe') as objectron:
        #     for idx, file in enumerate(IMAGE_FILES):
        #         image = cv2.imread(file)
        #         # Convert the BGR image to RGB and process it with MediaPipe Objectron.
        #         results = objectron.process(cv2.cvtColor(image, cv2.COLOR_BGR2RGB))

        #         # Draw box landmarks.
        #         if not results.detected_objects:
        #             print(f'No box landmarks detected on {file}')
        #             continue
        #         print(f'Box landmarks of {file}:')
        #         annotated_image = image.copy()
        #         for detected_object in results.detected_objects:
        #             mp_drawing.draw_landmarks(
        #                 annotated_image, detected_object.landmarks_2d, mp_objectron.BOX_CONNECTIONS)
        #             mp_drawing.draw_axis(annotated_image, detected_object.rotation,
        #                         detected_object.translation)
        #             cv2.imwrite('/tmp/annotated_image' + str(idx) + '.png', annotated_image)
            
        # (rows,cols,channels) = cv_image.shape
        # if cols > 60 and rows > 60 :
        #     cv2.circle(cv_image, (60, 60), 30, (0,0,255), -1)
        # 显示Opencv格式的图像
        # cv2.imshow("Image window", cv_image)
        # cv2.waitKey(3)

        # # 这个时候想将通过opencv处理好的图像再使用ROS节点发送发出去
        # #因此，需要再使用cv2_to_imgmsg将opencv格式额数据转换成ros image格式的数据发布
        # try:
        #     self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
        # except CvBridgeError as e:
        #     print(e)


if __name__ == '__main__':
    rospy.init_node("object_detection")
    rospy.loginfo("Starting object_detection node")
    object_detection()
    rospy.spin()

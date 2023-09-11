#!/home/cbl/anaconda3/bin python3
import cv2
from ultralytics import YOLO

import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Pose
from cv_bridge import CvBridge, CvBridgeError
# from fkie_nbv_planner.srv import SetObjPose, SetObjPoseRequest, SetObjPoseResponse
# Load the YOLOv8 model
model = YOLO('yolov8n.pt')

target_obj = "bottle" # get from ObjGoalNav task
obj_pose = Pose() # rpy -0.000035, -0.000113 -0.804678
obj_pose.position.x = 2.150750
obj_pose.position.y = 0.009582
obj_pose.position.z = 1.000010

# Open the video file
# video_path = "path/to/your/video/file.mp4"
# cap = cv2.VideoCapture(video_path)

class obj_pose_estimation():
    def __init__(self):
        self.obj_pose = Pose() # rpy -0.000035, -0.000113 -0.804678
        
    def get_obj_pose(self, target_obj):
        self.obj_pose.position.x = 2.150750
        self.obj_pose.position.y = 0.009582
        self.obj_pose.position.z = 1.000010
        return self.obj_pose

class yolov8_object_detection:
    def __init__(self) -> None:
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/camera/color/image_raw", Image, self.img_callback, queue_size=1)
        self.obj_pose_pub = rospy.Publisher("/target_obj_pose", Pose, queue_size=1)
        # self.obj_pose_srv_client = rospy.ServiceProxy('set_obj_pose', SetObjPose)
        # self.obj_pose_srv_client.wait_for_service()
        self.obj_pose_estimation = obj_pose_estimation()

    def img_callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)
            
        # Loop through the video frames
        # Read a frame from the video
        # success, frame = cap.read()

        # Run YOLOv8 inference on the frame
        results = model(cv_image)
        # if(target_obj in results):
        #     target_obj_pose = self.obj_pose_estimation.get_obj_pose(target_obj)
        #     self.obj_pose_pub.publish(target_obj_pose)
        target_obj_pose = self.obj_pose_estimation.get_obj_pose(target_obj)
        self.obj_pose_pub.publish(target_obj_pose)

        # Visualize the results on the frame
        annotated_frame = results[0].plot()

        # Display the annotated frame
        cv2.imshow("YOLOv8 Inference", annotated_frame)
        cv2.waitKey(3)


if __name__ == '__main__':
    rospy.init_node("yolov8_object_detection")
    rospy.loginfo("Starting object_detection node")
    yolov8_object_detection()
    rospy.spin()
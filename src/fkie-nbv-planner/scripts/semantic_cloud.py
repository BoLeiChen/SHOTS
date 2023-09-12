#!/home/cbl/anaconda3/envs/Pointr/bin python3
#!coding:utf-8
from __future__ import division
from __future__ import print_function

import sys
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
from sensor_msgs.msg import PointCloud2
from color_pcl_generator import PointType, ColorPclGenerator
import message_filters
from skimage.transform import resize
import cv2
import torch
from std_msgs.msg import Header
from geometry_msgs.msg import Pose
# from fkie_nbv_planner.srv import SetObjPose, SetObjPoseRequest, SetObjPoseResponse
import os
import CLIP.clip as clip
import PIL
import matplotlib.pyplot as plt
from captum.attr import visualization
from CLIP.clip.simple_tokenizer import SimpleTokenizer as _Tokenizer
import time
_tokenizer = _Tokenizer()

clip.clip._MODELS = {
    "ViT-B/32": "https://openaipublic.azureedge.net/clip/models/40d365715913c9da98579312b702a82c18be219cc2a73407c4526f58eba950af/ViT-B-32.pt",
    "ViT-B/16": "https://openaipublic.azureedge.net/clip/models/5806e77cd80f8b59890b7e101eabd078d9fb84e6937f9e85e4ecb61988df416f/ViT-B-16.pt",
    "ViT-L/14": "https://openaipublic.azureedge.net/clip/models/b8cca3fd41ae0c99ba7e8951adf17d267cdb84cd88be6f7c2e0eca1737a03836/ViT-L-14.pt",
}

class color:
   PURPLE = '\033[95m'
   CYAN = '\033[96m'
   DARKCYAN = '\033[36m'
   BLUE = '\033[94m'
   GREEN = '\033[92m'
   YELLOW = '\033[93m'
   RED = '\033[91m'
   BOLD = '\033[1m'
   UNDERLINE = '\033[4m'
   END = '\033[0m'
start_layer =  -1#@param {type:"number"}
start_layer_text =  -1#@param {type:"number"}

device = "cuda" if torch.cuda.is_available() else "cpu"


def interpret(image, texts, model, device, start_layer=start_layer, start_layer_text=start_layer_text):
    batch_size = texts.shape[0]
    images = image.repeat(batch_size, 1, 1, 1)
    logits_per_image, logits_per_text = model(images, texts)
    probs = logits_per_image.softmax(dim=-1).detach().cpu().numpy()
    #print(probs)

    index = [i for i in range(batch_size)]
    one_hot = np.zeros((logits_per_image.shape[0], logits_per_image.shape[1]), dtype=np.float32)
    one_hot[torch.arange(logits_per_image.shape[0]), index] = 1
    one_hot = torch.from_numpy(one_hot).requires_grad_(True)
    one_hot = torch.sum(one_hot.cuda() * logits_per_image)
    model.zero_grad()

    image_attn_blocks = list(dict(model.visual.transformer.resblocks.named_children()).values())

    if start_layer == -1:
      # calculate index of last layer
      start_layer = len(image_attn_blocks) - 1

    num_tokens = image_attn_blocks[0].attn_probs.shape[-1]
    R = torch.eye(num_tokens, num_tokens, dtype=image_attn_blocks[0].attn_probs.dtype).to(device)
    R = R.unsqueeze(0).expand(batch_size, num_tokens, num_tokens)
    for i, blk in enumerate(image_attn_blocks):
        if i < start_layer:
          continue
        grad = torch.autograd.grad(one_hot, [blk.attn_probs], retain_graph=True)[0].detach()
        cam = blk.attn_probs.detach()
        cam = cam.reshape(-1, cam.shape[-1], cam.shape[-1])
        grad = grad.reshape(-1, grad.shape[-1], grad.shape[-1])
        cam = grad * cam
        cam = cam.reshape(batch_size, -1, cam.shape[-1], cam.shape[-1])
        cam = cam.clamp(min=0).mean(dim=1)
        R = R + torch.bmm(cam, R)
    image_relevance = R[:, 0, 1:]


    text_attn_blocks = list(dict(model.transformer.resblocks.named_children()).values())

    if start_layer_text == -1:
      # calculate index of last layer
      start_layer_text = len(text_attn_blocks) - 1

    num_tokens = text_attn_blocks[0].attn_probs.shape[-1]
    R_text = torch.eye(num_tokens, num_tokens, dtype=text_attn_blocks[0].attn_probs.dtype).to(device)
    R_text = R_text.unsqueeze(0).expand(batch_size, num_tokens, num_tokens)
    for i, blk in enumerate(text_attn_blocks):
        if i < start_layer_text:
          continue
        grad = torch.autograd.grad(one_hot, [blk.attn_probs], retain_graph=True)[0].detach()
        cam = blk.attn_probs.detach()
        cam = cam.reshape(-1, cam.shape[-1], cam.shape[-1])
        grad = grad.reshape(-1, grad.shape[-1], grad.shape[-1])
        cam = grad * cam
        cam = cam.reshape(batch_size, -1, cam.shape[-1], cam.shape[-1])
        cam = cam.clamp(min=0).mean(dim=1)
        R_text = R_text + torch.bmm(cam, R_text)
    text_relevance = R_text

    return text_relevance, image_relevance, probs[0][0] > 0.75

def show_image_relevance(image_relevance, image, orig_image):
    # create heatmap from mask on image
    def show_cam_on_image(img, mask):
        heatmap = cv2.applyColorMap(np.uint8(255 * mask), cv2.COLORMAP_JET)
        heatmap = np.float32(heatmap) / 255
        cam = heatmap + np.float32(img)
        # cam = heatmap
        cam = cam / np.max(cam)
        return cam

    dim = int(image_relevance.numel() ** 0.5)
    image_relevance = image_relevance.reshape(1, 1, dim, dim)
    image_relevance = torch.nn.functional.interpolate(image_relevance, size=(480,640), mode='bilinear')
    image_relevance = image_relevance.reshape(480,640).cuda().data.cpu().numpy()
    image_relevance = (image_relevance - image_relevance.min()) / (image_relevance.max() - image_relevance.min())
    image = image.data.cpu().numpy()
    image = (image - image.min()) / (image.max() - image.min())
    vis = show_cam_on_image(image, image_relevance)
    vis = np.uint8(255 * vis)
    vis = cv2.cvtColor(np.array(vis), cv2.COLOR_RGB2BGR)
    return vis

def show_heatmap_on_text(text, text_encoding, R_text):
  CLS_idx = text_encoding.argmax(dim=-1)
  R_text = R_text[CLS_idx, 1:CLS_idx]
  text_scores = R_text / R_text.sum()
  text_scores = text_scores.flatten()
  #print(text_scores)
  text_tokens=_tokenizer.encode(text)
  text_tokens_decoded=[_tokenizer.decode([a]) for a in text_tokens]
  vis_data_records = [visualization.VisualizationDataRecord(text_scores,0,0,0,0,0,text_tokens_decoded,1)]
  visualization.visualize_text(vis_data_records)

def color_map(N=256, normalized=False):
    """
    Return Color Map in PASCAL VOC format (rgb)
    \param N (int) number of classes
    \param normalized (bool) whether colors are normalized (float 0-1)
    \return (Nx3 numpy array) a color map
    """
    def bitget(byteval, idx):
        return ((byteval & (1 << idx)) != 0)
    dtype = 'float32' if normalized else 'uint8'
    cmap = np.zeros((N, 3), dtype=dtype)
    for i in range(N):
        r = g = b = 0
        c = i
        for j in range(8):
            r = r | (bitget(c, 0) << 7-j)
            g = g | (bitget(c, 1) << 7-j)
            b = b | (bitget(c, 2) << 7-j)
            c = c >> 3
        cmap[i] = np.array([r, g, b])
    cmap = cmap/255.0 if normalized else cmap
    return cmap

def decode_segmap(temp, n_classes, cmap):
    """
    Given an image of class predictions, produce an bgr8 image with class colors
    \param temp (2d numpy int array) input image with semantic classes (as integer)
    \param n_classes (int) number of classes
    \cmap (Nx3 numpy array) input color map
    \return (numpy array bgr8) the decoded image with class colors
    """
    r = temp.copy()
    g = temp.copy()
    b = temp.copy()
    for l in range(0, n_classes):
        r[temp == l] = cmap[l,0]
        g[temp == l] = cmap[l,1]
        b[temp == l] = cmap[l,2]
    bgr = np.zeros((temp.shape[0], temp.shape[1], 3))
    bgr[:, :, 0] = b
    bgr[:, :, 1] = g
    bgr[:, :, 2] = r
    return bgr.astype(np.uint8)

class SemanticCloud:

    def __init__(self, gen_pcl = True):

        # Get point type
        point_type = 1 #rospy.get_param('/semantic_pcl/point_type')
        if point_type == 0:
            self.point_type = PointType.COLOR
            print('Generate color point cloud.')
        elif point_type == 1:
            self.point_type = PointType.SEMANTICS_MAX
            print('Generate semantic point cloud [max fusion].')
        elif point_type == 2:
            self.point_type = PointType.SEMANTICS_BAYESIAN
            print('Generate semantic point cloud [bayesian fusion].')
        else:
            print("Invalid point type.")
            return
        # Get image size
        self.img_width, self.img_height = 640, 480 #rospy.get_param('/camera/width'), rospy.get_param('/camera/height')
        # Set up CNN is use semantics
        if self.point_type is not PointType.COLOR:
            print('Setting up CLIP model...')
            # Set device
            self.device = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")
            self.model, self.preprocess = clip.load("ViT-B/32", device=device, jit=False)
            self.n_classes = 2
            self.cmap = color_map(N = self.n_classes, normalized = False) # Color map for semantic classes
        # Declare array containers
        if self.point_type is PointType.SEMANTICS_BAYESIAN:
            self.semantic_colors = np.zeros((3, self.img_height, self.img_width, 3), dtype = np.uint8) # Numpy array to store 3 decoded semantic images with highest confidences
            self.confidences = np.zeros((3, self.img_height, self.img_width), dtype = np.float32) # Numpy array to store top 3 class confidences
        # Set up ROS
        print('Setting up ROS...')
        self.bridge = CvBridge() # CvBridge to transform ROS Image message to OpenCV image
        # Semantic image publisher
        self.sem_img_pub = rospy.Publisher("/semantic_pcl/semantic_image", Image, queue_size = 1)
        # Set up ros image subscriber
        # Set buff_size to average msg size to avoid accumulating delay
        if gen_pcl:
            # Point cloud frame id
            frame_id = "/camera_color_optical_frame"#rospy.get_param('/semantic_pcl/frame_id')
            # Camera intrinsic matrix
            fx = 544.771755 #rospy.get_param('/camera/fx')
            fy = 546.966312 #rospy.get_param('/camera/fy')
            cx = 322.376103 #rospy.get_param('/camera/cx')
            cy = 245.357925 #rospy.get_param('/camera/cy')
            intrinsic = np.matrix([[fx, 0, cx], [0, fy, cy], [0, 0, 1]], dtype = np.float32)
            self.pcl_pub = rospy.Publisher("/semantic_pcl/semantic_pcl", PointCloud2, queue_size = 1)
            self.color_sub = message_filters.Subscriber("/camera/color/image_raw", Image, queue_size = 1, buff_size = 30*480*640)
            self.depth_sub = message_filters.Subscriber("/camera/depth/image_rect_raw", Image, queue_size = 1, buff_size = 40*480*640 ) # increase buffer size to avoid delay (despite queue_size = 1)
            self.ts = message_filters.ApproximateTimeSynchronizer([self.color_sub, self.depth_sub], queue_size = 1, slop = 0.3) # Take in one color image and one depth image with a limite time gap between message time stamps
            self.ts.registerCallback(self.clip_callback)
            self.cloud_generator = ColorPclGenerator(intrinsic, self.img_width,self.img_height, frame_id , self.point_type)
        else:
            self.image_sub = rospy.Subscriber(rospy.get_param('/semantic_pcl/color_image_topic'), Image, self.color_callback, queue_size = 1, buff_size = 30*480*640)
        print('Ready.')

    def clip_callback_old(self, color_img_ros, depth_img_ros):
        try:
            color_img = self.bridge.imgmsg_to_cv2(color_img_ros, "bgr8")
            depth_img = self.bridge.imgmsg_to_cv2(depth_img_ros, "32FC1")
        except CvBridgeError as e:
            print(e)        
        # Resize depth
        if depth_img.shape[0] is not self.img_height or depth_img.shape[1] is not self.img_width:
            depth_img = resize(depth_img, (self.img_height, self.img_width), order = 0, mode = 'reflect', anti_aliasing=False, preserve_range = True) # order = 0, nearest neighbour
            depth_img = depth_img.astype(np.float32)

        img = self.preprocess(PIL.Image.fromarray(color_img)).unsqueeze(0).to(device)
        # texts = ["bottle", "no bottle"]
        texts = ["traffic signal", "no traffic signal"]
        text = clip.tokenize(texts).to(device)

        R_text, R_image, detect_success = interpret(model=self.model, image=img, texts=text, device=device)
        R_text = R_text[0]
        R_image = R_image[0]

        dim = int(R_image.numel() ** 0.5)
        R_image = R_image.reshape(1, 1, dim, dim)
        R_image = torch.nn.functional.interpolate(R_image, size=(480,640), mode='bilinear')[0,0,...].cpu().numpy()
        heatmap = (R_image - R_image.min()) / (R_image.max() - R_image.min())
        heatmap = cv2.applyColorMap(np.uint8(255 * heatmap), cv2.COLORMAP_JET)     #热力图

        color_img_ = (color_img - color_img.min()) / (color_img.max() - color_img.min())
        heatmap = np.float32(heatmap) / 255
        cam = heatmap + np.float32(color_img_)                                       #热力图和原始图像叠加
        cam = cam / np.max(cam)
        vis = np.uint8(255 * cam)
        vis = cv2.cvtColor(np.array(vis), cv2.COLOR_RGB2BGR)

        #cv2.imshow("Clip Inference", heatmap)
        #cv2.waitKey(3)
        time2 = time.clock()
        #print("time:", time2 - time1)
        
        pred_confidence = (R_image - R_image.min()) / (R_image.max() - R_image.min())
        if detect_success:
            pred_label = np.where(R_image >= 0.02, 2, 0)
            print(R_image.max())
        else:
            pred_label = np.zeros_like(R_image)

        pred_label = resize(pred_label, (self.img_height, self.img_width), order = 0, mode = 'reflect', anti_aliasing=False, preserve_range = True) # order = 0, nearest neighbour
        pred_label = pred_label.astype(np.int)
        semantic_color = decode_segmap(pred_label, self.n_classes, self.cmap)
        pred_confidence = resize(pred_confidence, (self.img_height, self.img_width),  mode = 'reflect', anti_aliasing=True, preserve_range = True)

        cloud_ros = self.cloud_generator.generate_cloud_semantic_max(color_img, depth_img, semantic_color, pred_confidence, color_img_ros.header.stamp)

        # Publish semantic image
        if self.sem_img_pub.get_num_connections() > 0:
            if self.point_type is PointType.SEMANTICS_MAX:
                ros_img = Image()
                header = Header(stamp=rospy.Time.now())
                header.frame_id = 'result'            
                ros_img.encoding = 'rgb8'            
                ros_img.header = header
                ros_img.height = vis.shape[0]
                ros_img.width = vis.shape[1]                
                ros_img.step = vis.shape[1] * vis.shape[2]
                ros_img.data = np.array(vis).tostring()
                self.sem_img_pub.publish(ros_img)

        self.pcl_pub.publish(cloud_ros)

    def clip_callback(self, color_img_ros, depth_img_ros):
        try:
            color_img = self.bridge.imgmsg_to_cv2(color_img_ros, "bgr8")
            depth_img = self.bridge.imgmsg_to_cv2(depth_img_ros, "32FC1")
        except CvBridgeError as e:
            print(e)        
        # Resize depth
        if depth_img.shape[0] is not self.img_height or depth_img.shape[1] is not self.img_width:
            depth_img = resize(depth_img, (self.img_height, self.img_width), order = 0, mode = 'reflect', anti_aliasing=False, preserve_range = True) # order = 0, nearest neighbour
            depth_img = depth_img.astype(np.float32)

        img = self.preprocess(PIL.Image.fromarray(color_img)).unsqueeze(0).to(device)
        # texts = ["bottle", "no bottle"]
        texts = ["person"]
        text = clip.tokenize(texts).to(device)

        R_text, R_image, detect_success = interpret(model=self.model, image=img, texts=text, device=device)
        R_text = R_text[0]
        R_image = R_image[0]

        dim = int(R_image.numel() ** 0.5)
        R_image = R_image.reshape(1, 1, dim, dim)
        R_image = torch.nn.functional.interpolate(R_image, size=(480,640), mode='bilinear')[0,0,...].cpu().numpy()
        heatmap = (R_image - R_image.min()) / (R_image.max() - R_image.min())
        heatmap_ = cv2.applyColorMap(np.uint8(255 * heatmap), cv2.COLORMAP_JET)     #热力图
        color_img_ = (color_img - color_img.min()) / (color_img.max() - color_img.min())
        heatmap_ = np.float32(heatmap_) / 255
        cam = heatmap_ + np.float32(color_img_)                                       #热力图和原始图像叠加
        cam = cam / np.max(cam)
        vis = np.uint8(255 * cam)
        vis = cv2.cvtColor(np.array(vis), cv2.COLOR_RGB2BGR)

        # semantic_class = (heatmap * 10).astype(np.int)
        print(np.max(R_image))
        semantic_class = np.where(R_image >= 0.02, 1, 0)
        # print(np.max(heatmap))

        pred_label = resize(semantic_class, (self.img_height, self.img_width), order = 0, mode = 'reflect', anti_aliasing=False, preserve_range = True) # order = 0, nearest neighbour
        pred_label = pred_label.astype(np.int)
        semantic_color = decode_segmap(pred_label, self.n_classes, self.cmap)
        pred_confidence = resize(R_image, (self.img_height, self.img_width),  mode = 'reflect', anti_aliasing=True, preserve_range = True)

        cloud_ros = self.cloud_generator.generate_cloud_semantic_max(color_img, depth_img, semantic_color, pred_confidence, color_img_ros.header.stamp)

        # Publish semantic image
        if self.sem_img_pub.get_num_connections() > 0:
            if self.point_type is PointType.SEMANTICS_MAX:
                ros_img = Image()
                header = Header(stamp=rospy.Time.now())
                header.frame_id = 'result'            
                ros_img.encoding = 'rgb8'            
                ros_img.header = header
                ros_img.height = vis.shape[0]
                ros_img.width = vis.shape[1]                
                ros_img.step = vis.shape[1] * vis.shape[2]
                ros_img.data = np.array(vis).tostring()
                self.sem_img_pub.publish(ros_img)

        self.pcl_pub.publish(cloud_ros)

    def color_callback(self, color_img_ros):
        """
        Callback function for color image, de semantic segmantation and show the decoded image. For test purpose
        \param color_img_ros (sensor_msgs.Image) input ros color image message
        """
        print('callback')
        try:
            color_img = self.bridge.imgmsg_to_cv2(color_img_ros, "bgr8") # Convert ros msg to numpy array
        except CvBridgeError as e:
            print(e)
        # Do semantic segmantation
        class_probs = self.predict(color_img)
        confidence, label = class_probs.max(1)
        confidence, label = confidence.squeeze(0).numpy(), label.squeeze(0).numpy()
        label = resize(label, (self.img_height, self.img_width), order = 0, mode = 'reflect', anti_aliasing=False, preserve_range = True) # order = 0, nearest neighbour
        label = label.astype(np.int)
        # Add semantic class colors
        decoded = decode_segmap(label, self.n_classes, self.cmap)        # Show input image and decoded image
        confidence = resize(confidence, (self.img_height, self.img_width),  mode = 'reflect', anti_aliasing=True, preserve_range = True)
        cv2.imshow('Camera image', color_img)
        cv2.imshow('confidence', confidence)
        cv2.imshow('Semantic segmantation', decoded)
        cv2.waitKey(3)

    def color_depth_callback(self, color_img_ros, depth_img_ros):
        """
        Callback function to produce point cloud registered with semantic class color based on input color image and depth image
        \param color_img_ros (sensor_msgs.Image) the input color image (bgr8)
        \param depth_img_ros (sensor_msgs.Image) the input depth image (registered to the color image frame) (float32) values are in meters
        """
        try:
            color_img = self.bridge.imgmsg_to_cv2(color_img_ros, "bgr8")
            depth_img = self.bridge.imgmsg_to_cv2(depth_img_ros, "32FC1")
        except CvBridgeError as e:
            print(e)

        # Resize depth
        if depth_img.shape[0] is not self.img_height or depth_img.shape[1] is not self.img_width:
            depth_img = resize(depth_img, (self.img_height, self.img_width), order = 0, mode = 'reflect', anti_aliasing=False, preserve_range = True) # order = 0, nearest neighbour
            depth_img = depth_img.astype(np.float32)
        if self.point_type is PointType.COLOR:
            cloud_ros = self.cloud_generator.generate_cloud_color(color_img, depth_img, color_img_ros.header.stamp)
        else:
            # Do semantic segmantation
            if self.point_type is PointType.SEMANTICS_MAX:
                semantic_color, pred_confidence = self.predict_max(color_img)
                cloud_ros = self.cloud_generator.generate_cloud_semantic_max(color_img, depth_img, semantic_color, pred_confidence, color_img_ros.header.stamp)

            elif self.point_type is PointType.SEMANTICS_BAYESIAN:
                self.predict_bayesian(color_img)
                # Produce point cloud with rgb colors, semantic colors and confidences
                cloud_ros = self.cloud_generator.generate_cloud_semantic_bayesian(color_img, depth_img, self.semantic_colors, self.confidences, color_img_ros.header.stamp)

            # Publish semantic image
            if self.sem_img_pub.get_num_connections() > 0:
                if self.point_type is PointType.SEMANTICS_MAX:
                    semantic_color_msg = self.bridge.cv2_to_imgmsg(semantic_color, encoding="bgr8")
                else:
                    semantic_color_msg = self.bridge.cv2_to_imgmsg(self.semantic_colors[0], encoding="bgr8")
                self.sem_img_pub.publish(ros_img)

        # Publish point cloud
        self.pcl_pub.publish(cloud_ros)

    def predict_max(self, img):
        """
        Do semantic prediction for max fusion
        \param img (numpy array rgb8)
        """
        class_probs = self.predict(img) #N,C,W,H(1,150,473,473)
        # Take best prediction and confidence
        pred_confidence, pred_label = class_probs.max(1)
        #pred_confidence(1*473*473)每个值对应pred_label每个类别的置信度
        #pred_label(1*473*473)每个值对应每个像素点所属的类别标签
        pred_confidence = pred_confidence.squeeze(0).cpu().numpy()
        pred_label = pred_label.squeeze(0).cpu().numpy()
        pred_label = resize(pred_label, (self.img_height, self.img_width), order = 0, mode = 'reflect', anti_aliasing=False, preserve_range = True) # order = 0, nearest neighbour
        pred_label = pred_label.astype(np.int)
        # Add semantic color
        #for i in range(0,640):
        #    for j in range(0,480):
        #        if (pred_label[j][i]) != 5:
        #            pred_label[j][i]= -1
        semantic_color = decode_segmap(pred_label, self.n_classes, self.cmap)
        pred_confidence = resize(pred_confidence, (self.img_height, self.img_width),  mode = 'reflect', anti_aliasing=True, preserve_range = True)
        return (semantic_color, pred_confidence)

    def predict_bayesian(self, img):
        """
        Do semantic prediction for bayesian fusion
        \param img (numpy array rgb8)
        """
        class_probs = self.predict(img)
        # Take 3 best predictions and their confidences (probabilities)
        pred_confidences, pred_labels  = torch.topk(input = class_probs, k = 3, dim = 1, largest = True, sorted = True)
        pred_labels = pred_labels.squeeze(0).cpu().numpy()
        pred_confidences = pred_confidences.squeeze(0).cpu().numpy()
        # Resize predicted labels and confidences to original image size
        for i in range(pred_labels.shape[0]):
            pred_labels_resized = resize(pred_labels[i], (self.img_height, self.img_width), order = 0, mode = 'reflect', anti_aliasing=False, preserve_range = True) # order = 0, nearest neighbour
            pred_labels_resized = pred_labels_resized.astype(np.int)
            # Add semantic class colors
            self.semantic_colors[i] = decode_segmap(pred_labels_resized, self.n_classes, self.cmap)
        for i in range(pred_confidences.shape[0]):
            self.confidences[i] = resize(pred_confidences[i], (self.img_height, self.img_width),  mode = 'reflect', anti_aliasing=True, preserve_range = True)

    def predict(self, img):
        """
        Do semantic segmantation
        \param img: (numpy array bgr8) The input cv image
        """
        img = img.copy() # Make a copy of image because the method will modify the image
        #orig_size = (img.shape[0], img.shape[1]) # Original image size
        # Prepare image: first resize to CNN input size then extract the mean value of SUNRGBD dataset. No normalization
        img = resize(img, self.cnn_input_size, mode = 'reflect', anti_aliasing=True, preserve_range = True) # Give float64
        img = img.astype(np.float32)
        img -= self.mean
        # Convert HWC -> CHW
        img = img.transpose(2, 0, 1)
        # Convert to tensor
        img = torch.tensor(img, dtype = torch.float32)
        img = img.unsqueeze(0) # Add batch dimension required by CNN
        with torch.no_grad():
            img = img.to(self.device)
            # Do inference
            since = time.time()
            outputs = self.model(img) #N,C,W,H(1,150,473,473)
            # Apply softmax to obtain normalized probabilities
            outputs = torch.nn.functional.softmax(outputs, 1)
            return outputs


def main(args):
    rospy.init_node('semantic_cloud', anonymous=True)
    seg_cnn = SemanticCloud(gen_pcl = True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

if __name__ == '__main__':
    main(sys.argv)

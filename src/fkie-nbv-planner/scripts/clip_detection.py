# -*- coding: utf-8 -*-
#!/home/cbl/anaconda3/envs/Pointr/bin python3
import ctypes
libgcc_s = ctypes.CDLL('libgcc_s.so.1')
import cv2

import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Header
from geometry_msgs.msg import Pose
# from fkie_nbv_planner.srv import SetObjPose, SetObjPoseRequest, SetObjPoseResponse
import numpy as np
import os
import torch
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

device = "cuda" if torch.cuda.is_available() else "cpu"
model, preprocess = clip.load("ViT-B/32", device=device, jit=False)

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

target_obj = "bottle" # get from ObjGoalNav task
obj_pose = Pose() # rpy -0.000035, -0.000113 -0.804678
obj_pose.position.x = 2.150750
obj_pose.position.y = 0.009582
obj_pose.position.z = 1.000010

#@title Control context expansion (number of attention layers to consider)
#@title Number of layers for image Transformer
start_layer =  -1#@param {type:"number"}

#@title Number of layers for text Transformer
start_layer_text =  -1#@param {type:"number"}

def interpret(image, texts, model, device, start_layer=start_layer, start_layer_text=start_layer_text):
    batch_size = texts.shape[0]
    images = image.repeat(batch_size, 1, 1, 1)
    logits_per_image, logits_per_text = model(images, texts)
    probs = logits_per_image.softmax(dim=-1).detach().cpu().numpy()
    #print(probs)
    #print(probs[0][0] > 0.75)
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

    return text_relevance, image_relevance

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
  print(text_scores)
  text_tokens=_tokenizer.encode(text)
  text_tokens_decoded=[_tokenizer.decode([a]) for a in text_tokens]
  vis_data_records = [visualization.VisualizationDataRecord(text_scores,0,0,0,0,0,text_tokens_decoded,1)]
  visualization.visualize_text(vis_data_records)

##########################################################

class obj_pose_estimation():
    def __init__(self):
        self.obj_pose = Pose() # rpy -0.000035, -0.000113 -0.804678
        
    def get_obj_pose(self, target_obj):
        self.obj_pose.position.x = 2.150750
        self.obj_pose.position.y = 0.009582
        self.obj_pose.position.z = 1.000010
        return self.obj_pose

class clip_object_detection:
    def __init__(self):
        self.image_sub = rospy.Subscriber("/camera/color/image_raw", Image, self.img_callback, queue_size=5)
        self.obj_pose_pub = rospy.Publisher("/target_obj_pose", Pose, queue_size=1)
        # self.obj_pose_srv_client = rospy.ServiceProxy('set_obj_pose', SetObjPose)
        # self.obj_pose_srv_client.wait_for_service()
        self.obj_pose_estimation = obj_pose_estimation()
        self.clip_result_pub = rospy.Publisher("/clip_result", Image, queue_size=5)


    def img_callback(self, data):
        time1 = time.clock()
        cv_image = np.frombuffer(data.data, dtype=np.uint8).reshape(data.height, data.width, -1)
        
        img = preprocess(PIL.Image.fromarray(cv_image)).unsqueeze(0).to(device)
        texts = ["Stop"]
        text = clip.tokenize(texts).to(device)

        R_text, R_image = interpret(model=model, image=img, texts=text, device=device)

        dim = int(R_image.numel() ** 0.5)
        R_image = R_image.reshape(1, 1, dim, dim)
        R_image = torch.nn.functional.interpolate(R_image, size=(480,640), mode='bilinear')[0,0,...].cpu().numpy()
        heatmap = (R_image - R_image.min()) / (R_image.max() - R_image.min())
        heatmap = cv2.applyColorMap(np.uint8(255 * heatmap), cv2.COLORMAP_JET)     #热力图

        cv_image = (cv_image - cv_image.min()) / (cv_image.max() - cv_image.min())
        heatmap = np.float32(heatmap) / 255
        cam = heatmap + np.float32(cv_image)                                       #热力图和原始图像叠加
        cam = cam / np.max(cam)
        vis = np.uint8(255 * cam)
        vis = cv2.cvtColor(np.array(vis), cv2.COLOR_RGB2BGR)

        #cv2.imshow("Clip Inference", heatmap)
        #cv2.waitKey(3)

        ros_img = Image()
        header = Header(stamp=rospy.Time.now())
        header.frame_id = 'result'            
        ros_img.encoding = 'rgb8'            
        ros_img.header = header
        ros_img.height = vis.shape[0]
        ros_img.width = vis.shape[1]                
        ros_img.step = vis.shape[1] * vis.shape[2]
        ros_img.data = np.array(vis).tostring()
        self.clip_result_pub.publish(ros_img)
        time2 = time.clock()
        print("time:", time2 - time1)

        '''
        batch_size = text.shape[0]
        for i in range(batch_size):
          #show_heatmap_on_text(texts[i], text[i], R_text[i])
          vis = show_image_relevance(R_image[i], torch.from_numpy(cv_image), orig_image=PIL.Image.fromarray(cv_image))
          #plt.show()
          ros_img = Image()
          header = Header(stamp=rospy.Time.now())
          header.frame_id = 'result'            
          ros_img.encoding = 'rgb8'            
          ros_img.header = header
          ros_img.height = vis.shape[0]
          ros_img.width = vis.shape[1]                
          ros_img.step = vis.shape[1] * vis.shape[2]
          ros_img.data = np.array(vis).tostring()
          self.clip_result_pub.publish(ros_img)
        '''

        # if(target_obj in results):
        #     target_obj_pose = self.obj_pose_estimation.get_obj_pose(target_obj)
        #     self.obj_pose_pub.publish(target_obj_pose)
        #target_obj_pose = self.obj_pose_estimation.get_obj_pose(target_obj)
        #self.obj_pose_pub.publish(target_obj_pose)

        # Display the annotated frame
        #cv2.imshow("Clip Inference", annotated_frame)
        #cv2.waitKey(3)


if __name__ == '__main__':
    rospy.init_node("clip_object_detection")
    rospy.loginfo("Starting object_detection node")
    clip_object_detection()
    rospy.spin()

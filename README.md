# SHOTS: Semantic-Heuristic Object Target Search with Object Belief Field and Flexible Vision

Bolei Chen, Yongzheng Cui, Haonan Yang, Ping Zhong, Yu Sheng, Miao Li, Jianxin Wang

## 1. Overview

<img src="https://github.com/BoLeiChen/VM-SHOTS/assets/60416370/d0bfa21c-523c-4c2d-8db6-30e3ac121d44" width="80%" ></img>

Object Target Search (OTS) tasks require robots to localize and navigate to objects specified by semantic labels, (e.g., find a fire extinguisher). Many existing OTS methods strongly rely on the semantic co occurrence relations among objects to imagine and localize the object targets seen in the learning phase. However, simplistic domestic and chaotic rescue scenarios often fail to provide rich semantics even require the robot to navigate to novel object instances. In addition, most of the existing OTS methods agree on a fixed Field of View (FoV) setting relative to the robot base, thus making it challenging to seek obscured objects (e.g., a shoe under the bed). To alleviate the above problems, we propose a semantic-heuristic OTS algorithm by promptly updating an Object Belief Field (OBF) for localizing object targets and designing a utility function for balancing exploration and exploitation. In particular, our method employs a flexible vision and allows the manipulator to translate and rotate the robot’s FoV to look around or even inspect hidden corners obscured by obstacles. Sufficient comparative and ablation studies validate that our method significantly improves the success rates and SPL metrics relative to the baselines. Furthermore, real-world experiments demonstrate our method can find novel objects without requiring rich scene priors.

This repository contains code for  VM-SHOTS.

## 2. A demo video for SHOTS

This is a demo video of the experiments for the paper titled “SHOTS: Semantic-Heuristic Object Target Search with Object Belief Field and Flexible Vision”.

[![SHOTS](https://res.cloudinary.com/marcomontalbano/image/upload/v1694594948/video_to_markdown/images/youtube--mzsFSr4e3sQ-c05b58ac6eb4c4700831b2b3070cd403.jpg)](https://www.youtube.com/watch?v=mzsFSr4e3sQ "SHOTS")

## 3. Installation

Clone
```sh
git clone https://github.com/BoLeiChen/VM-SHOTS.git
```
Compile
```sh
cd VM-SHOTS
catkin build
```
Configure the conda environment
```
cd ..
git clone https://github.com/ultralytics/ultralytics.git
conda create -n SHOTS python=3.7
conda activate SHOTS
cd ultralytics
pip install -r requirements.txt -i https://mirrors.bfsu.edu.cn/pypi/web/simple/
pip install ultralytics
```

## 4. Run

Sumilation enviromnent
```
source ./devel/setup.bash
roslaunch husky_ur5 husky_empty_world.launch
```

Local planner
```
source ./devel/setup.bash
roslaunch vehicle_simulator system_real_robot.launch
```

NBV Planner
```
source ./devel/setup.bash
roslaunch fkie_nbv_planner run_planner.launch
```

Load CLIP(Run in SHOTS environment)
```
source activate SHOTS
cd ./src/fkie-nbv-planner/scripts
python semantic_cloud.py
```

**Experimental diagram**

<img src="https://github.com/BoLeiChen/VM-SHOTS/assets/60416370/367452e7-451c-430b-9bb9-29c0649020db" width="80%" ></img>

We simulate the search for casualties scenario in a 10 x 10 x 2 ($m^3$) sized simulation. The VM-robot uses a depth camera to acquire RGB images and depth images for CLIP object localization and Octomap construction, respectively. In the above figure, the robot first localizes to the target object via CLIP using the acquired RGB image (a) and maps its confidence into the local Octomap (b). Finally, the VM-robot switches from exploration to search mode (d) when the evaluated object confidence and its surrounding uncertainty exceed a preset threshold.

## 5. Real robot experiment

<img src="https://github.com/BoLeiChen/VM-SHOTS/assets/60416370/af7c2157-6f41-4fbf-a1da-b0b06eb0fec0" width="80%" ></img>

We deploy VM-SHOTS to the Husky-UR5 robotics platform to conduct experiments in a 8 × 15 × 6 ($m^3$) sized real indoor scenario. We employ the Nvidia Jetson AGX Xavier as the robot’s onboard computer to organize the onboard sensors, run the Husky and UR5 controllers, and execute the associated motion planning algorithms. The CLIP-based image inference and VM-SHOTS are executed on a workstation with an Intel i9-11900K CPU at 3.50 GHz with 16 cores and an NVIDIA 3090 Graphics Card. Similar to the task setup in the simulation, the robot is required to search for a humanoid robot and a fire extinguisher in an indoor scene with a size of almost. The right side shows the depth image captured by the camera and the CLIP-based object belief map inference for humanoid robot localization.

## 6. License
This repository is released under the () license. See [LICENSE](https://github.com/BoLeiChen/VM-SHOTS/blob/main/LICENSE) for additional details.

## 7. Acknowledgement
Code acknowledgements:
 - `src/fkie-nbv-planner/scripts/semantic_cloud.py` modified from [Chefer et al.'s codebase](https://github.com/hila-chefer/Transformer-MM-Explainability), which was in turn modified from the original [CLIP codebase](https://github.com/openai/CLIP).
 

**Coming soon ...**


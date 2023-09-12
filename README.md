# VM-SHOTS: Semantics-Heuristic Zero-Shot Object Target Search with Vehicle-Manipulator Cooperation

Bolei Chen, Yongzheng Cui, Haonan Yang, Ping Zhong, Yu Sheng, Jianxin Wang

Central South University

## 1. Overview

<img src="https://github.com/BoLeiChen/VM-SHOTS/assets/60416370/d0bfa21c-523c-4c2d-8db6-30e3ac121d44" width="80%" ></img>

Object Target Search (OTS) as an essential robotic technique is considered to have promising applications in rescue robots and domestic service assistants. OTS tasks require robots to localize and navigate to objects specified by semantic labels, (e.g., find a human). Many existing OTS methods strongly rely on the subordination and co-occurrence relations among objects to imagine and localize the previously seen object instances. However, simplistic domestic and chaotic rescue scenarios often fail to provide rich semantics even require the robot to navigate to novel objects. In addition, the vast majority of existing OTS methods agree on a fixed Field of View (FoV) setting relative to the robot’s body, thus making it challenging to seek hidden objects (e.g., a human under the bed). To alleviate the above problems, we propose a semantic-heuristic zero-shot OTS framework by combining a sample-based explorer with various visual language model-based object localizers. In particular, we propose tackling the OTS problem with VM cooperation, which allows the manipulator to translate and rotate the FoV to look around or even inspect hidden corners obscured by obstacles. Sufficient omparative experiments demonstrate that our approach is more practice-oriented, i.e., capable of finding diverse objects without requiring rich scene priors. Furthermore, real-world experiments demonstrate our method can be migrated to the real robotic platform. The video of the simulation and real robot experiments is available here: https://www.youtube.com/watch?v=ZK8YKOSjwOk.

This repository contains code for  VM-SHOTS.

## 2. Installation

Clone
```sh
git clone https://github.com/BoLeiChen/VM-SHOTS.git
```
Compile
```sh
cd VM-SHOTS
catkin build
```
Configure yolov8
```
cd ..
git clone https://github.com/ultralytics/ultralytics.git
conda create -n yolov8 python=3.7
conda activate yolov8
cd ultralytics
pip install -r requirements.txt -i https://mirrors.bfsu.edu.cn/pypi/web/simple/
pip install ultralytics
```

## 3. Run

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

Load CLIP(Run in yolov8 environment)
```
source activate yolov8
cd ./src/fkie-nbv-planner/scripts
python semantic_cloud.py
```

**Experimental diagram**

<img src="https://github.com/BoLeiChen/VM-SHOTS/assets/60416370/367452e7-451c-430b-9bb9-29c0649020db" width="80%" ></img>

We simulate the search for casualties scenario in a 10 x 10 x 2 ($m^3$) sized simulation. The VM-robot uses a depth camera to acquire RGB images and depth images for CLIP object localization and Octomap construction, respectively. In the above figure, the robot first localizes to the target object via CLIP using the acquired RGB image (a) and maps its confidence into the local Octomap (b). Finally, the VM-robot switches from exploration to search mode (d) when the evaluated object confidence and its surrounding uncertainty exceed a preset threshold.

## 4. Real robot experiment

<img src="https://github.com/BoLeiChen/VM-SHOTS/assets/60416370/af7c2157-6f41-4fbf-a1da-b0b06eb0fec0" width="80%" ></img>

We deploy VM-SHOTS to the Husky-UR5 robotics platform to conduct experiments in a 8 × 15 × 6 ($m^3$) sized real indoor scenario. We employ the Nvidia Jetson AGX Xavier as the robot’s onboard computer to organize the onboard sensors, run the Husky and UR5 controllers, and execute the associated motion planning algorithms. The CLIP-based image inference and VM-SHOTS are executed on a workstation with an Intel i9-11900K CPU at 3.50 GHz with 16 cores and an NVIDIA 3090 Graphics Card. Similar to the task setup in the simulation, the robot is required to search for a humanoid robot and a fire extinguisher in an indoor scene with a size of almost. The right side shows the depth image captured by the camera and the CLIP-based object belief map inference for humanoid robot localization.

## 5. License
This repository is released under the () license. See [LICENSE](https://github.com/BoLeiChen/VM-SHOTS/blob/main/LICENSE) for additional details.

## 6. Acknowledgement


**Coming soon ...**


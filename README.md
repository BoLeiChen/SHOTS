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

## 5. License
This repository is released under the MIT license. See [LICENSE](https://github.com/BoLeiChen/VM-SHOTS/blob/main/LICENSE) for additional details.

## 6. Acknowledgement


**Coming soon ...**


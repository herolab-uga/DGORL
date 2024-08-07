# DGORL
DGORL: Distributed Graph Optimization based Relative Localization of Multi-Robot Systems

## Overview
By creating relative position-weighted connectivity graphs using RSSI as local sensor data, expanding these graphs based on potential positions at a particular location, and then further optimizing to obtain relative position estimates for all connected robots, DGORL seeks to efficiently achieve high localization accuracy. An overview of the DGORL configuration space can be found in Figure below:

![Overview](/figures/graph_mrl_overview.png)

## Demonstration of DGROL implementation on real robots: Here, five swarm robots are executing a simple multi-robot rendezvous algorithm to group together. The left side shows the robot's overhead video, the center plot shows the actual trajectory based on AprilTag tracking, the right plot shows the DGROL's localization (with Graph) outcome.

<p align="center">
<img src="https://github.com/herolab-uga/DGORL/blob/main/figures/dgrorl-formation-pentagon.gif" width="600">
</p>

## Paper Citation
If you use this work in your research, please cite it as 
Latif, E., Parasuraman, R. (2024). DGORL: Distributed Graph Optimization Based Relative Localization of Multi-robot Systems. In: Bourgeois, J., et al. Distributed Autonomous Robotic Systems. DARS 2022. Springer Proceedings in Advanced Robotics, vol 28. Springer, Cham. https://doi.org/10.1007/978-3-031-51497-5_18

Paper Link: https://link.springer.com/chapter/10.1007/978-3-031-51497-5_18

The preprint of the paper is available in ARXiv at https://arxiv.org/abs/2210.01662

## Installation Requirements
* C++ requirements.   
([pybind11](https://github.com/pybind/pybind11) is also required, but it's built in this repository, you don't need to install)
* python 3.6+
* [g2o installed](https://github.com/uoip/g2opy.git)

### g2o Installation
```
$ git clone https://github.com/uoip/g2opy.git
$ cd g2opy
$ mkdir build
$ cd build
$ cmake ..
$ make -j8
$ cd ..
$ python setup.py install
```
Tested under Ubuntu 16.04, Python 3.6+.

## About Repository
This repository contains script *graph_optimization_mrl.py* to simulate the DGORL on 60 x 60 m workspace based on g2o optimizer along with the graph input files in [g2o format](https://github.com/uoip/g2opy.git).


## How to run
```
$ git clone https://github.com/herolab-uga/DGORL.git
$ cd DGORL
$ python3 graph_optimization_mrl.py
```
## Setup parameters
You can set following paramters in [script](graph_optimization_mrl.py):
1. number of robots
2. number of iterations
3. workspace dimensions
4. number of max_iterations for optimizer
5. dampting factory for optimizer

## Core contributors

* **Ehsan Latif** - PhD student

* **Dr. Ramviyas Parasuraman** - Principal Investigator


## Heterogeneous Robotics (HeRoLab)

**Heterogeneous Robotics Lab (HeRoLab), Department of Computer Science, University of Georgia.**  

For further information, contact Prof. Ramviyas Parasuraman ramviyas@uga.edu

https://hero.uga.edu/

<p align="center">
<img src="https://herolab.org/wp-content/uploads/2021/04/herolab_newlogo_whitebg.png" width="300">
</p>




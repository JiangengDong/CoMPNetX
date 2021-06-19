# CoMPNetX

This project aims to solved constrained motion planning problem. It mainly depends on two libraries: OMPL and OpenRAVE. The OMPL library provides an framework for sample-based motion planning. The OpenRAVE is used as a simulation environment for rigid body calculation.

## Dependencies

- Ubuntu 16.04
- OpenRAVE: 0.9.0
- OMPL: 1.4.2
- Boost: 1.58
- ROS Kinetic
- ROS package
  - or_urdf
  - baxter_common
  - openrave_catkin
  - srdfdom
  - urdfdom

## Project structure

```
CoMPNetX
├── README.md
├── CMakeLists.txt
├── ... some other files
├── data
│   └── .gitkeep
├── docker
│   └── Dockerfile
├── python
│   ├── main.py
│   ├── train.py
│   ├── test.py
│   └── ... other python scripts
└── src
    ├── CMakeLists.txt
    └── compnetx
        ├── ...C++ headers
        └── src
            ├── lib.cpp
            └── ... C++ source files
```

The structure of this project is as shown above. We depend on OpenRAVE's plugin mechanism to easily interact with files and strings while retaining the ability to plan quickly.

- [python/](python/): This folder contains Python2.7 scripts for training and testing our CoMPNetX algorithm. [python/main.py](python/main.py) is the entrance of this project that routes you to different planners and constraint adherence methods. For its detailed usage, check the [Usage](#Usage) section below.

- [src/compnetx/](src/compnetx/): This folder contains the C++ source code for our planner. It will be compiled into an OpenRAVE's plugin so that it can be loaded into python.

- [docker/Dockerfile](docker/Dockerfile): A docker file which packs all the dependencies for our project. A prebuilt image ([jiangengdong/compnetx](#TODO_fix_this_link)) is also available on docker hub.

- [data/](data/): This is the placeholder for our dataset, which you need to download manually from our [Google Drive](#TODO_fix_this_link). Our dataset has the following structure.

    ```
    data
    ├── environment_setup
    ├── openrave_model
    ├── pytorch_model
    ├── voxel
    │   ├── raw
    │   └── encoded
    └── task_text
        ├── raw
        └── encoded
    ```

## Usage

First of all, you need to make sure all the dependencies are satisfied. You can either [install them on your computer](#Install-on-the-host-computer) or [use a docker container](#Use-docker-container).

Then you can build the C++ part with CMake. A [plugin/](plugin/) folder will be created automatically under the project's root where the generated library lies.

```bash
$ cd CoMPNetX
$ mkdir build && cd build
$ cmake ..
$ make -j
```

Now you can run [python/main.py](python/main.py). It accepts the following command line arguments.
TODO: this part will be updated after the interface is done.

```
--train
--use_grad
--space        ["proj", "atals", "tb"]
--environment  ["bartender", "kitchen"]
```

## How to prepare the dependencies

### Install on the host computer

TODO: fix this part

### Use docker container

TODO: fix this part

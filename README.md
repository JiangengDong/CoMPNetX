# AtlasMPNet

This project aims to solved constrained motion planning problem.
It implements AtlasRRTConnect algorithm. The target file is a plugin for OpenRAVE.

The project mainly depends on two libraries: OMPL and OpenRAVE. The OMPL library provides an framework of sample-based motion planning problem.
The OpenRAVE is used as a simulation environment for rigid body calculation. 

## Dependencies

- OpenRAVE: 0.9.0
- OMPL: 1.4.2
- Boost: 1.58
- ROS Kinetic
- ROS package
  - or_urdf
  - baxter_common
  - openrave_catkin
  - pr_assets
  - srdfdom
  - urdfdom
- CoMPS: using their openrave models

## How to run

We build a docker image that contains all the dependencies and environment variables for this project. Please refer to [JiangengDong/OpenRAVE-Docker](https://github.com/JiangengDong/OpenRAVE-Docker) for the details. Otherwise, you need to check all the dependencies on your own.

### Using a docker image

1. Build this project. The result library is a plugin for OpenRAVE, which is put under `plugins` directory.
    ```
    cd /path/to/project/root
    mkdir build && cd build
    cmake ..
    make -j 4
    ```

1. Set environment variable `OPENRAVE_PLUGINS`. The OpenRAVE loads plugins by searching in directories defined in environment variable `OPENRAVE_PLUGINS`, hence we need
to add our plugin to OPENRAVE's searching path. This environment variable is not set in the image because the project root 
may change, so you still need to set it manually.
    ```
    export OPENRAVE_PLUGINS=${OPENRAVE_PLUGINS}:/path/to/project/root/plugins
    ```

1. Run the scripts under `scripts` with python2 to test this plugin.

### Not using a docker image

1. Build this project. The result library is a plugin for OpenRAVE, which is put under `plugins` directory.
    ```
    cd /path/to/project/root
    mkdir build && cd build
    cmake ..
    make -j 4
    ```

1. Set environment variable `OPENRAVE_PLUGINS`. The OpenRAVE loads plugins by searching in directories defined in environment variable `OPENRAVE_PLUGINS`, hence we need
to add our plugin to OPENRAVE's searching path. 
    ```
    export OPENRAVE_PLUGINS=${OPENRAVE_PLUGINS}:/path/to/project/root/plugins
    ```

1. Similarly, OPENRAVE searches models under the directories specified by `OPENRAVE_DATA`. We use comps's model files.
    ```
    export OPENRAVE_DATA=${DATA}:/path/to/comps/ormodels
    ```

1. This project also depends on a ROS package `or_urdf` that loads URDF to OPENRAVE.
    ``` 
    source /path/to/catkin_ws/devel/setup.bash
    ```

1. After finishing the steps above, you can run the scripts under `scripts` with python2 to test this plugin.

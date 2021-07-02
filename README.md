# CoMPNetX

This repository is the codebase for our CoMPNetX paper, which solves Constrained Motion Planning (CMP) problems with high success rate and low computation time. It is a neural planning approach, comprising a conditional deep neural generator and discriminator with neural gradients-based fast projections to the constraint manifolds. 

Please check [CoMPNetX's webpage](https://sites.google.com/view/compnetx/home) for visual results.

## Dependencies

- [OpenRAVE] : 0.9.0

- [OMPL] : 1.4.2

- [Boost] : 1.58

- [ROS] & ROS package

  - [or_urdf]

  - [openrave_catkin]

  - [srdfdom]

  - [urdfdom]

  - baxter_common

    We modified baxter common, because the "up axis" in OpenRAVE is different from that in Gazebo. The [modified code](docker/catkin_ws/src/baxter_common) is included in this repository.

### Suggestions

We highly recommend you use our pre-built docker image [jiangengdong/compnetx:2.11](https://hub.docker.com/repository/docker/jiangengdong/compnetx). It is the same image we used during development, so it is guaranteed to run smoothly. Check the [usage](#usage) section below for how to use it.

Otherwise, we suggest you use the following version of operating system and ROS if you want to install all the dependencies manually. We found it difficult to build the ROS package or_urdf under Ubuntu 18.04. The [dockerfile](docker/Dockerfile) is a good reference for how we install the dependencies.

- [Ubuntu] : 16.04
- [ROS] : kinetic

## Project layout

```
CoMPNetX/
├── README.md
├── CMakeLists.txt
├── ... # some other files
├── data/
├── experiments/
├── docker/
│   ├── Dockerfile
│   ├── .devcontainer/
│   │   └── devcontainer.json
│   └── catkin_ws/
│       └── ... # necessary packages
├── python/
│   ├── train.py
│   ├── test.py
│   └── ... # other python scripts
└── src/
    ├── CMakeLists.txt
    └── compnetx/
        ├── ... # C++ headers
        └── src/
            ├── lib.cpp
            └── ... # C++ source files
```

The structure of this project is as shown above. We depend on OpenRAVE's plugin mechanism to interact with files and strings effectively while retaining the ability to plan quickly, so there are a Python part and a C++ part in our project. Here is a brief introduction for each folder. 

- [data/](data/): A placeholder for our dataset and 3D models, which you need to download manually from our [Google Drive](#TODO_fix_this_link). A "slim" dataset, which is a subset of the "full" dataset, is also provided [here](#TODO_fix_this_link) for testing purpose only. After downloading and unzipping, it should look like follows. Check [dataset](#dataset) section below for more details.

    ```
    data/
    ├── README.md
    ├── openrave_model/
    └── dataset/
    ```

- [experiments/](experiments/): Another placeholder for our pretrained models. They are released [here](#TODO_fix_this_link). After downloading and unzipping, it should look like follows. 

    ```
    experiments/
    ├── README.md
    ├── exp1/
    └── exp2/
    ```

- [docker/](docker/): This folder contains files for building and running the docker image. 

- [python/](python/): This folder contains Python scripts for training and testing our CoMPNetX algorithm. For its detailed usage, check the [Usage](#Usage) section below.

- [src/compnetx/](src/compnetx/): This folder contains the C++ source code for our planner. It will be compiled into an OpenRAVE's plugin so that it can be loaded into python.

## Dataset



## Usage

### Training

[python/train.py](python/train.py) is the main entrance for training. Apart from some regular arguments like number of epoches, checkpoint interval and output directory, there are some flags that changes the networks' structure. 

- `use_text`: Use the text embedding instead of NTP embedding as the task representation. When this flag is not set, the default choice is NTP embedding. 
  
- `use_reach`: Include reach paths into the expert demonstrations. Two kinds of paths are provided in the dataset: the path from robot's initial pose to the start pose where the robot starts to grab an object is called `reach` path, while the path from the start pose to the goal pose is called `pick_place` path. We train with `pick_place` path exclusively by default.
  
- `use_tsr`: Predict TSR chain's virtual config as well as the robot's config. This flag should be set if you want a good performance, even though we do not enable it by default.

    *Explanation*: Our constraint function is defined as a "handshake" process. A virtual manipulator is constructed based on the TSR chain's parameters. The constraint is satisfied if and only if the virtual manipulator's and the real robot's end effector overlap. Hence, it is as important to predict the virtual config as to predict the real config. (TSR chain is defined in [this paper](https://journals.sagepub.com/doi/abs/10.1177/0278364910396389?casa_token=xuKHXIFQ4aYAAAAA%3AyFdqV1u_0vnvoGhS9ofT3KzCSdCwLAIcx9yJPJxEicFPP5FpG_OwzWQy4O5nxHvkWlVbtuy535FaXJU&))

After training, the output directory will have the following layout. 

```
output/
├── args.yaml      # arguments used for training
├── model_weight/  # checkpoint files
├── tensorboard/   # record of losses during training
├── script/        # backup of the python script
├── embedding/     # embedding of voxel and task representation
└── torchscript/   # torchscripts for neural generator and discriminator
```

The last two folders are the most significant ones. After the training is done, we store outputs of voxel encoder and task encoder under [embedding](embedding) directory, saving our effort during testing. The neural generator and discriminator, which will be frequently used during testing, are converted to torchscript models for later loading into C++. 

### Testing

The first step is to build the C++ code into a plugin with CMake. A [plugins/](plugins/) folder will be created automatically under the project's root where the generated library lies.

```bash
$ cd CoMPNetX
$ mkdir build && cd build
$ cmake ..
$ make -j
```

Then you need to make sure the [plugins/](plugins/) folder is included in the environment variable `OPENRAVE_PLUGINS`. If not, set the environment variable with the following commands. 

```bash
$ cd CoMPNetX
$ export OPENRAVE_PLUGINS=$OPENRAVE_PLUGINS:`pwd`/plugins
```

Now you can run [python/test.py](python/test.py). The most important command line arguments are `work_dir`, `space`, `algorithm` and `use_dnet`. **Note**: this is a Python2.7 script due to the limitation of OpenRAVE.

- `work_dir (-d)`: The output directory during training. Pytorch models, encoded voxels and task representations are loaded from this directory. Since the training arguments are saved as a `args.yaml` file under the work directory, they are also load automatically.
- `space (-s)`: There are three kinds of constraint-adherence method: Projection, Atlas and Tangent Bundle, and CoMPNetX can work with any of them. The default option is `atlas`.
- `algorithm (-a)`: Test with classical algorithm or neural-based ones. Since we can't put CoMPNet and CoMPNetX result in the same output directory, they share the same option as `CoMPNetX`. The test script will determine which algorithm to use based on the content of `args.yaml`.
- `use_dnet (-p)`: Whe you are testing CoMPNet or CoMPNetX, you can choose to opt-in the neural projector. 

The other command line arguments are listed below.

```bash
$ python2 python/test.py --help
usage: test.py [-h] [-l {0,1,2,3,4}] [-v] [-s {proj,atlas,tb}] -d WORK_DIR
               [-a {compnetx,rrtconnect}] [-p]

A all-in-one script to test RRTConnect, CoMPNet and CoMPNetX algorithm.

optional arguments:
  -h, --help            show this help message and exit
  -l {0,1,2,3,4}, --log_level {0,1,2,3,4}
                        Set log level. Lower level generates more logs.
  -v, --visible         Show a 3D visualization of the planning.
  -d WORK_DIR, --work_dir WORK_DIR
                        Output directory during training. Setting, data and
                        models will be read from this directory automatically.
  -s {proj,atlas,tb}, --space {proj,atlas,tb}
                        Constraint-adherence method to use.
  -a {compnetx,rrtconnect}, --algorithm {compnetx,rrtconnect}
                        Select an algorithm. Choose `compnetx` for both
                        CoMPNet and CoMPNetX, and the exact algorithm will be
                        selected according to the settings in the work
                        directory.
  -p, --use_dnet        Use neural projector.
```

### Docker image

We provide a pre-built docker image [jiangengdong/compnetx:2.11](https://hub.docker.com/repository/docker/jiangengdong/compnetx) on docker hub, which can satisfy your testing requirement most of the time. If you want to build the docker image by yourself, the [dockerfile](docker/Dockerfile) and [modified baxter_common package](docker/catkin_ws/) are included in this repository for your reference.

We also provide two ways for your to run the docker container. **Note**: both of them require [CUDA] >10.2 and [nvidia-container] installed on your host machine.

- [VSCode Remote - Container](https://code.visualstudio.com/docs/remote/containers): This is the recommended method. This way you can mount the workspace into the container and take advantage of the toolchain installed inside. We provide a configuration file [devcontainer.json](docker/.devcontainer/devcontainer.json) that can help you start quickly.
  
- Manually start: Use the following command only if you are accustomed to CLI and vim. The first three lines mounts current directory to the container so that we can run the scripts. Line 4-7 forwards the X11 unix socket into the container to enable visualization. Line 8 allows the container to use all the gpus on the host machine, so CUDA is available inside the container. 
    
    ```bash
    docker run --mount type=bind,source=`pwd`,target=/workspaces/CoMPNetX \
               -w /workspaces/CoMPNetX -i -t \
               -e OPENRAVE_PLUGINS=/workspaces/CoMPNetX/plugins \
               --mount type=bind,source=/tmp/.X11-unix,target=/tmp/.X11-unix \
               --mount type=bind,source=${XAUTHORITY},target=/root/.Xauthority \
               -e DISPLAY \
               -e QT_X11_NO_MITSHM=1 \
               --gpus all \
               jiangengdong/compnetx:2.11
    ```

## Acknowledge

The [TSRChain](src/compnetx/TaskSpaceRegionChain.h) code is adopted from the Constrained Manipulation Planning Suite ([CoMPS]). 

The definition of the [constraint function](src/compnetx/Constraint.h) is inspired by [CuikSuite], but we use quaternion instead of matrix for rotation representation. 


[CoMPS]: https://sourceforge.net/projects/comps/
[CuikSuite]: https://www.iri.upc.edu/research/webprojects/cuikweb/CuikSuite3-Doc/html/index.html
[OpenRAVE]: http://openrave.org/
[OMPL]: https://ompl.kavrakilab.org/
[Boost]: https://www.boost.org/doc/libs/1_58_0/
[ROS]: https://www.ros.org/
[Ubuntu]: https://ubuntu.com/
[or_urdf]: https://github.com/personalrobotics/or_urdf
[urdfdom]: https://github.com/ros/urdfdom
[srdfdom]: https://github.com/ros-planning/srdfdom
[openrave_catkin]: https://github.com/personalrobotics/openrave_catkin
[CUDA]: https://developer.nvidia.com/cuda-toolkit
[nvidia-container]: https://github.com/NVIDIA/nvidia-docker
# CoMPNetX

This repository is the codebase for our CoMPNetX paper, which solves Constrained Motion Planning (CMP) problems with high success rate and low computation time. It is a neural planning approach, comprising a conditional deep neural generator and discriminator with neural gradients-based fast projections to the constraint manifolds.

Please check [CoMPNetX's webpage](https://sites.google.com/view/compnetx/home) for visual results.

## Contents

- [CoMPNetX](#compnetx)
  - [Contents](#contents)
  - [Dependencies](#dependencies)
    - [Suggestions](#suggestions)
  - [Project layout](#project-layout)
  - [Data](#data)
    - [Experiment introduction](#experiment-introduction)
    - [OpenRAVE models](#openrave-models)
    - [Dataset](#dataset)
  - [Usage](#usage)
    - [Clone the repository](#clone-the-repository)
    - [Training](#training)
    - [Testing](#testing)
    - [Docker image](#docker-image)
  - [Pretrained models](#pretrained-models)
    - [Model list](#model-list)
    - [Test result for bartender (exp1 and exp2)](#test-result-for-bartender-exp1-and-exp2)
      - [Accuracy](#accuracy)
      - [Average time](#average-time)
    - [Test result for kitchen (exp3 and exp4)](#test-result-for-kitchen-exp3-and-exp4)
      - [Accuracy](#accuracy-1)
      - [Average time](#average-time-1)
  - [Acknowledge](#acknowledge)

## Dependencies

- [OpenRAVE] : 0.9.0
- [OMPL] : 1.4.2
- [Boost] : 1.58
- [ROS] & ROS package
  - [or_urdf]
  - [openrave_catkin]
  - [srdfdom]
  - [urdfdom]
  - baxter_common:
    We modified baxter common, because the "up axis" in OpenRAVE is different from that in Gazebo. The [modified code](docker/catkin_ws/src/baxter_common) is included in this repository.

### Suggestions

We highly recommend you use our pre-built docker image [jiangengdong/compnetx:2.11](https://hub.docker.com/repository/docker/jiangengdong/compnetx). It is the same image we used during development. Check the [Usage](#docker-image) section below for how to use it.

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
├── extern/
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

- [data/](data/): A placeholder for our dataset and 3D models, which you need to download manually from our [Google Drive](https://drive.google.com/file/d/1FSd7OC6zEzQMmf_RMuOsEB8DS1tEjpFj/view?usp=sharing). A "slim" dataset, which is a subset of the "full" dataset, is also provided [here](https://drive.google.com/file/d/1W_cMgrXvx-Lin3vRUAiBgCZw8SP8qrAA/view?usp=sharing) for testing purpose only. After downloading and unzipping, it should look like follows. Check the [Data](#data) section below for more details.

    ```
    data/
    ├── openrave_model/
    └── dataset/
    ```

- [experiments/](experiments/): Another placeholder for our pretrained models. They are released [here](https://drive.google.com/file/d/1yJcqL8WpYU_R9aRGbxtm4ReHlXTeoR8O/view?usp=sharing). After downloading and unzipping, it should look like follows. Check the [Pretrained Models](#pretrained-models) section below for more details.

    ```
    experiments/
    ├── exp1/
    ├── exp2/
    ├── exp3/
    └── exp4/
    ```
- [extern/](extern/): Packages used by our C++ code. These packages are included in this repository as [git submodule](https://git-scm.com/book/en/v2/Git-Tools-Submodules), so remember to pull them with the following commands.

    ```bash
    # you can setup the submodules when you clone the repository
    git clone --recursive --branch v1.1 https://github.com/JiangengDong/CoMPNetX.git
    # or you can init and update submodules later
    git submodule init
    git submodule update
    ```

- [docker/](docker/): This folder contains files for building and running the docker image.

- [python/](python/): This folder contains Python scripts for training and testing our CoMPNetX algorithm. For its detailed usage, check the [Usage](#Usage) section below.

- [src/compnetx/](src/compnetx/): This folder contains the C++ source code for our planner. It will be compiled into an OpenRAVE's plugin so that it can be loaded into python.

## Data

### Experiment introduction

Our algorithm is tested in two environments, bartender and kitchen, each having a number of difference scenes for training and testing.

|    env    | train | test  |
| :-------: | :---: | :---: |
| bartender | 1727  |  106  |
|  kitchen  | 1525  |  138  |

- Bartender: There are 5 movable objects in this environment: a fuze bottle, juice can, soda can, kettle, and plastic mug. The first three objects can be moved freely without orientation constraints, while the others should be kept upright to avoid spilling out. The robot bartender is asked to place the bottle and cans to a trash bin and the others onto the tray.

- Kitchen: In this scenario we have 7 manipulatable objects: soda can, juice can, fuze bottle, cabinet door, black mug, red mug, and pitcher. The objective is to move the cans and bottle to the trash bin without orientation constraints, open the cabinet door from any starting angle to a fixed final angle (π/2.7), transfer (without tilting) the black and red mugs from the cabinet to the tray, and move the pitcher (without tilting) from the table into the cabinet.

For each object, the robot needs to plan two paths: one from the robot's initial pose to the object's start pose ("reach"), and the other from the object's start pose to the goal pose ("pick & place"). We focus on the "pick & place" path in this project because the reach path is unconstrained and any off the shelf planner (e.g., MPNet, RRT*) can be used for that task.

### OpenRAVE models

Models for the 9 manipulatable objects and 2 fixed objects (trash bin and tray) are included in [data/openrave_model/](data/openrave_model/). These models are copied and modified from OpenRAVE's builtin models and [CoMPS]'s codebase.

### Dataset

Before we start, let's first take a look at some placeholders that will be used in this section.

- `env_id`: Either "bartender" or "kitchen".
- `dataset`: "setup", "voxel", "text_embedding", "ntp_embedding", "path", or "tsr_path".
- `scene_id`: A string that represent a unique scene. Valid `scene_id`s are store in [data/dataset/description.yaml](data/dataset/description.yaml). This YAML has the follow structure.
    ```yaml
    bartender:
      test:
      - {scene_id_1}
      - {scene_id_2}
      - ...
      train:
      - ...
    kitchen:
      test:
      - ...
      train:
      - ...
    ```
- `obj_name`: Objects in the scene. For the bartender environment, manipulatable objects can be one of `["fuze_bottle", "juice", "coke_can", "teakettle", "plasticmug"]`. For the kitchen environment, they can be `["fuze_bottle", "juice", "coke_can", "door", "mugred", "mugblack", "pitcher"]`.

Now we can take a look at the datasets. All the datasets are store with the name `{env_id}_{dataset}.hdf5` as HDF5 files, and most of them are organized in the structure `[/prefix]/{scene_id}/{obj_name}[/suffix]`, where the prefix and suffix are optional.

- **`setup` dataset**

    This dataset contains the information on how to setup the environments.

    ```
    /
    ├── {scene_id}/
    │   ├── obj_order
    │   ├── {fixed obj_name}/
    │   │   └── start_trans    # Robot's initial config.
    │   ├── {movable obj_name}/
    │   │   ├── initial_config # Robot's initial config.
    │   │   ├── start_trans    # The object's start transform in the work space.
    │   │   ├── start_config   # Robot's config when grabing the object at the start point.
    │   │   ├── goal_trans     # Opposed to `start_trans`. Also T0_w of the TSR.
    │   │   ├── goal_config    # Opposed to `start_config`.
    │   │   ├── tsr_bound      # Bw of the TSR.
    │   │   └── tsr_offset     # Tw_e of the TSR.
    │   ├── door/
    │   │   ├── initial_config # Robot's initial config.
    │   │   ├── start_trans    # The start joint value of door hinge
    │   │   ├── start_config   # Robot's config when grabing the handle at the start point.
    │   │   ├── goal_trans     # Opposed to `start_trans`.
    │   │   ├── goal_config    # Opposed to `start_config`.
    │   │   ├── tsr_base       # T0_w of the TSR chain.
    │   │   ├── tsr_bound0     # Bw of the first TSR in the TSR chain.
    │   │   ├── tsr_offset0    # Tw_e of the first TSR in the TSR chain.
    │   │   ├── tsr_bound1     # Bw the second TSR in the TSR chain.
    │   │   └── tsr_offset1    # Tw_e of the second TSR in the TSR chain.
    │   └── ... # the other objects
    └── ... # the other scenes
    ```

    Objects in the scene are divided into three categories.

    - The first category is fixed objects, i.e. the `recyclingbin` and the `tray`. These objects' transforms are fixed for each scene, so they have only one field `start_trans`.

    - The second category is movable objects. It includes all the manipulatable objects except `door`. Their fields are shown above with comments.

    - The third category has only one object, the `door`. This is a special object, as it is not movable. Its `start_trans` is actually the joint value of the door's hinge, but we use the same name as the other objects for easier programming. Besides, its TSR chain is defined with two TSRs, unlike the other objects which have only one TSR.

    One special part in the `setup` dataset is the `obj_order`, which is an array of string that defines the order for moving all the manipulatable object. Each scene has its own `obj_order`.

- **`voxel` dataset**

    This dataset contains the raw voxels of the environment. This is the robot's perception of its surroundings before moving each object. It has the simplest layout among all the datasets.

    ```
    /
    ├── {scene_id}/
    │   ├── {obj_name}
    │   └── ... # the other objects
    └── ... # the other scenes
    ```

- **`ntp_embedding` and `text_embedding` dataset**

    These two datasets are the NTP and text-based representation of the tasks. They share the same layout, and the difference is that NTP embeddings are 270-d vectors, while text embeddings are 4096-d vectors.

    ```
    /
    ├── reach/
    │   ├── {scene_id}/
    │   │   ├── {obj_name}
    │   │   └── ... # the other objects
    │   └── ... # the other scenes
    └── pick_place/
        └── ... # the same layout as `reach`
    ```

    As we already discussed in the [experiment introduction](#experiment-introduction) above, the robot need to plan two paths for each object. Hence, the task representations are divided into two groups, `reach` and `pick_place`.

- **`path` dataset**

    This dataset contains the expert demonstration paths genarated by [CoMPS]. It has the same layout as the `embedding`s dataset

    ```
    /
    ├── reach/
    │   ├── {scene_id}/
    │   │   ├── {obj_name}
    │   │   └── ... # the other objects
    │   └── ... # the other scenes
    └── pick_place/
        └── ... # the same layout as `reach`
    ```

- **`tsr_path` dataset**

    This dataset contains the configs of the virtual TSR chain manipulator corresponding to points in the `path` dataset.

    Our constraint function is defined as a "handshake" process. A virtual manipulator is constructed based on the TSR chain's parameters. The constraint is satisfied if and only if the virtual manipulator's and the real robot's end effector overlap. Hence, it is as important to predict the virtual config as to predict the real config. (TSR chain is defined in [this paper](https://journals.sagepub.com/doi/abs/10.1177/0278364910396389?casa_token=xuKHXIFQ4aYAAAAA%3AyFdqV1u_0vnvoGhS9ofT3KzCSdCwLAIcx9yJPJxEicFPP5FpG_OwzWQy4O5nxHvkWlVbtuy535FaXJU&))

    ```
    /
    ├── reach/
    │   ├── config/
    │   │   ├── {scene_id}/
    │   │   │   ├── {obj_name}
    │   │   │   └── ... # the other objects
    │   │   └── ... # the other scenes
    │   └── distance/
    │       └── ... # the same layout as `config`
    └── pick_place/
        └── ... # the same layout as `reach`
    ```

    `TSR config`, `TSR distance` and `path` for the same scene and object always have the same length along axis 0.

All the datasets above are included in the [full](https://drive.google.com/file/d/1FSd7OC6zEzQMmf_RMuOsEB8DS1tEjpFj/view?usp=sharing) package. The [slim](https://drive.google.com/file/d/1W_cMgrXvx-Lin3vRUAiBgCZw8SP8qrAA/view?usp=sharing) package has only `setup` dataset which is used in testing.

## Usage

### Clone the repository

We use [git submodule](https://git-scm.com/book/en/v2/Git-Tools-Submodules) to organize some dependencies, so it would be better to use recursive clone.

```bash
git clone --recursive --branch v1.1 https://github.com/JiangengDong/CoMPNetX.git
```

If you forget the `--recursive` argument during cloning, you can still update the submodules with the follow commands.

```bash
git submodule init
git submodule update
```

### Training

[python/train.py](python/train.py) is the main entrance for training. Apart from some regular arguments like number of epoches, checkpoint interval and output directory, there are some flags that changes the networks' structure.

- `use_text`: Use the text embedding instead of NTP embedding as the task representation. When this flag is not set, the default choice is NTP embedding.

- `use_reach`: Include reach paths into the expert demonstrations. Two kinds of paths are provided in the dataset: the path from robot's initial pose to the start pose where the robot starts to grab an object is called `reach` path, while the path from the start pose to the goal pose is called `pick_place` path. We train with `pick_place` path exclusively by default.

- `use_tsr`: Predict TSR chain's virtual config as well as the robot's config. This flag should be set if you want a good performance, even though we do not enable it by default.

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

## Pretrained models

**Note**: These models are trained and tested with only pick and place task.
### Model list

| folder |    env    | algorithm | task representation | has neural discriminator? |
| :----: | :-------: | :-------: | :-----------------: | :-----------------------: |
|  exp1  | bartender |  CoMPNet  |   text embedding    |            No             |
|  exp2  | bartender | CoMPNetX  |    NTP embedding    |            Yes            |
|  exp3  |  kitchen  |  CoMPNet  |   text embedding    |            No             |
|  exp4  |  kitchen  | CoMPNetX  |    NTP embedding    |            Yes            |

### Test result for bartender (exp1 and exp2)

#### Accuracy

| Space | RRTConnect | CoMPNet | CoMPNetX (w/o proj) | CoMPNetX |
| :---: | :--------: | :-----: | :-----------------: | :------: |
| atlas |   97.17%   |  100%   |        100%         |   100%   |
|  tb   |   91.89%   |  100%   |        100%         |  99.81%  |
| proj  |   97.92%   | 99.43%  |       98.30%        |  99.81%  |

#### Average time

| Space | RRTConnect | CoMPNet | CoMPNetX (w/o proj) | CoMPNetX |
| :---: | :--------: | :-----: | :-----------------: | :------: |
| atlas |   11.70    |  9.29   |        7.73         |   6.72   |
|  tb   |   15.10    |  12.31  |        11.03        |  10.52   |
| proj  |   20.52    |  12.34  |        9.91         |   7.68   |

### Test result for kitchen (exp3 and exp4)

#### Accuracy

| Space | RRTConnect | CoMPNet | CoMPNetX (w/o proj) | CoMPNetX |
| :---: | :--------: | :-----: | :-----------------: | :------: |
| atlas |   94.78%   |  100%   |        100%         |   100%   |
|  tb   |   94.78%   | 99.71%  |       99.71%        |  99.27%  |
| proj  |   96.37%   | 99.71%  |       99.56%        |  99.71%  |

#### Average time

| Space | RRTConnect | CoMPNet | CoMPNetX (w/o proj) | CoMPNetX |
| :---: | :--------: | :-----: | :-----------------: | :------: |
| atlas |   44.30    |  26.40  |        19.43        |  21.93   |
|  tb   |   48.00    |  40.30  |        35.64        |  34.36   |
| proj  |   72.72    |  44.51  |        32.09        |  30.91   |


## Bibliography
```
@article{qureshi2021constrained,
  title={Constrained Motion Planning Networks X},
  author={Qureshi, Ahmed H and Dong, Jiangeng and Baig, Asfiya and Yip, Michael C},
  journal={IEEE Transactions on Robotics},
  year={2021}
}
@ARTICLE{qureshi2020compnet,
  author={A. H. {Qureshi} and J. {Dong} and A. {Choe} and M. C. {Yip}},
  journal={IEEE Robotics and Automation Letters},
  title={Neural Manipulation Planning on Constraint Manifolds},
  year={2020},
  volume={5},
  number={4},
  pages={6089-6096}
}
```

## Acknowledgements

The [TSRChain](src/compnetx/TaskSpaceRegionChain.h) code is adopted from the Constrained Manipulation Planning Suite ([CoMPS]). Many OpenRAVE models are also copied and modified from CoMPS's codebase.

The definition of the [constraint function](src/compnetx/Constraint.h) is inspired by [CuikSuite], but we use quaternion instead of matrix for rotation representation.

We also use [HighFive] to read HDF5 in C++.


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
[HighFive]: https://bluebrain.github.io/HighFive/

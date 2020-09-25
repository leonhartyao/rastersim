# flypulator_uav_omni
This repository holds ROS packages special for the over-actuated UAV with variably tilted rotors.

[![License](https://img.shields.io/badge/license-GPLv3-blue)](https://opensource.org/licenses/GPL-3.0)

## Download and Build
The content of this repository must be cloned into the `/src` folder of a catkin workspace ([how to create an empty workspace](http://wiki.ros.org/catkin/Tutorials/create_a_workspace)). For an existing workspace in `~/catkin_ws` that requires the following steps (in a new terminal):

```sh
$ cd ~/your_ws/src/
$ git clone git@gitlab.com:FLYing-maniPULATOR/flypulator_uav_omni.git
$ cd ..
$ catkin_make
```
You can specify a desired branch to clone:
```sh
$ git clone -b <branch> git@gitlab.com:FLYing-maniPULATOR/flypulator_uav_omni.git
```
or checkout to the desired branch after default clone of master branch:
```sh
$ git checkout <branch>
```
If some ROS packages are missing, just install them with
```sh
$ sudo apt install ros-[your_version]-[package name]
```

Dont forget sourcing the setup.bash file:
```sh
$ source ~/your_ws/devel/setup.bash
```
or if you using Zsh:
```sh
$ source ~/your_ws/devel/setup.zsh
```
To do sourcing permanently, edit the .bashrc file with `gedit ~/.bashrc` and add the source command from above (`source ~/your_ws/devel/setup.bash`). *Note that you have to start a new terminal to apply the changes*. You can check if the packages are available using:
```sh
$ rospack list
```
## Usage
Refer to the readme.md file in corresponding packages for details.

## Tipps
 - When upgrading ROS version, clean your workspace folder (delete `devel` and `build`) and **remove related `source ~/your_ws/devel/setup.bash`**, since it still points to old version and causes errors.
 - If some header files are missing while compiling, which should be generated during the compile process, the dependency is not well defind in the CMakeLists.txt file to force the compile order. Declare the dependency in the file or build again for a quick fix (may still fail).

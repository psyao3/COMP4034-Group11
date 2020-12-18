# COMP4034-Group11
Group 11


We are unable to submit the entirety of our darknet_ros package, due to size limitations.
Please follow the guide below, to setup darknet_ros on your personal machine.

***CUDA RECOMMENDED*** for adequate performance, CPU performance is sub-optimal. (0.2 FPS vs 40 FPS)

Once everything installed, please run:
	
	roslaunch assignment assignment_launch.launch (Modified to include the launch file for darknet)
	roslaunch assignment navigation.launch (Modified to launch gmapping)
	rosrun assignment main.py
	
########################################################################################################

Final ~/catkin_workspace should be:

~/catkin_workspace/src/darknet_ros
    -/darknet                                      from: git@github.com:AlexeyAB/darknet.git
        -Makefile   --replaced with modified ver.  
    -/darknet_ros                                  from: original git repo git@github.com:tom13133/darknet_ros.git
        -CMakeLists.txt   --edited
        -/config
            -yolov4-tiny.yaml   --added
        -/yolo_network_config                     from:https://github.com/psyao3/COMP4034-Group11/src/yolo_network_config
    -/darknet_ros_msgs                             from: original git repo git@github.com:tom13133/darknet_ros.git
    -
    -
    -misc files

########################################################################################################

Step 2a is optional if you have a CUDA enabled GPU

Steps 2b & 2c (specific to our Yolov4 Model), can be done after step 3.


########################################################################################################

STEP 1 ----------------------------------------------------
do: 

git clone --recursive git@github.com:tom13133/darknet_ros.git

########################################################################################################

STEP 2 ---------------------------------------------------- #FOR USE WITH CPU
do the following:

from "~/catkin_workspace/src/darknet_ros/" 
delete the directory "/darknet" and git clone git@github.com:AlexeyAB/darknet.git

from "~/catkin_workspace/src/darknet_ros/darknet/Makefile"
edit lines 4,5 and 6
=0 to =1
OPENCV=1
AVX=1                if an error occurs set AVX=0
OPENMP=1

from "~/catkin_workspace/src/darknet_ros/darknet_ros/CMakeLists.txt"
comment out line 215 "${DARKNET_PATH}/src/yolo_v2_class.cpp         ${DARKNET_PATH}/src/yolo_console_dll.cpp"
(thanks Alex)

you may need to remove line 25 "-gencode arch=compute_30,code=sm_30" if you get the error: 
"nvcc -fatal: unsupported gpu architecture -gencode arch=compute_30"

STEP 2a ---------------------------------------------------- #FOR USE WITH GPU
install the appropriate Cuda-Toolkit and cuDNN for your system. 
Refer to https://docs.nvidia.com/cuda/cuda-installation-guide-linux/index.html
&
https://docs.nvidia.com/deeplearning/cudnn/install-guide/index.html#installlinux

from "~/catkin_workspace/src/darknet_ros/darknet/Makefile"
edit lines 1,2,3 and 4
=0 to =1
GPU=1
CUDNN=1
CUDNN_HALF=1
OPENCV=1

## IF HAVING ISSUES:
Use these commands to do a clean uninstall of your nvidia drivers and CUDA. You will need to download and install the appropriate nvidia drivers again 
for your GPU before following CUDA/CUDNN installation instructions. (Personally, after this clean uninstall, installing CUDA via debian network and 
CUDNN via the tgz file worked for me - Alex) 

To remove CUDA Toolkit:
$ sudo apt-get --purge remove "*cublas*" "*cufft*" "*curand*" \
 "*cusolver*" "*cusparse*" "*npp*" "*nvjpeg*" "cuda*" "nsight*" 

To remove NVIDIA Drivers:
$ sudo apt-get --purge remove "*nvidia*"

To clean up the uninstall:
$ sudo apt-get autoremove

ALSO: make sure to delete your build folder in your catkin workspace. For some reason mine wouldn't find the right CUDA version until I had.

########################################################################################################

STEP 2b ----------------------------------------------------
do:

from "~/catkin_workspace/src/darknet_ros/darknet_ros/config"
add "yolov4-tiny.yaml" from https://github.com/psyao3/COMP4034-Group11/src/yolov4-tiny.yaml

########################################################################################################

STEP 2c ----------------------------------------------------
do:

from "~/catkin_workspace/darknet_ros/darknet_ros/yolo_network_config"
replace with yolo_network_config from:https://github.com/psyao3/COMP4034-Group11/src/yolo_network_config

########################################################################################################

STEP 3 ----------------------------------------------------
do: 

catkin_make -DCMAKE_BUILD_TYPE=Release






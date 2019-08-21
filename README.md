# jetbot_ros
ROS nodes and Gazebo model for NVIDIA JetBot with Jetson Nano

- This repository is based on the following repository.
  - https://github.com/dusty-nv/jetbot_ros

## System Configuration

It is assumed that the Nano has been setup with JetPack 4.2 and that CUDA, cuDNN, and TensorRT have been installed.

> **Note**:  the process below will likely exceed the disk capacity of the default 16GB filesystem,
> &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; so a larger SD card should be used.  If using the 'Etcher' method with JetPack-L4T image,
> &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; the APP partition will automatically be resized to fill the SD card upon first booting the system.
> &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; Otherwise flash with L4T using the -S option (example given for 64GB SD card):
> &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; `sudo ./flash.sh -S 58GiB jetson-nano-sd mmcblk0p1`

# jetbot basic setup
## GW-450D2のドライバセットアップ
- 参考：https://kitto-yakudatsu.com/archives/6003#5GHzLAN_USB
- 5G帯での通信を利用したい場合はこれを使うと良い．ただし，OLEDに表示されていたIPアドレス等が表示されなくなる弊害あり．

```
cd /usr/local/src
sudo git clone https://github.com/xtknight/mt7610u-linksys-ae6000-wifi-fixes.git
cd mt7610u-linksys-ae6000-wifi-fixes
sudo make
sudo make install
```
- このあと再起動して接続設定
- パフォーマンスはBuffaloの2.4G帯だけのものなどと比べると良さそう

# jetbot software setup
- 参考
  - https://github.com/NVIDIA-AI-IOT/jetbot/wiki/software-setup
## clone jetbot repository
```
git clone https://github.com/NVIDIA-AI-IOT/jetbot
cd jetbot
sudo python3 setup.py install
```
- Replace the old notebooks with the new notebooks by entering
```
sudo apt-get install rsync
cd
rsync jetbot/notebooks ~/Notebooks
```


# ROS Setup for jetbot
- 参考URL
  - GPG Error発生時の対処
  - http://pyopyopyo.hatenablog.com/entry/20180514/p1
  - https://github.com/balena-io/etcher/issues/1786

## Install ros-melodic-ros-base
```
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver keyserver.ubuntu.com --recv-key F42ED6FBAB17C654
sudo apt-get update
sudo apt-get install ros-melodic-ros-base
sudo sh -c 'echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc'
```
- Close and restart the terminal.

## Install Adafruit Libraries
- These Python libraries from Adafruit support the TB6612/PCA9685 motor drivers and the SSD1306 debug OLED:

```bash
sudo apt-get install python-pip
pip install Adafruit-MotorHAT
pip install Adafruit-SSD1306
```
- Grant your user access to the i2c bus:
```bash
sudo usermod -aG i2c $USER
sudo shutdown -r now
```
- Reboot the system for the changes to take effect.

## create the catkin workspace
- Create a ROS Catkin workspace to contain our ROS packages:

```
mkdir -p ~/workspace/catkin_ws/src
cd ~/workspace/catkin_ws
catkin_make
```

- add catkin_ws path to bashrc
```
sudo sh -c 'echo "source ~/workspace/catkin_ws/devel/setup.bash" >> ~/.bashrc'
```

```
> Note:  out of personal preference, my catkin_ws is created as a subdirectory under ~/workspace

- Close and open a new terminal window.
- Verify that your catkin_ws is visible to ROS:
```bash
$ echo $ROS_PACKAGE_PATH
/home/nvidia/workspace/catkin_ws/src:/opt/ros/melodic/share
```

# Build jetson-inference
## clone the repo and submodules
```
cd ~/workspace
git clone -b onnx https://github.com/dusty-nv/jetson-inference
cd jetson-inference
git submodule update --init
sudo apt-get install libglew-dev libqt4-dev qt4-qmake
```

## build from source
```
mkdir build
cd build
cmake ../
make
sudo make install
```


# ROS development sample
## Writing a Simple Publisher and Subscriber (Python)

```
cd ~/workspace/catkin_ws/src
catkin_create_pkg beginner_tutorials std_msgs rospy
cd ~/catkin_ws
catkin_make
```

- http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28python%29
```
source ~/workspace/catkin_ws/devel/setup.bash
roscd beginner_tutorials
mkdir scripts
cd scripts
```

- Download https://raw.github.com/ros/ros_tutorials/kinetic-devel/rospy_tutorials/001_talker_listener/talker.py into ~/workspace/catkin_ws/src/beginner_tutorials/scripts
- Download https://raw.github.com/ros/ros_tutorials/kinetic-devel/rospy_tutorials/001_talker_listener/listener.py into ~/workspace/catkin_ws/src/beginner_tutorials/scripts
- After downloading the talker.py and listener.py, execute chmod command for each file.
```
chmod +x talker.py
chmod +x listener.py
```
- Build executables by 'catkin_make'.
```
$ cd ~/workspace/catkin_ws
$ catkin_make
```

- Examining the Simple Publisher and Subscriber(Python)
  - cf. http://wiki.ros.org/ROS/Tutorials/ExaminingPublisherSubscriber
- You have to launch some terminals.
- Launch roscore.
```
$ roscore
```
- Open new terminal and launch talker.
```
cd ~/workspace/catkin_ws
rosrun beginner_tutorials talker.py
```
- Open new terminal and launch listener.
```
cd ~/workspace/catkin_ws
rosrun beginner_tutorials listener.py
```
- You can terminate the programs by Ctrl + C key.


# Build ros_deep_learning
## Clone and build the ros_deep_learning repo:
- install dependencies
```
sudo apt-get install ros-melodic-vision-msgs ros-melodic-image-transport ros-melodic-image-publisher
cd ~/workspace/catkin_ws/src
git clone https://github.com/dusty-nv/ros_deep_learning
```

## make ros_deep_learning
```
cd ../    # cd ~/workspace/catkin_ws
catkin_make
```

## confirm that the package can be found
```
$ rospack find ros_deep_learning
/home/nvidia/workspace/catkin_ws/src/ros_deep_learning
```

# Build jetbot_ros
## Clone and build the jetbot_ros repo:
```
cd ~/workspace/catkin_ws/src
git clone https://github.com/dusty-nv/jetbot_ros
```
## build the package
```
cd ../    # cd ~/workspace/catkin_ws
catkin_make
```

## confirm that jetbot_ros package can be found
```
$ rospack find jetbot_ros
/home/nvidia/workspace/catkin_ws/src/jetbot_ros
```

# Testing JetBot
## Running the Motors
- jetbot_rosの動作テスト
- まず新しいターミナルを開いてroscoreを実行する

```
$ roscore
```
- jetbot_motors.pyを下記にする
  - https://github.com/igakilab/jetbot_ros/blob/3cac34af82be9a5212359c49aa84483a89d4035c/scripts/jetbot_motors.py
- Open a new terminal, and start the jetbot_motors node:
```
rosrun jetbot_ros jetbot_motors.py
```
- The jetbot_motors node will listen on the following topics:
```
/jetbot_motors/cmd_dir relative heading (degree [-180.0, 180.0], speed [-1.0, 1.0])
/jetbot_motors/cmd_raw raw L/R motor commands (speed [-1.0, 1.0], speed [-1.0, 1.0])
/jetbot_motors/cmd_str simple string commands (left/right/forward/backward/stop)
Note: as of 2/22/19, only cmd_str method is implemented. Other methods coming soon.
```

## Test Motor Commands
- Open a new terminal, and run some test commands:
```
$ rostopic pub /jetbot_motors/cmd_str std_msgs/String --once "forward"
$ rostopic pub /jetbot_motors/cmd_str std_msgs/String --once "backward"
$ rostopic pub /jetbot_motors/cmd_str std_msgs/String --once "left"
$ rostopic pub /jetbot_motors/cmd_str std_msgs/String --once "right"
$ rostopic pub /jetbot_motors/cmd_str std_msgs/String --once "stop"
```
- it is recommended to initially test with JetBot up on blocks, wheels not touching the ground

## Create SimpleJetbot
- キー入力で動かせるようなプログラムの作成
- 準備
```
$ pip install py-getch
```
- jetbot_motors.pyを下記にする
  - https://github.com/igakilab/jetbot_ros/blob/34116b438595bfbba3fbcc814e3dbeaee441048a/scripts/jetbot_motors.py
- Open a new terminal, and start the jetbot_motors node:
  - 別のターミナルでroscoreを動かしておくこと
```
rosrun jetbot_ros jetbot_motors.py
```
- sを押すと前進，qを押すと停止，Ctr+Cを押すとプログラム終了

# Using the Debug OLED
- If you have an SSD1306 debug OLED on your JetBot, you can run the jetbot_oled node to display system information and user-defined text:
```
$ rosrun jetbot_ros jetbot_oled.py
```
- By default, jetbot_oled will refresh the display every second with the latest memory usage, disk space, and IP addresses.
- The node will also listen on the /jetbot_oled/user_text topic to recieve string messages from the user that it will display:

```
rostopic pub /jetbot_oled/user_text std_msgs/String --once "HELLO!"
```

# Using the Camera
- To begin streaming the JetBot camera, start the jetbot_camera node:

```
$ rosrun jetbot_ros jetbot_camera
```

# Ros program development on Jupyter(失敗中)
- Jupyterのセルに↓を記述
```
import os
import sys
sys.path.append('/opt/ros/melodic/lib/python2.7/dist-packages')
import pprint
pprint.pprint(sys.path)
```
- rospkgがないといわれたので，インストール
```
pip install rospkg
```
- これだけでもNoModuleと言われたので，`pip show rospkg` でLocationを確認．
  - `sys.path.append('/home/jetbot/.local/lib/python2.7/site-packages')` 今回はこれを追加したらいけた．pythonpathとかの環境変数を使う方法に将来は対応した．

- セルに以下を追記してみた．rosで必要なsetupスクリプトの呼び出し(.bashrcに書いてあるやつ)
  - ``!. **setup.sh``
  - これでもrosgraphはResourceが無いと言われるので，jupyterのセルでやるのはちょっと断念．

# pepper_demo

## Instructions

### Installation
1. Clone this repository into a catkin_ws and use catkin_make to build the package.
2. You will need to install the package here as well: https://github.com/firephinx/openpose_ros

### Running the Actual Demo
1. Ssh to pepper in a new terminal and start roscore on Pepper.
	```
	ssh nao@128.237.247.249
	./startros
	```
2. Create a new terminal and start the speech node on Pepper.
	```
	ssh nao@128.237.247.249
	source gspeech/speech_bash.sh
	cd ~/catkin_ws/src/rcah18_pepper_speech
	python speech/gcloudmulti.py
	```
3. Start the openpose_ros node.
	```
	export ROS_MASTER_URI=http://128.237.247.249:11311
	roslaunch openpose_ros openpose_ros.launch
	```
4. Start the speech node.
	```
	export ROS_MASTER_URI=http://128.237.247.249:11311
	roscd pepper_demo
	cd scripts
	python timestamp_speech.py
	```	
5. Start the process openpose node.
	```
	export ROS_MASTER_URI=http://128.237.247.249:11311
	roscd pepper_demo
	cd scripts
	python process_openpose_humans.py
	```	
6. Start the pepper_demo script.
	```
	export ROS_MASTER_URI=http://128.237.247.249:11311
	roscd pepper_demo
	cd scripts
	python2 pepper_demo_script.py
	```

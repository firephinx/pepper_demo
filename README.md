# pepper_demo

## Instructions

### Running the Actual Demo
1. Ssh to pepper in a new terminal and start roscore on Pepper.
	```
	sshnao
	./startros
	```
2. Create a new terminal and start the speech node on Pepper.
	```
	sshnao
	source gspeech/speech_bash.sh
	cd ~/catkin_ws/src/rcah18_pepper_speech
	python speech/gcloudmulti.py
	```
3. Start the pepper_demo node in a new terminal.
	```
	snao
	roslaunch pepper_demo pepper_demo.launch
	```

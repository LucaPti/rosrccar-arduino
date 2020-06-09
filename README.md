# rosrccar-arduino
Arduino part of my project for an RC car that passes all signals to a Raspberry Pi that runs ROS. While this is mostly for educational purposes, I hope to make implement some fun controllers.

## Project Goal
The Arduino, currently an Arduino Uno is intended as a real-time capable hardware interface for ROS and should run some configurable controllers.
For now, I intend to provide the hardware interface:
* Read throttle and steering from an RC receiver.
* Read data from an optical [ADNS3050 sensor](https://github.com/Tom101222/Adns-3050-Optical-Sensor).
* Output PWM signals for throttle and steering.
In the future I want to have some routing capability that can either pipe through the RC commands or execute a simple controller in real time.
Via serial connection I provide ROS topics for RC controls, optical sensor and car controls.

## Preparations
In order to have the necessary libraries to compile the code, you need to generate them from ROS, using the project from my other repository. I found [this tutorial](https://maker.pro/arduino/tutorial/how-to-use-arduino-with-robot-operating-system-ros) to be instructive except for generating the library, which is easy: 
* `cd catkin_ws`
* `source ./devel/setup.bash`
* `rosrun rosserial_arduino make_libraries.py ~/Arduino/libraries`
and then add them to your Arduino IDE. Build and flash your project like normal. If you cannot access your Arduino you might have to modify the access rights with this command: `sudo chmod a+rw /dev/ttyACM0`.

## See something in ROS
You can visualize your project in ROS using these commands in seperate terminal windows
* `roscore`
* `rosrun rosserial_python serial_node.py /dev/ttyACM0` (possibly after modifying the access rights to your Arduino and sourcing setup.bash, replace ttyACM0 with correct device name).
* `rostopic echo _whatevertopicyoulike_` (after sourcing setup.bash)
* `rqt_plot optical_sensor/delta_x` (after sourcing setup.bash, choose any other signal you like)
* `rostopic hz _whatevertopicyoulike_` gives the publication frequency.

## Drive the car via computer
You can pipe the commands from the RC controller back to the arduino using `rosrun topic_tools relay rc_input rc_output`.
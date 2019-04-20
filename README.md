# Rasp Dumpling

## Raspberry Pi Operation
### Turning the Pi on
Plug USB power cable into the Pi. Wait for both red and green LED lights to turn on. It will automatically connect to the `CMU` WiFi network. To SSH, make sure your computer is either on the `CMU-SECURE` or `CMU` network.

### SSH using password
SSH Hostname: arcpi.wv.cc.cmu.edu  
login: ozro  
passwd: dumpling  
```
$ ssh ozro@arcpi.wv.cc.cmu.edu
```

### SSH using key
SSH login: ozro@arcpi.wv.cc.cmu.edu  
The SSH key file `ubuntu_core` is on Google drive.

### SSH over ethernet
If the RaspberryPi is connected to the computer through ethernet, we can connect to it at its static IP. The Pi's IP is `10.42.0.128` and the computer's IP is `10.42.0.1`.
We can SSH using this static IP.
```
$ ssh ozro@10.42.0.128
```

### Classic Mode
To use commands such as `git`, `apt` and any ROS command, we need to enter classic mode.
```
$ sudo classic
...
(classic) localhost:~$ roscore
```

### Turning the Pi off
First exit classic mode by pressing `ctrl+D` or typing the `exit` command. Then run the shutdown command.
```
$ sudo shutdown now
```
Wait for LED lights on the Pi to stop blinking. There should only be a solid red LED left. Unplug the power cable.

## Launching the Nodes
We need to enter "classic" mode to use ROS and other functions such as `apt` and `git`.
```
$ sudo classic
```
It is probably a good idea to make sure the code is up to date.
```
(classic) localhost:~$ git pull
```
If the remote master is not availabe, run `roscore` first in another terminal session.  
```
(classic) localhost:~$ roscore
```
To run the controller node we need elevated privileges because we are accessing GPIO pins.
We also need to run the `pigpio` daemon for the encoder counter to work.
```
(classic) localhost:~$ sudo su
(classic) localhost:~$ source devel/setup.bash
```
We need to export the appropriate variables
```
(classic) localhost:~$ export ROS_MASTER_URI=http://10.42.0.1:11311
(classic) localhost:~$ export ROS_IP=10.42.0.128
```
We can then use the launch file to start all required nodes
```
(classic) localhost:~$ roslaunch rasp teleop.launch
```
The nodes can also be run individually. For example:
```
(classic) localhost:~$ rosrun rasp EncoderNode.py
```
## Node details
* `joy_node` - Publishes joystick button/axis events. For details see https://wiki.ros.org/joy.
* `TeleopNode.py` - Converts joystick events into motor commands. These may need to be adjusted.
  * The `start` button starts the motors
  * The `back` button stops the motors
  * The `B` button brakes the motors
  * The left stick controls the linear velocity
  * The right stick (moved left or right) controls the rotation
* `MotorNode.py` - Sends serial commands to motors and publishes motor status
  * Subscribers
    * Velocities (float [0,1]) `/motor_vel` Float32MultiArray
    * Start/Stop (True starts motors, False stops motors) `/motor_cmd/start` Bool
    * Brake (Sets velocities to zero) `/motor_cmd/brake`
  * Publishers (For details see Pololu SMC G2 user manual)
    * Error Status `/motor_status/errors` UInt16MultiArray
    * Limit Status `/motor_status/limit_status` UInt16MultiArray
    * Desired Speeds `/motor_status/target_speeds` Int16MultiArray
    * Actual Speeds `/motor_status/current_speeds` Int16MultiArray
    * Board temperatures `/motor_status/temps` UInt16MultiArray
    * Currents `/motor_status/currents` UInt16MultiArray
    * Voltages `/motor_status/voltages` UInt16MultiArray
* `EncoderNode.py` - Publishes encoder tick counts
  * Publishers
    * `/motor_status/encoder_counts` Int16MultiArrayStamped

## Encoder Data
Encoder counts per revolution: 134.4

## Motor Controller Details
Find the user's manual here https://www.pololu.com/docs/pdf/0J77/simple_motor_controller_g2.pdf. The mapping follows 5.4 Logitech Wireless Gamepad.

### LED Feedback
There are three LEDs on the board:
* Green (Only on if USB is connected)
* Red (Error)
  * This light turns on if there are any errors stopping the motor (see section 3.3). The motor will not move if the red light is on. The `exit_safe_start` command will remove the error. You can send a `True` message on the topic `/motor_cmd/start` to start the motors. Sending `False` will stop the motors.
* Yellow (Status)
  * During start up a pattern shows the source of the last reset
    * 8 blinks over the first two seconds means the controller was reset by pin
    * 3 blinks over the first two seconds means the controller lost power
    * Rapid flickering for the first two seconds means there was a software fault or upgrade
  * After start up the LED gives feedback on motor driver outputs
    * An even blinking pattern of on for 2/3s and off for 2/3s indicates that the controller is not driving the motor and has not yet detected the baud rate. This means no correctly formatted message was sent yet.
    * A brief flash once per second indicates that the controller is not driving the motor, but it has received at least one correctly formatted messsage and learned the baud rate.
    * A repeating, gradual increase in brightness every second indicates that the controller isdriving the motor forward.
    * A repeating, gradual decrease in brightness every second indicates that the controller isdriving the motor in reverse.

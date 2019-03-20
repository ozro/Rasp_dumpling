# Rasp Dumpling
## Turning the RaspberryPi on
Plug USB power cable into RaspberryPi. Wait for LED lights to turn on. It will automatically connect to the `CMU` WiFi network.

## SSH into RaspberryPi
SSH Hostname: arcpi.wv.cc.cmu.edu  
login: pi  
passwd: dumpling  
```$ ssh pi@arcpi.wv.cc.cmu.edu```

## Turning the RaspberryPi off
First exit classic mode using `ctrl+D`.
```
sudo shutdown now
```
Wait for LED lights on Pi to stop blinking. There should only be a red solid LED left. Unplug the power cable.

## Nodes
* joy_node - Publishes joystick button/axis events
* TeleopNode.py - Converts joystick events into motor commands
* MotorNode.py - Sends serial commands to motors and publishes motor status
  * Publishers (For details see Pololu SMC G2 user manual)
    * Error Status `/motor_status/errors` UInt16MultiArray
    * Limit Status `/motor_status/limit_status` UInt16MultiArray
    * Desired Speeds `/motor_status/target_speeds` Int16MultiArray
    * Actual Speeds `/motor_status/current_speeds` Int16MultiArray
    * Board temperatures `/motor_status/temps` UInt16MultiArray
    * Currents `/motor_status/currents` UInt16MultiArray
    * Voltages `/motor_status/voltages` UInt16MultiArray
* EncoderNode.py - Publishes encoder tick counts
  * Publishers
    * `/motor_status/encoder_counts` Int16MultiArray

## Launching Teleop Nodes
We need to enter "classic" mode to use ROS.
If the remote master is not availabe, run `roscore` first.
We need elevated privileges to access GPIO pins.
```
$ sudo classic
$ sudo su
$ source devel/setup.bash
$ roslaunch rasp teleop.launch
```

## Encoder data
Encoder counts per revolution: 134.4

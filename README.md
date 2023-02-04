# DroneForce

## What it does
- Control individual motor PWM outputs
- Give body torque as input and automatically resolve to PWM

## How it works
- DroneForce is initalized
    - Frame data
        - Type, class
        - Mass
        - Inertia
    - Motors (currently hard-coded)
        - kx (thrust co-efficient)
        - lx (x-distance to CG)
        - ly (y-distance to CG)
    - Actuator Effectiveness Matrix (currently hard-coded)
        - Rows - r, p, y, T allocation for each motor
        - Cols - repeat Row for each motor
    - Control Allocation Matrix
        - Pseudo-inverse of EA Matrix
- Motor is pre-set to be controlled over PWM
    - ```commander.set_motor_mode(i, 1)```
    - Above command sets the parameter [SERVOn_FUNCTION](https://ardupilot.org/plane/docs/parameters.html#servo1-function-servo-output-function) to 1 (RCPassThru)
    - SERVO1_FUNCTION is usually set to Motor1 (and so on...), an identifier for the selected n-th motor. It can be set to another value for changing the config.
    - SERVOn_FUNCTION value RCPassThru lets us take a single RC channel as PWM value output for motor (i.e. Direct RC override without the usual RC mapping for channels[roll, pitch, yaw, th] etc)
- Controller gives input for torque(p,q,r) and thrust (T)
    - For e.g, in [tests/test-26-pid-hold.py](tests/test-26-pid-hold.py):
        - ```torq_cmd``` takes Torque values (numpy array of 3 floats)
        - ```th_cmd``` takes Thrust value (float)
- Scale torque/thrust output to PWM range
    - linearly
        - ```torque_to_PWM()``` maps input Torque range to output PWM range
    - non-linearly (by modelling motor characterstics)
        - (To-Do)
- Send motor PWM output command via MAVLink
    - ```commander.set_servo(i, PWM)``` sets the PWM value for the i-th motor
    - Above function uses MAVLink`s [MAV_CMD_DO_SET_SERVO](https://mavlink.io/en/messages/common.html#MAV_CMD_DO_SET_SERVO) packet through [command_long_encode](https://mavlink.io/en/messages/common.html#COMMAND_LONG) function
    - MAVLink packet is sent using ```message_factory.command_long_encode()``` function from the DroneKit API
        - Can be replaced with MAVLink alternative directly (To-Do)\
        - (To-Do) Will reduce 1 connection to ArduPilot; might improve response time
## Setting up DroneForce
- Create your workspace
```
mkdir -p ~/df_ws/src && cd ~/df_ws/src
```
- Clone DroneForce
```
git clone https://github.com/Embedded-Fault-Tolerant-Control/DroneForce
```
- Clone Ardupilot and [set up SITL](https://ardupilot.org/dev/docs/setting-up-sitl-on-linux.html) (optional)
```
git clone --recursive https://github.com/ArduPilot/ardupilot
```
- Clone Ardupilot-Gazebo and [set it up](https://github.com/SwiftGust/ardupilot_gazebo) (optional)
```
git clone https://github.com/SwiftGust/ardupilot_gazebo
```
- Install prerequisites with pip
```
cd ~/df_ws/src/DroneForce
pip install -r requirements.txt
```
- Install Terminator (for multiple terminal windows)
```
sudo apt install terminator
```
- Ensure you have ROS setup already in your ws (tested on Noetic)
## Using DroneForce in your project
### GPS dependant
#### For simulation, run:
Open up the following terminals

1. 
```
mavproxy.py --master 127.0.0.1:14551 --out=udp:127.0.0.1:14552 --out=udp:127.0.0.1:14553 --out=udp:127.0.0.1:14554
```
2. 
```
cd ~/df_ws/src/ardupilot_gazebo/worlds 
gazebo --verbose iris_ardupilot.world
```
3. Only when you need to generate a trajectory (ideally after drone is hovering and stable)
```
cd ~/df_ws/src/DroneForce/tests
python3 test-26-trajectory-generator.py
```
4. 
```
cd ~/df_ws/src/ardupilot/Tools/autotest
```
For Gazebo simulation:
```
python3 sim_vehicle.py -v ArduCopter -f gazebo-iris -m --mav10
```
For headless simulation:
```
python3 sim_vehicle.py -v ArduCopter -f X -m --mav10
```
5. Source your ROS setup if you already haven't, before running this step
```
roslaunch mavros apm.launch fcu_url:=udp://:14553@
```
6. When you want to record a rosbag file
```
cd ~/df_ws/src/DroneForce/dist
rosbag record -a
```
7. Actual test file for the controller
```
cd ~/df_ws/src/DroneForce/tests
python3 test-26-pid-hold.py
```
#### For real, run:
Open up the following terminals

1. Ensure your Pixhawk hardware running ArduPilot firmware is already connected
```
mavproxy.py --master=/dev/ttyUSB0 --out=udp:127.0.0.1:14552 --out=udp:127.0.0.1:14553 --out=udp:127.0.0.1:14554
```
2. Only when you need to generate a trajectory (ideally after drone is hovering and stable)
```
cd ~/df_ws/src/DroneForce/tests
python3 test-26-trajectory-generator.py
```
3. Source your ROS setup if you already haven't, before running this step
```
roslaunch mavros apm.launch fcu_url:=udp://:14553@
```
4. When you want to record a rosbag file
```
cd ~/df_ws/src/DroneForce/dist
rosbag record -a
```
5. Actual test file for the controller
```
cd ~/df_ws/src/DroneForce/tests
python3 test-26-pid-hold.py
```

### Motion-capture dependant
- (To-do)
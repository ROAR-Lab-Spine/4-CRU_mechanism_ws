# 4-CRU Mechanism Control

## rosserial Installation [Link](http://wiki.ros.org/rosserial_arduino/Tutorials/Arduino%20IDE%20Setup)
1. Remember to setup keys
```
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
sudo apt-get update
```
2. Run the command (for ROS kinetic)
```
sudo apt-get install ros-kinetic-rosserial-arduino
sudo apt-get install ros-kinetic-rosserial
```
3. Add ros_lib to Arduino library
```
cd ~/Arduino/libraries
rosrun rosserial_arduino make_libraries.py ~/Arduino/libraries
```

## SymPy Installation [Documentations](https://docs.sympy.org/latest/index.html) and [Downloads](https://github.com/sympy/sympy/releases)
'''
tar zxvf sympy-<VERSION>.tar.gz
cd sympy-<VERSION>
sudo python setup.py install
'''

## Arduino Libraries
- [PID](https://playground.arduino.cc/Code/PIDLibrary/)
- [Timeaction](https://playground.arduino.cc/Code/TimedAction/)


Enable user to access Arduino MEGA 2560 port
```
sudo usermod -a -G dialout $USER
sudo chmod a+rw /dev/ttyACM0 
```

# 5-Axis Robotic Arm Firmware
<br>
![[robot_gif.gif]]
<br>

## What is this program?
<p>The firmware is an Arduino-based program which purpose is to enable the control a 5-axis robot's end effector. It can run on all Arduino and Teensy boards. Among others, it features smooth motion control, Cartesian coordinates and orientation control of the end effector.</p>
<p>Specifically made for a custom 5-axis robot arm, it will need to be modified if intended to be used on a different machine.</p>


## How to use it?
<ol>
    <li><p>Download the [Arduino IDE](https://www.arduino.cc/en/software).</p></li>
    <li><p>If using a Teensy board, download the [Teensy pluggin](https://www.pjrc.com/teensy/td_download.html) for Arduino compatibility.</p></li>
    <li><p>Make sure the Servo.h library is installed in your system. If using a Teensy board, make sure to use Teensy's specific Servo.h library.</p></li>
    <li><p>Update the calibration of the servo motors' pulse width with the ones of your own motors.</p></li>
    <li><p>Compile and flash the firmware</p></li>
<ol>
<br>
A full description of the project and its technical details [can be found here.](https://valentinbetbeze.com/robotic_arm.html)
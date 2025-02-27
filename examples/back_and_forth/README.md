# Back And Forth Starling Flight Path with ROS

## Deriving Flight Parameters
To compute the flight trajectory, we will first define the desired characteristic. Here, we want Starling to take off to 1 meter of altitude, fly forward 1 meter over the course of 2 seconds, pause for 2 seconds, then fly backwards over 1 meter for 2 seconds before landing. 

Here, I will implement a cubic easing function which generates a smooth and natural flight. To compute such a function, I can use the function
$$
s(t) = x\cdot\left[3\left(\frac{t}{T}\right)^2-2\left(\frac{t}{T}\right)^3\right].\tag{1}
$$

From this position function, where $x$ is the end position and $T$ is the total time elapsed, the velocity and acceleration can be derived using Eqns. 2 & 3.

$$v(t)=\dfrac{d}{dt}s(t)\tag{2}$$
$$a(t)=\dfrac{d^2}{dt^2}s(t)\tag{3}$$

This leaves a resultant velocity of
$$v(t)\frac{-6t(t-T)x}{T^3}\tag{4}$$

and an acceleration of 
$$a(t)=\frac{-6(2t-T)x}{T^3}.\tag{5}$$

Plugging in $x=1$ (our final position) and $T=2$ (our elapsed time), the flight characteristics for the starling fter takeoff are as follows:

| |x|y|z|
|---|---|---|---|
|Position|$\left(\frac{t}{2}\right)^2-2\left(\frac{t}{2}\right)^3$|0|-1|
|Velocity|$\frac{-6t(t-2)}{8}$|0|0|
|Acceleration|$\frac{-6(2t-2)}{8}$|0|0|

Note that the altitude ($z$-axis) is set to a constant $-1$. This is because for all setpoints in ROS on the Starling, the PX4 reference frame must be adhered to. 

![image](https://docs.px4.io/main/assets/frame_heading.BvCcZ-mD.png)
$$\text{Figure 1: PX4 coordinate frames}$$

## Implementing Control Script
See [back_and_forth.py](back_and_forth.py) for an example of this implementation. 

## Setup
Follow the typical setup steps provided by ModalAI. Install ROS using Modal AI's [ROS2 guide](https://docs.modalai.com/ros2-installation-voxl2/). Getting everything configured correctly to communicate with other ROS devices can be challenging, as currently only ROS2 Foxy is supported by ModalAI. 

## Simulation
Prior to running a ROS script on the Starling in a live test environment, it is often useful to simulate the flight to ensure behavior is as expected. Follow [Modal AI's guide](https://docs.modalai.com/voxl2-PX4-hitl/) on HITL simulation. 

## Flight
For flying in offboard mode, the included Starling controller is not necessary, but can aid in safety. Power on the controller and set the left bumper toggle to the center position. Then, ensure the kill switch is not depressed. 

After configuring Starling, you should be able to SSH into its command line, in which you can pull this repository or otherwise install the flight control sctipt. Be sure any scripts intended to run with ROS are properly configured in a ROS package. 




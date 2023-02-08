# Quaternion Based AHRS Estimation Using MPU9250 and STM32F407-Disc
# What is AHRS ?
AHRS stands for “Attitude and Heading Reference System”.

An AHRS consists of sensors on three axes that provide attitude information for aircraft, including roll, pitch and yaw. These are sometimes referred to as MARG (Magnetic, Angular Rate, and Gravity) sensors and consist of either solid-state or microelectromechanical systems (MEMS) gyroscopes, accelerometers and magnetometers.

# What is Quaternion ?
There are lots of different definitions present for Quaternions, and they are varys on application.

If we talk about its use in AHRS systems, we use it to equate two different coordinate systems to each other, in the most “rough” terms. Here, the first is the plane’s coordinate system (The body frame), and the other is the coordinate system on which its navigation  (It’s Earth ).


![body frame](https://user-images.githubusercontent.com/109542834/182877391-cfed3324-baaf-49fb-b1a0-652efef50571.gif)

To explain in a little more detail, the orientation of a aircraft can be described by two coordinate frames. As shown in Figure 1, navigation frame (n frame) is organized by East-North-Up (ENU) definition. Let aircrafts right side as body frame (b frame) xb positive direction, forward as body frame yb positive direction and straight up as body frame zb positive direction. The origin of the b frame and n frame are aligned at point O. Hence, θ, ϕ, ψ represent pitch, roll and yaw angles respectively according to the Euler angles definition. A quaternion q is a ℜ4 vector that can be used to represent the orientation of b frame relative to n frame in ℜ3. An arbitrary orientation of b frame relative to n frame can be achieved through a rotation of angle α around the vector r in n frame.

Quaternion q is defined by:


![qyatjpg](https://user-images.githubusercontent.com/109542834/183050496-e66e41e2-e6f0-4603-ba46-654a99df5dd3.jpg)

Quaternion provides us with a way for rotating a point around a specified axis by a specified angle.

# Conversion between Quaternions and Euler Angles 

To get pitch, yaw and roll, we need to convert estimated quaternions to the Euler Angles.

A unit quaternion can be described as:

![1](https://user-images.githubusercontent.com/109542834/183052292-5f7ef92b-df05-472c-b697-e2e43ea34934.png)

We can associate a quaternion with a rotation around an axis by the following expression:

![2](https://user-images.githubusercontent.com/109542834/183052435-1d23f1a0-8dfd-4579-a96d-554d11317faf.jpg)

where α is a simple rotation angle (the value in radians of the angle of rotation) and cos(βx), cos(βy) and cos(βz) are the “direction cosines” of the angles between the three coordinate axes and the axis of rotation.

To better understand how “direction cosines” work with quaternions:

![3](https://user-images.githubusercontent.com/109542834/183052592-c7573725-dc46-44ea-a588-927e0b232e92.png)

The Euler angles can be obtained from the quaternions via the relations:

![4](https://user-images.githubusercontent.com/109542834/183052831-df4aa949-267e-46b5-ba48-859e0ce89ac7.png)

# Hardware and Setup
I used STM32F407 dicovery kit and MPU9250 as the IMU sensor :

![circuit](https://user-images.githubusercontent.com/109542834/183058842-f07fce78-5494-4300-bf7d-01a71817cc99.jpg)

# The code 

![image](https://user-images.githubusercontent.com/109542834/183060180-0d5eb808-a315-4bae-b7ca-f1669a580430.png)


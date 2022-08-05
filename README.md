# Quaternion Based AHRS Estimation Using MPU9250 and STM32F407
# What is AHRS ?
AHRS stands for “Attitude and Heading Reference System”.

An AHRS consists of sensors on three axes that provide attitude information for aircraft, including roll, pitch and yaw. These are sometimes referred to as MARG (Magnetic, Angular Rate, and Gravity) sensors and consist of either solid-state or microelectromechanical systems (MEMS) gyroscopes, accelerometers and magnetometers.

# What is Quaternion ?
There are lots of different definitions present for Quaternions, and they are varys on application.

If we talk about its use in AHRS systems, we use it to equate two different coordinate systems to each other, in the most “rough” terms. Here, the first is the plane’s coordinate system (The body frame), and the other is the coordinate system on which its navigation  (It’s Earth ).


![body frame](https://user-images.githubusercontent.com/109542834/182877391-cfed3324-baaf-49fb-b1a0-652efef50571.gif)

To explain in a little more detail, the orientation of a aircraft can be described by two coordinate frames. As shown in Figure 1, navigation frame (n frame) is organized by East-North-Up (ENU) definition. Let aircrafts right side as body frame (b frame) xb positive direction, forward as body frame yb positive direction and straight up as body frame zb positive direction. The origin of the b frame and n frame are aligned at point O. Hence, θ, ϕ, ψ represent pitch, roll and yaw angles respectively according to the Euler angles definition. A quaternion q is a ℜ4 vector that can be used to represent the orientation of b frame relative to n frame in ℜ3. An arbitrary orientation of b frame relative to n frame can be achieved through a rotation of angle α around the vector r in n frame.

Quaternion q is defined by:

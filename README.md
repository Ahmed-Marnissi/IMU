# Quaternion Based AHRS Estimation Using MPU9250 and STM32F407
#What is AHRS?
AHRS stands for “Attitude and Heading Reference System”.

An AHRS consists of sensors on three axes that provide attitude information for aircraft, including roll, pitch and yaw. These are sometimes referred to as MARG (Magnetic, Angular Rate, and Gravity) sensors and consist of either solid-state or microelectromechanical systems (MEMS) gyroscopes, accelerometers and magnetometers. They are designed to replace traditional mechanical gyroscopic flight instruments.

The key difference between an inertial measurement unit (IMU) and an AHRS is the addition of an on-board processing system in an AHRS, which provides attitude and heading information. This is in contrast to an IMU, which just delivers sensor data to an additional device that computes attitude and heading. With sensor fusion, drift from the gyroscopes integration is compensated for by reference vectors, namely gravity, and the earth magnetic field. This results in a drift-free orientation, making an AHRS a more cost effective solution than conventional high-grade IMUs (Inertial Measurement Units) that only integrate gyroscopes and rely on a very high bias stability of the gyroscopes. In addition to attitude determination an AHRS may also form part of an inertial navigation system.

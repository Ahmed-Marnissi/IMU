///*
// * IMU.h
// *
// *  Created on: Jul 19, 2022
// *      Author: EXtrA
// */
//#ifndef INC_IMU_H_
//#define INC_IMU_H_
//
//#include "math.h"
//#include "stdint.h"
//#include "stm32f4xx.h"
//#include "MPU9250RegisterMap.h"
//#include "Inc/Quaternions.h"
//
//
//
//
//
//
//typedef enum   {
//    A2G,
//    A4G,
//    A8G,
//    A16G
//}ACCEL_FS_SEL;
//typedef enum   {
//    G250DPS,
//    G500DPS,
//    G1000DPS,
//    G2000DPS
//}GYRO_FS_SEL;
//typedef enum    {
//    M14BITS,
//    M16BITS
//}MAG_OUTPUT_BITS;
//
//typedef enum  {
//    SMPL_1000HZ,
//    SMPL_500HZ,
//    SMPL_333HZ,
//    SMPL_250HZ,
//    SMPL_200HZ,
//    SMPL_167HZ,
//    SMPL_143HZ,
//    SMPL_125HZ,
//}FIFO_SAMPLE_RATE;
//
//typedef enum {
//	GYRODLPF_250HZ,
//	GYRODLPF_184HZ,
//	GYRODLPF_92HZ,
//	GYRODLPF_41HZ,
//	GYRODLPF_20HZ,
//	GYRODLPF_10HZ,
//	GYRODLPF_5HZ,
//	GYRODLPF_3600HZ,
//}GYRO_DLPF_CFG;
//
//typedef enum  {
//	ACCELDLPF_218HZ_0,
//	ACCELDLPF_218HZ_1,
//	ACCELDLPF_99HZ,
//	ACCELDLPF_45HZ,
//	ACCELDLPF_21HZ,
//	ACCELDLPF_10HZ,
//	ACCELDLPF_5HZ,
//	ACCELDLPF_420HZ,
//}ACCEL_DLPF_CFG ;
//
//
//struct MPU9250
//{
//	ACCEL_FS_SEL AccelFs ;
//	GYRO_FS_SEL GyroFs ;
//
//	MAG_OUTPUT_BITS MagOutputBits ;
//
//	FIFO_SAMPLE_RATE FifoSampleRate ;
//	uint8_t GyroFChoice ;
//	GYRO_DLPF_CFG GyroDlpfCfg;
//	uint8_t AccelFChoice ;
//	ACCEL_DLPF_CFG AccelDlpfCfg ;
//
//};
//
//
//I2C_HandleTypeDef hi2c1;
//
//
//struct MPU9250  MPU9250Settings ={.AccelFs =A16G,
//		.GyroFs =  G2000DPS,
//
//		. MagOutputBits =M16BITS,
//
//		. FifoSampleRate=  SMPL_200HZ ,
//		. GyroFChoice= 0x03,
//		. GyroDlpfCfg=GYRODLPF_41HZ,
//		. AccelFChoice =0x01,
//		 .AccelDlpfCfg= ACCELDLPF_45HZ,
//} ;
//
//
//
// float AccEst[3] ={0.0,0.0,0.0} ; // Accelerometer data estimation buffer
// float GyroEst[3] ={0.0,0.0,0.0} ; // Gyroscope data estimation buffer
// float MagEst[3] ={0.0,0.0,0.0} ; // magnetometer data estimation buffer
// float Yaw = 0.0 , Pitch = 0.0 , Roll = 0.0,yaw_filtred=0.0 ; // The Euler angles that will be calculated
// float RollDeg =0.0 , PitchDeg = 0.0 ;// the angles in degrees
// float NormMag = 0.0 , NormAcc =0.0 ;// the norm of the accelerometer and magnetometer data
//
// static const uint8_t MPU9250_WHOAMI_DEFAULT_VALUE =0x71;
// static const uint8_t MPU9255_WHOAMI_DEFAULT_VALUE =0x73;
// static const uint8_t MPU6500_WHOAMI_DEFAULT_VALUE =0x70;
//
// static const uint8_t MPU9250_DEFAULT_ADDRESS =0xD0;  // Device address when ADO = 0
// static const uint8_t AK8963_ADDRESS =0x18;           //  Address of magnetometer
// static const uint8_t AK8963_WHOAMI_DEFAULT_VALUE =0x48;
// uint8_t MpuI2cAddress =MPU9250_DEFAULT_ADDRESS;
// uint8_t MagI2cAddress =AK8963_ADDRESS;
//
//
//
//    // TODO: this should be configured!!
//    static const uint8_t MAG_MODE =0x06;  // 0x02 for 8 Hz, 0x06 for 100 Hz continuous magnetometer data read
//    float AccResolution=0.0 ;                // scale resolutions per LSB for the sensors
//    float GyroResolution =0.0;               // scale resolutions per LSB for the sensors
//    float MagResolution =0.0;                // scale resolutions per LSB for the sensors
//
//    // Calibration Parameters
//    float AccBias[3] = {0.0, 0.0, 0.0};   // acc calibration value in ACCEL_FS_SEL: 2g
//    float GyroBias[3] = {0.0, 0.0, 0.0};   // gyro calibration value in GYRO_FS_SEL: 250dps
//    float MagBiasFactory[3] = {0.0, 0.0, 0.0};
//    float MagBias[3] = {0.0, 0.0, 0.0};  // mag calibration value in MAG_OUTPUT_BITS: 16BITS
//    float MagScale[3]= {0.0, 0.0, 0.0};
//    float MagneticDeclination = 0.03490659;  //Arianaa en rad
//
//
//
//    // IMU Data
//    float A[3]={0.0, 0.0, 0.0};
//    float G[3] ={0.0, 0.0, 0.0};
//    float M[3] ={0.0, 0.0, 0.0};
//
//
//   //Temperature
//    int16_t TemperatureCount =0;  // temperature raw count output
//    float Temperature=0.;        // Stores the real internal chip temperature in degrees Celsius
//
//
//    // Other settings
//    int  HasConnected =0;
//
//
//    static const uint16_t CalibGyroSensitivity =131;     // LSB/degrees/sec
//    static const uint16_t CalibAccSensitivity=16384;  // LSB/g
//
//
//
//
//    float Q[4] = {1.0, 0.0, 0.0, 0.0};  // vector to hold quaternion
//    float rpy[3] = {0.0, 0.0, 0.0};
//
//
//int IsConnectedMPU9250();
//int IsConnectedAK8963();
//int IsConnected();
//void  InitMPU9250();
//void InitAK8963();
//int  Setup();
//float GetAccResolution(ACCEL_FS_SEL AccelFs);
//float GetMagResolution( MAG_OUTPUT_BITS MagOutputBits);
//float GetGyroResolution ( GYRO_FS_SEL GyroFs);
//void  UpdateAccelGyro();
//
//void CalibrateGyroAcc();
//void SetAccGyroToCalibration();
//void WriteAccelOffset();
//void WriteGyroOffset();
//void CollectAccGyroDataToCalibration(float* ABias, float* GBias) ;
//
//uint8_t  ReadByte(uint8_t DevAddress, uint8_t Register );
//void ReadBytes(uint8_t DevAddress, uint8_t Register, uint8_t Count, uint8_t* Destination);
//void WriteByte(uint8_t DevAddress, uint8_t Register, uint8_t Data);
//void CollectMagDataToCalibration(float* MBias, float* MScale);
//int CalibrateMag();
//void SetAccBias( float x,  float y,  float z) ;
//
//void SetGyroBias( float x,  float y,  float z) ;
//
//void SetMagBias( float x,  float y,  float z) ;
//
//void SetMagScale( float x,  float y,  float z) ;
//
//void Update() ;
//float GetAccX() ;
//float GetAccY() ;
//float GetAccZ() ;
//float GetGyroX();
//float GetGyroY() ;
//float GetGyroZ() ;
//float GetMagX()  ;
//float GetMagY()  ;
//float GetMagZ()  ;
//
//
//
//float GetAccBiasX() ;
//float GetAccBiasY() ;
//float GetAccBiasZ() ;
//float GetGyroBiasX() ;
//float GetGyroBiasY() ;
//float GetGyroBiasZ();
//float GetMagBiasX();
//float GetMagBiasY();
//float GetMagBiasZ();
//float GetMagScaleX();
//float GetMagScaleY();
//float GetMagScaleZ();
//
//
//void SetAccBias( float x,  float y,  float z);
//
//void SetGyroBias( float x,  float y,  float z) ;
//
//void SetMagBias( float x,  float y,  float z) ;
//
//void SetMagScale( float x,  float y,  float z) ;
//
//void SetMagneticDeclination(  float d);
//
//
//void UpdateRpy();
//
//
//#endif /* INC_IMU_H_ */

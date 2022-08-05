
#ifndef INC_IMU_H_
#define INC_IMU_H_

#include "math.h"
#include "stdint.h"
#include "stm32f4xx.h"
#include "MPU9250RegisterMap.h"
#include "Inc/Quaternions.h"






typedef enum   {
    A2G,
    A4G,
    A8G,
    A16G
}ACCEL_FS_SEL;
typedef enum   {
    G250DPS,
    G500DPS,
    G1000DPS,
    G2000DPS
}GYRO_FS_SEL;
typedef enum    {
    M14BITS,
    M16BITS
}MAG_OUTPUT_BITS;

typedef enum  {
    SMPL_1000HZ,
    SMPL_500HZ,
    SMPL_333HZ,
    SMPL_250HZ,
    SMPL_200HZ,
    SMPL_167HZ,
    SMPL_143HZ,
    SMPL_125HZ,
}FIFO_SAMPLE_RATE;

typedef enum {
	GYRODLPF_250HZ,
	GYRODLPF_184HZ,
	GYRODLPF_92HZ,
	GYRODLPF_41HZ,
	GYRODLPF_20HZ,
	GYRODLPF_10HZ,
	GYRODLPF_5HZ,
	GYRODLPF_3600HZ,
}GYRO_DLPF_CFG;

typedef enum  {
	ACCELDLPF_218HZ_0,
	ACCELDLPF_218HZ_1,
	ACCELDLPF_99HZ,
	ACCELDLPF_45HZ,
	ACCELDLPF_21HZ,
	ACCELDLPF_10HZ,
	ACCELDLPF_5HZ,
	ACCELDLPF_420HZ,
}ACCEL_DLPF_CFG ;


struct MPU9250
{
	ACCEL_FS_SEL AccelFs ;
	GYRO_FS_SEL GyroFs ;

	MAG_OUTPUT_BITS MagOutputBits ;

	FIFO_SAMPLE_RATE FifoSampleRate ;
	uint8_t GyroFChoice ;
	GYRO_DLPF_CFG GyroDlpfCfg;
	uint8_t AccelFChoice ;
	ACCEL_DLPF_CFG AccelDlpfCfg ;

};


extern I2C_HandleTypeDef hi2c1;



 extern float AccEst[3];  // Accelerometer data estimation buffer
 extern float GyroEst[3] ; // Gyroscope data estimation buffer
 extern float MagEst[3] ;// magnetometer data estimation buffer
 extern float Yaw , Pitch  , Roll ,yaw_filtred ; // The Euler angles that will be calculated
 extern float RollDeg , PitchDeg  ;// the angles in degrees
 extern float NormMag  , NormAcc ;// the norm of the accelerometer and magnetometer data

 extern  const uint8_t MPU9250_WHOAMI_DEFAULT_VALUE ;
 extern  const uint8_t MPU9255_WHOAMI_DEFAULT_VALUE ;
 extern  const uint8_t MPU6500_WHOAMI_DEFAULT_VALUE ;

 extern  const uint8_t MPU9250_DEFAULT_ADDRESS ;  // Device address when ADO = 0
 extern  const uint8_t AK8963_ADDRESS ;           //  Address of magnetometer
 extern  const uint8_t AK8963_WHOAMI_DEFAULT_VALUE ;
 extern uint8_t MpuI2cAddress ;
 extern uint8_t MagI2cAddress ;



    // TODO: this should be configured!!
 extern   const uint8_t MAG_MODE ;  // 0x02 for 8 Hz, 0x06 for 100 Hz continuous magnetometer data read
 extern float AccResolution ;                // scale resolutions per LSB for the sensors
 extern float GyroResolution ;               // scale resolutions per LSB for the sensors
 extern float MagResolution;                // scale resolutions per LSB for the sensors

    // Calibration Parameters
 extern  float AccBias[3] ;   // acc calibration value in ACCEL_FS_SEL: 2g
 extern  float GyroBias[3] ;   // gyro calibration value in GYRO_FS_SEL: 250dps
 extern float MagBiasFactory[3] ;
 extern float MagBias[3] ;  // mag calibration value in MAG_OUTPUT_BITS: 16BITS
 extern float MagScale[3];
 extern  float MagneticDeclination;  //Arianaa en rad



    // IMU Data
    extern  float A[3];
    extern float G[3] ;
    extern float M[3] ;


   //Temperature
    extern  int16_t TemperatureCount;  // temperature raw count output
    extern float Temperature;        // Stores the real internal chip temperature in degrees Celsius


    // Other settings
    extern int  HasConnected ;


    extern const uint16_t CalibGyroSensitivity ;     // LSB/degrees/sec
    extern  const uint16_t CalibAccSensitivity;  // LSB/g




    extern float Q[4] ;  // vector to hold quaternion
    extern  float rpy[3] ;


int IsConnectedMPU9250();
int IsConnectedAK8963();
int IsConnected();
void  InitMPU9250();
void InitAK8963();
int  Setup();
float GetAccResolution(ACCEL_FS_SEL AccelFs);
float GetMagResolution( MAG_OUTPUT_BITS MagOutputBits);
float GetGyroResolution ( GYRO_FS_SEL GyroFs);
void  UpdateAccelGyro();

void CalibrateGyroAcc();
void SetAccGyroToCalibration();
void WriteAccelOffset();
void WriteGyroOffset();
void CollectAccGyroDataToCalibration(float* ABias, float* GBias) ;

uint8_t  ReadByte(uint8_t DevAddress, uint8_t Register );
void ReadBytes(uint8_t DevAddress, uint8_t Register, uint8_t Count, uint8_t* Destination);
void WriteByte(uint8_t DevAddress, uint8_t Register, uint8_t Data);
void CollectMagDataToCalibration(float* MBias, float* MScale);
int CalibrateMag();
void SetAccBias( float x,  float y,  float z) ;

void SetGyroBias( float x,  float y,  float z) ;

void SetMagBias( float x,  float y,  float z) ;

void SetMagScale( float x,  float y,  float z) ;

void Update() ;
float GetAccX() ;
float GetAccY() ;
float GetAccZ() ;
float GetGyroX();
float GetGyroY() ;
float GetGyroZ() ;
float GetMagX()  ;
float GetMagY()  ;
float GetMagZ()  ;



float GetAccBiasX() ;
float GetAccBiasY() ;
float GetAccBiasZ() ;
float GetGyroBiasX() ;
float GetGyroBiasY() ;
float GetGyroBiasZ();
float GetMagBiasX();
float GetMagBiasY();
float GetMagBiasZ();
float GetMagScaleX();
float GetMagScaleY();
float GetMagScaleZ();

float GetRoll();
float GetPitch();
float GetYaw();
void SetAccBias( float x,  float y,  float z);

void SetGyroBias( float x,  float y,  float z) ;

void SetMagBias( float x,  float y,  float z) ;

void SetMagScale( float x,  float y,  float z) ;

void SetMagneticDeclination(  float d);


void UpdateRpy();


#endif /* INC_IMU_H_ */

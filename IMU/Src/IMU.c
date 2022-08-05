


#include "../IMU/Inc/IMU.h"

I2C_HandleTypeDef hi2c1;

struct MPU9250  MPU9250Settings ={.AccelFs =A16G,
		.GyroFs =  G2000DPS,

		. MagOutputBits =M16BITS,

		. FifoSampleRate=  SMPL_200HZ ,
		. GyroFChoice= 0x03,
		. GyroDlpfCfg=GYRODLPF_41HZ,
		. AccelFChoice =0x01,
		.AccelDlpfCfg= ACCELDLPF_45HZ,
} ;



float AccEst[3] ={0.0,0.0,0.0} ; // Accelerometer data estimation buffer
float GyroEst[3] ={0.0,0.0,0.0} ; // Gyroscope data estimation buffer
float MagEst[3] ={0.0,0.0,0.0} ; // magnetometer data estimation buffer
float Yaw = 0.0 , Pitch = 0.0 , Roll = 0.0,yaw_filtred=0.0 ; // The Euler angles that will be calculated
float RollDeg =0.0 , PitchDeg = 0.0 ;// the angles in degrees
float NormMag = 0.0 , NormAcc =0.0 ;// the norm of the accelerometer and magnetometer data

const uint8_t MPU9250_WHOAMI_DEFAULT_VALUE =0x71;
const uint8_t MPU9255_WHOAMI_DEFAULT_VALUE =0x73;
const uint8_t MPU6500_WHOAMI_DEFAULT_VALUE =0x70;

const uint8_t MPU9250_DEFAULT_ADDRESS =0xD0;  // Device address when ADO = 0
const uint8_t AK8963_ADDRESS =0x18;           //  Address of magnetometer
const uint8_t AK8963_WHOAMI_DEFAULT_VALUE =0x48;
uint8_t MpuI2cAddress =MPU9250_DEFAULT_ADDRESS;
uint8_t MagI2cAddress =AK8963_ADDRESS;



// TODO: this should be configured!!
const uint8_t MAG_MODE =0x06;  // 0x02 for 8 Hz, 0x06 for 100 Hz continuous magnetometer data read
float AccResolution=0.0 ;                // scale resolutions per LSB for the sensors
float GyroResolution =0.0;               // scale resolutions per LSB for the sensors
float MagResolution =0.0;                // scale resolutions per LSB for the sensors

// Calibration Parameters
float AccBias[3] = {0.0, 0.0, 0.0};   // acc calibration value in ACCEL_FS_SEL: 2g
float GyroBias[3] = {0.0, 0.0, 0.0};   // gyro calibration value in GYRO_FS_SEL: 250dps
float MagBiasFactory[3] = {0.0, 0.0, 0.0};
float MagBias[3] = {0.0, 0.0, 0.0};  // mag calibration value in MAG_OUTPUT_BITS: 16BITS
float MagScale[3]= {0.0, 0.0, 0.0};
float MagneticDeclination = 3.1;  //Bizerte en degeres



// IMU Data
float A[3]={0.0, 0.0, 0.0};
float G[3] ={0.0, 0.0, 0.0};
float M[3] ={0.0, 0.0, 0.0};


//Temperature
int16_t TemperatureCount =0;  // temperature raw count output
float Temperature=0.;        // Stores the real internal chip temperature in degrees Celsius


// Other settings
int  HasConnected =0;


const uint16_t CalibGyroSensitivity =131;     // LSB/degrees/sec
const uint16_t CalibAccSensitivity=16384;  // LSB/g




float Q[4] = {1.0, 0.0, 0.0, 0.0};  // vector to hold quaternion
float rpy[3] = {0.0, 0.0, 0.0};

int  Setup()
{
	if (IsConnectedMPU9250())
	{
		InitMPU9250();

		if (IsConnectedAK8963())
		{  InitAK8963();
		return 1;
		}
	}

	return 0;
}



void  InitMPU9250()
{
	AccResolution = GetAccResolution(  MPU9250Settings.AccelFs  );
	GyroResolution = GetGyroResolution(  MPU9250Settings.GyroFs );






	// reset device

	WriteByte(MpuI2cAddress, (uint8_t )PWR_MGMT_1, 0x80);

	HAL_Delay(100);

	// wake up device : Clear sleep mode bit (6), enable all sensors

	WriteByte(MpuI2cAddress, (uint8_t )PWR_MGMT_1, 0x00);

	HAL_Delay(100);



	// get stable time source
	// Auto select clock source to be PLL gyroscope reference if ready else


	WriteByte(MpuI2cAddress, (uint8_t )PWR_MGMT_1, 0x01);
	HAL_Delay(200);

	// Configure Gyro and Thermometer
	// Disable FSYNC and set thermometer and gyro bandwidth to 41 and 42 Hz, respectively;
	// minimum delay time for this setting is 5.9 ms, which means sensor fusion update rates cannot
	// be higher than 1 / 0.0059 = 170 Hz
	// GYRO_DLPF_CFG = bits 2:0 = 011; this limits the sample rate to 1000 Hz for both
	// With the MPU9250, it is possible to get gyro sample rates of 32 kHz (!), 8 kHz, or 1 kHz

	uint8_t MpuConfig = (uint8_t)  MPU9250Settings.GyroDlpfCfg;



	WriteByte(MpuI2cAddress, (uint8_t )MPU_CONFIG, MpuConfig);

	// Set sample rate = gyroscope output rate/(1 + SMPLRT_DIV)
	uint8_t SampleRate = (uint8_t)MPU9250Settings.FifoSampleRate;
	// Use a 200 Hz rate; a rate consistent with the filter update rate
	// determined inset in CONFIG above



	WriteByte(MpuI2cAddress, (uint8_t )SMPLRT_DIV, SampleRate);

	// Set gyroscope full scale range
	// Range selects FS_SEL and GFS_SEL are 0 - 3, so 2-bit values are left-shifted into positions 4:3

	// get current GYRO_CONFIG register value
	uint8_t Reception;

	Reception= ReadByte(MpuI2cAddress, (uint8_t ) GYRO_CONFIG ) ;

	Reception = Reception & ~0xE0;                                     // Clear self-test bits [7:5]
	Reception = Reception & ~0x03;                                     // Clear Fchoice bits [1:0]
	Reception = Reception & ~0x18;                                     // Clear GYRO_FS_SEL bits [4:3]
	Reception = Reception | ( ((uint8_t) MPU9250Settings.GyroFs )<< 3);       // Set full scale range for the gyro
	Reception = Reception | ((~(uint8_t)MPU9250Settings.GyroFChoice) & 0x03);   // Set Fchoice for the gyro

	// Write new GYRO_CONFIG value to register

	WriteByte(MpuI2cAddress, (uint8_t )GYRO_CONFIG, Reception);

	// Set accelerometer full-scale range configuration


	// get current ACCEL_CONFIG register value


	Reception= ReadByte(MpuI2cAddress, (uint8_t ) ACCEL_CONFIG ) ;

	Reception = Reception & ~0xE0;                                 // Clear self-test bits [7:5]
	Reception = Reception & ~0x18;                                 // Clear ACCEL_FS_SEL bits [4:3]
	Reception = Reception | (  ((uint8_t )MPU9250Settings.AccelFs) << 3);  // Set full scale range for the accelerometer


	// Write new ACCEL_CONFIG register value


	WriteByte(MpuI2cAddress, (uint8_t )ACCEL_CONFIG, Reception);
	// Set accelerometer sample rate configuration
	// It is possible to get a 4 kHz sample rate from the accelerometer by choosing 1 for
	// accel_fchoice_b bit [3]; in this case the bandwidth is 1.13 kHz

	// get current ACCEL_CONFIG2 register value


	Reception= ReadByte(MpuI2cAddress, (uint8_t ) ACCEL_CONFIG2 ) ;

	Reception = Reception & ~0x0F;                                     // Clear accel_fchoice_b (bit 3) and A_DLPFG (bits [2:0])
	Reception = Reception | (~(MPU9250Settings.AccelFChoice << 3) & 0x08);    // Set accel_fchoice_b to 1
	Reception = Reception | ((uint8_t)(MPU9250Settings.AccelDlpfCfg) & 0x07);  // Set accelerometer rate to 1 kHz and bandwidth to 41 Hz




	// Write new ACCEL_CONFIG2 register value
	WriteByte(MpuI2cAddress, (uint8_t )ACCEL_CONFIG2, Reception);

	// The accelerometer, gyro, and thermometer are set to 1 kHz sample rates,
	// but all these rates are further reduced by a factor of 5 to 200 Hz because of the SMPLRT_DIV setting

	// Configure Interrupts and Bypass Enable
	// Set interrupt pin active high, push-pull, hold interrupt pin level HIGH until interrupt cleared,
	// clear on read of INT_STATUS, and enable I2C_BYPASS_EN so additional chips
	// can join the I2C bus and all can be controlled by the Arduino as master


	WriteByte(MpuI2cAddress, (uint8_t )INT_PIN_CFG, 0x22);


	// Enable data ready (bit 0) interrupt


	WriteByte(MpuI2cAddress, (uint8_t )INT_ENABLE, 0x01);


	HAL_Delay(100);




}

void InitAK8963()

{

	MagResolution = GetMagResolution(MPU9250Settings.MagOutputBits);

	// First extract the factory calibration for each magnetometer axis
	uint8_t RawData[3];                            // x/y/z Mag calibration data stored here
	// Power down magnetometer



	WriteByte(MagI2cAddress, (uint8_t )AK8963_CNTL, 0x00);
	HAL_Delay(10);

	// Enter Fuse ROM access mode


	WriteByte(MagI2cAddress, (uint8_t )AK8963_CNTL, 0x0F);
	HAL_Delay(10);



	ReadBytes(MagI2cAddress,( uint8_t )AK8963_ASAX ,3 ,  RawData);

	// Read the x-, y-, and z-axis calibration values
	MagBiasFactory[0] = (float)(RawData[0] - 128) / 256. + 1.;  // Return x-axis sensitivity adjustment values, etc.
	MagBiasFactory[1] = (float)(RawData[1] - 128) / 256. + 1.;
	MagBiasFactory[2] = (float)(RawData[2] - 128) / 256. + 1.;


	// Power down magnetometer

	WriteByte(MagI2cAddress, (uint8_t )AK8963_CNTL, 0x00);
	HAL_Delay(10);


	// Configure the magnetometer for continuous read and highest resolution
	// set Mscale bit 4 to 1 (0) to enable 16 (14) bit resolution in CNTL register,
	// and enable continuous mode data acquisition MAG_MODE (bits [3:0]), 0010 for 8 Hz and 0110 for 100 Hz sample rates

	// Set magnetometer data resolution and sample ODR

	uint8_t D=0;
	D =( ((uint8_t)MPU9250Settings.MagOutputBits )<< 4 | MAG_MODE);


	WriteByte(MagI2cAddress, (uint8_t )AK8963_CNTL, D);
	HAL_Delay(10);

}

void ReadAccelGyro (int16_t* Destination)
{
	uint8_t RawData[14];

	// Read the 14 raw data registers into data array
	ReadBytes(MpuI2cAddress,( uint8_t )ACCEL_XOUT_H ,14,  RawData);


	// Turn the MSB and LSB into a signed 16-bit value
	Destination[0] = ((int16_t)RawData[0] << 8) | (int16_t)RawData[1];
	Destination[1] = ((int16_t)RawData[2] << 8) | (int16_t)RawData[3];
	Destination[2] = ((int16_t)RawData[4] << 8) | (int16_t)RawData[5];
	Destination[3] = ((int16_t)RawData[6] << 8) | (int16_t)RawData[7];
	Destination[4] = ((int16_t)RawData[8] << 8) | (int16_t)RawData[9];
	Destination[5] = ((int16_t)RawData[10] << 8) | (int16_t)RawData[11];
	Destination[6] = ((int16_t)RawData[12] << 8) | (int16_t)RawData[13];

}


void UpdateAccelGyro()
{
	int16_t RawAccGyroData[7];
	ReadAccelGyro(RawAccGyroData);


	// Now we'll calculate the accleration value into actual g's
	A[0] = (float)RawAccGyroData[0] * AccResolution;  // get actual g value, this depends on scale being set
	A[1] = (float)RawAccGyroData[1] * AccResolution;
	A[2] = (float)RawAccGyroData[2] * AccResolution;

	TemperatureCount = RawAccGyroData[3];                  // Read the adc values
	Temperature = ((float)TemperatureCount) / 333.87 + 21.0;  // Temperature in degrees Centigrade

	// Calculate the gyro value into actual degrees per second
	G[0] = (float)RawAccGyroData[4] * GyroResolution;  // get actual gyro value, this depends on scale being set
	G[1] = (float)RawAccGyroData[5] * GyroResolution;
	G[2] = (float)RawAccGyroData[6] * GyroResolution;
}


float GetMagResolution( MAG_OUTPUT_BITS MagOutputBits)
{
	switch (MagOutputBits)
	{
	// Possible magnetometer scales (and their register bit settings) are:
	// 14 bit resolution (0) and 16 bit resolution (1)
	// Proper scale to return milliGauss
	case M14BITS:
		return (float)(10.0 * 4912.0 / 8190.0);
	case M16BITS:
		return (float)(10.0 * 4912.0 / 32760.0);
	default:
		return 0.0;
	}
}
int  ReadMag (int16_t* Destination)
{


	uint8_t St1=0;
	uint8_t RegisterSt1 =AK8963_ST1 ;



	while(HAL_I2C_Master_Transmit(&hi2c1,(uint16_t)MagI2cAddress,&RegisterSt1,1,1000) != HAL_OK);
	while(HAL_I2C_Master_Receive(&hi2c1,(uint16_t)MagI2cAddress,&St1,1,1000) != HAL_OK);

	St1=ReadByte(MagI2cAddress, (uint8_t )AK8963_ST1);


	if (St1 & 0x01) {                                                    // wait for magnetometer data ready bit to be set
		uint8_t RawData[7];

		// x/y/z gyro register data, ST2 register stored here, must read ST2 at end of data acquisition


		ReadBytes(MagI2cAddress,( uint8_t )AK8963_XOUT_L ,7 ,  RawData);
		// Read the six raw data and ST2 registers sequentially into data array
		if (MAG_MODE == 0x02 || MAG_MODE == 0x04 || MAG_MODE == 0x06) {  // continuous or external trigger read mode
			if ((St1 & 0x02) != 0)                                       // check if data is not skipped
				return 0;                                            // this should be after data reading to clear DRDY register
		}

		uint8_t C = RawData[6];                                         // End data read by reading ST2 register
		if (!(C & 0x08)) {                                               // Check if magnetic sensor overflow set, if not then report data
			Destination[0] = ((int16_t)RawData[1] << 8) | RawData[0];  // Turn the MSB and LSB into a signed 16-bit value
			Destination[1] = ((int16_t)RawData[3] << 8) | RawData[2];  // Data stored as little Endian
			Destination[2] = ((int16_t)RawData[5] << 8) | RawData[4];
			return 1;
		}
	}

	return 0;

}

void UpdateMag() {
	int16_t MagCount[3] = {0, 0, 0};  // Stores the 16-bit signed magnetometer sensor output

	// Read the x/y/z adc values
	if ( ReadMag (MagCount)) {
		// Calculate the magnetometer values in milliGauss
		// Include factory calibration per data sheet and user environmental corrections
		// mag_bias is calcurated in 16BITS
		float BiasToCurrentBits = MagResolution /  GetMagResolution(M16BITS);


		M[0] = (float)(MagCount[0] * MagResolution * MagBiasFactory[0] - MagBias[0] * BiasToCurrentBits) * MagScale[0];  // get actual magnetometer value, this depends on scale being set
		M[1] = (float)(MagCount[1] * MagResolution * MagBiasFactory[1] - MagBias[1] * BiasToCurrentBits) * MagScale[1];
		M[2] = (float)(MagCount[2] * MagResolution * MagBiasFactory[2] - MagBias[2] * BiasToCurrentBits) * MagScale[2];
	}
}



float GetAccResolution(ACCEL_FS_SEL AccelFs)
{
	switch (AccelFs) {
	// Possible accelerometer scales (and their register bit settings) are:
	// 2 Gs (00), 4 Gs (01), 8 Gs (10), and 16 Gs  (11).
	// Here's a bit of an algorith to calculate DPS/(ADC tick) based on that 2-bit value:
	case A2G:
		return 2.0 / 32768.0;
	case A4G:
		return 4.0 / 32768.0;
	case A8G:
		return 8.0 / 32768.0;
	case A16G:
		return 16.0 / 32768.0;
	default:
		return 0.0;
	}
}

float GetGyroResolution ( GYRO_FS_SEL GyroFs)
{
	switch (GyroFs) {
	// Possible gyro scales (and their register bit settings) are:
	// 250 DPS (00), 500 DPS (01), 1000 DPS (10), and 2000 DPS  (11).
	// Here's a bit of an algorith to calculate DPS/(ADC tick) based on that 2-bit value:
	case G250DPS:
		return 250.0 / 32768.0;
	case G500DPS:
		return 500.0 / 32768.0;
	case G1000DPS:
		return 1000.0 / 32768.0;
	case G2000DPS:
		return 2000.0 / 32768.0;
	default:
		return 0.;
	}
}

//  Function which accumulates gyro and accelerometer data after device initialization. It calculates the average
//   of the at-rest readings and then loads the resulting offsets into accelerometer and gyro bias registers.
//   ACCEL_FS_SEL: 2g (maximum sensitivity)
//   GYRO_FS_SEL: 250dps (maximum sensitivity)
void CalibrateGyroAcc()
{
	SetAccGyroToCalibration();

	CollectAccGyroDataToCalibration(AccBias, GyroBias) ;
	WriteAccelOffset();
	WriteGyroOffset();
	HAL_Delay(100);
	InitMPU9250();
	HAL_Delay(1000);
}

void SetAccGyroToCalibration()
{





	// reset device
	// Write a one to bit 7 reset bit; toggle reset device



	WriteByte(MpuI2cAddress, (uint8_t )PWR_MGMT_1, 0x80);
	HAL_Delay(100);


	// get stable time source; Auto select clock source to be PLL gyroscope reference if ready
	// else use the internal oscillator, bits 2:0 = 001
	WriteByte(MpuI2cAddress, (uint8_t )PWR_MGMT_1, 0x01);
	HAL_Delay(100);



	WriteByte(MpuI2cAddress, (uint8_t )PWR_MGMT_2, 0x00);
	HAL_Delay(100);



	// Configure device for bias calculation

	// Disable all interrupts


	WriteByte(MpuI2cAddress, (uint8_t )INT_ENABLE, 0x00);




	// Disable FIFO


	WriteByte(MpuI2cAddress, (uint8_t )FIFO_EN, 0x00);


	// Turn on internal clock source


	WriteByte(MpuI2cAddress, (uint8_t )PWR_MGMT_1, 0x00);

	// Disable I2C master



	WriteByte(MpuI2cAddress, (uint8_t )I2C_MST_CTRL, 0x00);


	// Disable FIFO and I2C master modes


	WriteByte(MpuI2cAddress, (uint8_t )USER_CTRL, 0x00);
	// Reset FIFO and DMP

	WriteByte(MpuI2cAddress, (uint8_t )USER_CTRL, 0x0C);
	HAL_Delay(15);

	// Configure MPU6050 gyro and accelerometer for bias calculation

	// Set low-pass filter to 188 Hz


	WriteByte(MpuI2cAddress, (uint8_t )MPU_CONFIG, 0x01);

	// Set sample rate to 1 kHz


	WriteByte(MpuI2cAddress, (uint8_t )SMPLRT_DIV, 0x00);

	// Set gyro full-scale to 250 degrees per second, maximum sensitivity



	WriteByte(MpuI2cAddress, (uint8_t )GYRO_CONFIG, 0x00);


	// Set accelerometer full-scale to 2 g, maximum sensitivity



	WriteByte(MpuI2cAddress, (uint8_t )ACCEL_CONFIG, 0x00);

	// Configure FIFO to capture accelerometer and gyro data for bias calculation


	// Enable FIFO


	WriteByte(MpuI2cAddress, (uint8_t )USER_CTRL, 0x40);

	// Enable gyro and accelerometer sensors for FIFO  (max size 512 bytes in MPU-9150)



	WriteByte(MpuI2cAddress, (uint8_t )FIFO_EN, 0x78);

	// accumulate 40 samples in 40 milliseconds = 480 bytes
	HAL_Delay(40);
}

void CollectAccGyroDataToCalibration(float* ABias, float* GBias) {


	// At end of sample accumulation, turn off FIFO sensor read
	uint8_t Data[12];                                    // data array to hold accelerometer and gyro x, y, z, data
	WriteByte(MpuI2cAddress , (uint8_t ) FIFO_EN, 0x00);             // Disable gyro and accelerometer sensors for FIFO





	ReadBytes(MpuI2cAddress, (uint8_t )FIFO_COUNTH, 2, &Data[0]);  // read FIFO sample count
	uint16_t FifoCount = ((uint16_t)Data[0] << 8) | Data[1];
	uint16_t PacketCount = FifoCount / 12;  // How many sets of full gyro and accelerometer data for averaging

	for (uint16_t ii = 0; ii < PacketCount; ii++)
	{
		int16_t AccelTemp[3] = {0, 0, 0};
		int16_t GyroTemp[3]  = {0, 0, 0};


		// read data for averaging :

		ReadBytes(MpuI2cAddress, (uint8_t ) FIFO_R_W , 12, &Data[0]);

		// Form signed 16-bit integer for each sample in FIFO :

		AccelTemp[0] = (int16_t)(((int16_t)Data[0] << 8) | Data[1]);
		AccelTemp[1] = (int16_t)(((int16_t)Data[2] << 8) | Data[3]);
		AccelTemp[2] = (int16_t)(((int16_t)Data[4] << 8) | Data[5]);
		GyroTemp[0] = (int16_t)(((int16_t)Data[6] << 8) | Data[7]);
		GyroTemp[1] = (int16_t)(((int16_t)Data[8] << 8) | Data[9]);
		GyroTemp[2] = (int16_t)(((int16_t)Data[10] << 8) | Data[11]);

		// Sum individual signed 16-bit biases to get accumulated signed 32-bit biases:

		ABias[0] += (float)AccelTemp[0];
		ABias[1] += (float)AccelTemp[1];
		ABias[2] += (float)AccelTemp[2];
		GBias[0] += (float)GyroTemp[0];
		GBias[1] += (float)GyroTemp[1];
		GBias[2] += (float)GyroTemp[2];
	}

	// Normalize sums to get average count biases :
	ABias[0] /= (float)PacketCount;
	ABias[1] /= (float)PacketCount;
	ABias[2] /= (float)PacketCount;
	GBias[0] /= (float)PacketCount;
	GBias[1] /= (float)PacketCount;
	GBias[2] /= (float)PacketCount;


	// Remove gravity from the z-axis accelerometer bias calculation:

	if (ABias[2] > 0L)
	{
		ABias[2] -= (float)CalibAccSensitivity;
	}
	else
	{
		ABias[2] += (float)CalibAccSensitivity;
	}


}


void WriteAccelOffset()
{
	// Construct the accelerometer biases for push to the hardware accelerometer bias registers. These registers contain
	// factory trim values which must be added to the calculated accelerometer biases; on boot up these registers will hold
	// non-zero values. In addition, bit 0 of the lower byte must be preserved since it is used for temperature
	// compensation calculations. Accelerometer bias registers expect bias input as 2048 LSB per g, so that
	// the accelerometer biases calculated above must be divided by 8.

	uint8_t ReadData[2] = {0,0};

	// A place to hold the factory accelerometer trim biases :
	int16_t AccBiasReg[3] = {0, 0, 0};
	// Read factory accelerometer trim values :

	ReadBytes(MpuI2cAddress,(uint8_t) XA_OFFSET_H, 2, &ReadData[0]);
	AccBiasReg[0] = ((int16_t)ReadData[0] << 8) | ReadData[1];



	ReadBytes(MpuI2cAddress,(uint8_t) YA_OFFSET_H, 2, &ReadData[0]);
	AccBiasReg[1] = ((int16_t)ReadData[0] << 8) | ReadData[1] ;




	ReadBytes(MpuI2cAddress,(uint8_t) ZA_OFFSET_H, 2, &ReadData[0]);
	AccBiasReg[2] = ((int16_t)ReadData[0] << 8) | ReadData[1] ;



	// Define array to hold mask bit for each accelerometer bias axis :

	int16_t MaskBit[3] = {1, 1, 1};

	for (int i = 0; i < 3; i++)

	{

		if (AccBiasReg[i] % 2)
		{
			MaskBit[i] = 0;
		}

		// Subtract calculated averaged accelerometer bias scaled to 2048 LSB/g :
		AccBiasReg[i] -= (int16_t)AccBias[i] >> 3;

		if (MaskBit[i])
		{   // Preserve temperature compensation bit :
			AccBiasReg[i] = AccBiasReg[i] & ~MaskBit[i];
		}
		else
		{	  // Preserve temperature compensation bit :
			AccBiasReg[i] = AccBiasReg[i] | 0x0001;
		}
	}

	uint8_t WriteData[6] = {0,0,0,0,0,0};
	WriteData[0] = (AccBiasReg[0] >> 8) & 0xFF;
	WriteData[1] = (AccBiasReg[0]) & 0xFF;
	WriteData[2] = (AccBiasReg[1] >> 8) & 0xFF;
	WriteData[3] = (AccBiasReg[1]) & 0xFF;
	WriteData[4] = (AccBiasReg[2] >> 8) & 0xFF;
	WriteData[5] = (AccBiasReg[2]) & 0xFF;

	// Push accelerometer biases to hardware registers
	WriteByte(MpuI2cAddress, (uint8_t)XA_OFFSET_H, WriteData[0]);
	WriteByte(MpuI2cAddress,(uint8_t) XA_OFFSET_L, WriteData[1]);
	WriteByte(MpuI2cAddress,(uint8_t) YA_OFFSET_H, WriteData[2]);
	WriteByte(MpuI2cAddress,(uint8_t) YA_OFFSET_L, WriteData[3]);
	WriteByte(MpuI2cAddress,(uint8_t) ZA_OFFSET_H, WriteData[4]);
	WriteByte(MpuI2cAddress, (uint8_t)ZA_OFFSET_L, WriteData[5]);
}

void WriteGyroOffset()
{
	// Construct the gyro biases for push to the hardware gyro bias registers, which are reset to zero upon device startup
	uint8_t GyroOffsetData[6]= {0,0,0,0,0,0};
	GyroOffsetData[0] = (-(int16_t)GyroBias[0] / 4 >> 8) & 0xFF;  // Divide by 4 to get 32.9 LSB per deg/s to conform to expected bias input format
	GyroOffsetData[1] = (-(int16_t)GyroBias[0] / 4) & 0xFF;       // Biases are additive, so change sign on calculated average gyro biases
	GyroOffsetData[2] = (-(int16_t)GyroBias[1] / 4 >> 8) & 0xFF;
	GyroOffsetData[3] = (-(int16_t)GyroBias[1] / 4) & 0xFF;
	GyroOffsetData[4] = (-(int16_t)GyroBias[2] / 4 >> 8) & 0xFF;
	GyroOffsetData[5] = (-(int16_t)GyroBias[2] / 4) & 0xFF;

	// Push gyro biases to hardware registers
	WriteByte(MpuI2cAddress, (uint8_t)XG_OFFSET_H, GyroOffsetData[0]);
	WriteByte(MpuI2cAddress, (uint8_t)XG_OFFSET_L, GyroOffsetData[1]);
	WriteByte(MpuI2cAddress, (uint8_t)YG_OFFSET_H, GyroOffsetData[2]);
	WriteByte(MpuI2cAddress, (uint8_t)YG_OFFSET_L, GyroOffsetData[3]);
	WriteByte(MpuI2cAddress, (uint8_t)ZG_OFFSET_H, GyroOffsetData[4]);
	WriteByte(MpuI2cAddress, (uint8_t)ZG_OFFSET_L, GyroOffsetData[5]);
}


// mag calibration is executed in MAG_OUTPUT_BITS: 16BITS :
int CalibrateMag() {
	// set MAG_OUTPUT_BITS to maximum to calibrate

	MAG_OUTPUT_BITS MagOutputBitsCache ;
	MagOutputBitsCache = MPU9250Settings.MagOutputBits ;

	MPU9250Settings.MagOutputBits = M16BITS;
	InitAK8963();
	CollectMagDataToCalibration(MagBias, MagScale);



	// restore MAG_OUTPUT_BITS

	MPU9250Settings.MagOutputBits = MagOutputBitsCache;
	InitAK8963();

	return 1;
}

void CollectMagDataToCalibration(float* MBias, float* MScale) {

	// Mag Calibration: Wave device in a figure eight


	// shoot for ~fifteen seconds of mag data
	uint16_t SampleCount = 0;
	if (MAG_MODE == 0x02)
		SampleCount = 128;     // at 8 Hz ODR, new mag data is available every 125 ms
	else if (MAG_MODE == 0x06)  // in this library, fixed to 100Hz
		SampleCount = 1500;    // at 100 Hz ODR, new mag data is available every 10 ms

	int32_t Bias[3]  = {0, 0, 0};
	int32_t Scale[3]= {0, 0, 0};

	int16_t MagMax[3] = {-32767, -32767, -32767};
	int16_t MagMin[3] = {32767, 32767, 32767};




	int16_t MagTemp[3] = {0, 0, 0};


	for (uint16_t ii = 0; ii < SampleCount; ii++) {

		// Read the mag data
		ReadMag(MagTemp);


		for (int jj = 0; jj < 3; jj++)
		{
			if (MagTemp[jj] > MagMax[jj]) MagMax[jj] = MagTemp[jj];
			if (MagTemp[jj] < MagMin[jj]) MagMin[jj] = MagTemp[jj];
		}
		if (MAG_MODE == 0x02) HAL_Delay(135);  // at 8 Hz ODR, new mag data is available every 125 ms
		if (MAG_MODE == 0x06) HAL_Delay(12);   // at 100 Hz ODR, new mag data is available every 10 ms
	}



	// Get hard iron correction
	Bias[0] = (MagMax[0] + MagMin[0]) / 2;  // get average x mag bias in counts
	Bias[1] = (MagMax[1] + MagMin[1]) / 2;  // get average y mag bias in counts
	Bias[2] = (MagMax[2] + MagMin[2]) / 2;  // get average z mag bias in counts

	float BiasResolution = GetMagResolution( M16BITS );

	MBias[0] = (float)Bias[0] * BiasResolution * MagBiasFactory[0];  // save mag biases in G for main program
	MBias[1] = (float)Bias[1] * BiasResolution * MagBiasFactory[1];
	MBias[2] = (float)Bias[2] * BiasResolution * MagBiasFactory[2];

	// Get soft iron correction estimate
	//*** multiplication by mag_bias_factory added in accordance with the following comment
	//*** https://github.com/kriswiner/MPU9250/issues/456#issue-836657973
	Scale[0] = (float)(MagMax[0] - MagMin[0]) * MagBiasFactory[0] / 2;  // get average x axis max chord length in counts
	Scale[1] = (float)(MagMax[1] - MagMin[1]) * MagBiasFactory[1] / 2;  // get average y axis max chord length in counts
	Scale[2] = (float)(MagMax[2] - MagMin[2]) * MagBiasFactory[2] / 2;  // get average z axis max chord length in counts

	float Avg = Scale[0] + Scale[1] + Scale[2] ;
	Avg /= 3.0;

	MScale[0] = Avg / ((float)Scale[0]);
	MScale[1] = Avg / ((float)Scale[1]);
	MScale[2] = Avg / ((float)Scale[2]);
}

int IsConnected() {
	HasConnected = IsConnectedMPU9250() && IsConnectedAK8963() ;
	return HasConnected;
}

int IsConnectedMPU9250() {



	uint8_t Reception;
	Reception =  ReadByte( MpuI2cAddress, (uint8_t) WHO_AM_I_MPU9250);

	if  ( (Reception==MPU9250_WHOAMI_DEFAULT_VALUE)
			||(Reception==MPU9255_WHOAMI_DEFAULT_VALUE)
			||(Reception==MPU6500_WHOAMI_DEFAULT_VALUE))
		return 1;
	else
		return 0;




}

int IsConnectedAK8963()
{
	uint8_t BypassEnable = 0x02;
	HAL_I2C_Mem_Write(&hi2c1, 0xD0, 55, I2C_MEMADD_SIZE_8BIT, &BypassEnable,1,1000);



	uint8_t Reception;



	Reception=  ReadByte( MagI2cAddress, (uint8_t)AK8963_WHO_AM_I  );


	return(Reception==AK8963_WHOAMI_DEFAULT_VALUE);

}




void WriteByte(uint8_t DevAddress, uint8_t Register, uint8_t Data)

{
	uint8_t D[2];
	D[0]=Register;
	D[1]=Data;
	while ( HAL_I2C_Master_Transmit(&hi2c1,(uint16_t)DevAddress,(uint8_t *)D  , 2 , 100  ) != HAL_OK);


}

uint8_t  ReadByte(uint8_t DevAddress, uint8_t Register ) {
	uint8_t Data = 0;

	while(HAL_I2C_Master_Transmit(&hi2c1,(uint16_t)DevAddress,&Register,1,1000) != HAL_OK);
	while(HAL_I2C_Master_Receive(&hi2c1,(uint16_t)DevAddress,&Data,1,1000) != HAL_OK);


	return Data;
}

void ReadBytes(uint8_t DevAddress, uint8_t Register, uint8_t Count, uint8_t* Destination)

{

	//uint8_t Register=AK8963_ASAX;

	while(HAL_I2C_Master_Transmit(&hi2c1, (uint16_t) DevAddress,&Register,1,1000) != HAL_OK);
	while(HAL_I2C_Master_Receive(&hi2c1, (uint16_t) DevAddress, (uint8_t*)Destination,Count,1000) != HAL_OK);


}
void Update()
{


	UpdateAccelGyro();
	UpdateMag();



}
float GetAccX() { return A[0]; }
float GetAccY() { return A[1]; }
float GetAccZ() { return A[2];}
float GetGyroX(){ return G[0]; }
float GetGyroY()  { return G[1];}
float GetGyroZ()  { return G[2]; }
float GetMagX()  { return M[0]; }
float GetMagY()  { return M[1]; }
float GetMagZ()  { return M[2]; }



float GetAccBiasX()  { return AccBias[0]; }
float GetAccBiasY()  { return AccBias[1]; }
float GetAccBiasZ()  { return AccBias[2]; }
float GetGyroBiasX()  { return GyroBias[0]; }
float GetGyroBiasY()  { return GyroBias[1]; }
float GetGyroBiasZ()  { return GyroBias[2]; }
float GetMagBiasX()  { return MagBias[0]; }
float GetMagBiasY()  { return MagBias[1]; }
float GetMagBiasZ()  { return MagBias[2]; }
float GetMagScaleX()  { return MagScale[0]; }
float GetMagScaleY()  { return MagScale[1]; }
float GetMagScaleZ()  { return MagScale[2]; }

float GetRoll()  { return rpy[0]; }
float GetPitch()  { return rpy[1]; }
float GetYaw()  { return rpy[2]; }



void SetAccBias( float x,  float y,  float z) {
	AccBias[0] = x;
	AccBias[1] = y;
	AccBias[2] = z;
	WriteAccelOffset();
}
void SetGyroBias( float x,  float y,  float z) {
	GyroBias[0] = x;
	GyroBias[1] = y;
	GyroBias[2] = z;
	WriteGyroOffset();
}
void SetMagBias( float x,  float y,  float z) {
	MagBias[0] = x;
	MagBias[1] = y;
	MagBias[2] = z;
}
void SetMagScale( float x,  float y,  float z) {
	MagScale[0] = x;
	MagScale[1] = y;
	MagScale[2] = z;
}
void SetMagneticDeclination(  float d) { MagneticDeclination = d; }

void UpdateRpy() {
	// Define output variables from updated quaternion---these are Tait-Bryan angles, commonly used in aircraft orientation.
	// In this coordinate system, the positive z-axis is down toward Earth.
	// Yaw is the angle between Sensor x-axis and Earth magnetic North (or true North if corrected for local declination, looking down on the sensor positive yaw is counterclockwise.
			// Pitch is angle between sensor x-axis and Earth ground plane, toward the Earth is positive, up toward the sky is negative.
	// Roll is angle between sensor y-axis and Earth ground plane, y-axis up is positive roll.
	// These arise from the definition of the homogeneous rotation matrix constructed from quaternions.
	// Tait-Bryan angles as well as Euler angles are non-commutative; that is, the get the correct orientation the rotations must be
	// applied in the correct order which for this configuration is yaw, pitch, and then roll.
	// For more see http://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles which has additional links.
	UpdateQuaternion(A,G,M,Q);

	float qw, qx,qy,qz ;
	qw=Q[0];
	qx=Q[1];
	qy=Q[2];
	qz=Q[3];

	float a12, a22, a31, a32, a33;  // rotation matrix coefficients for Euler angles and gravity components
	a12 = 2.0f * (qx * qy + qw * qz);
	a22 = qw * qw + qx * qx - qy * qy - qz * qz;
	a31 = 2.0f * (qw * qx + qy * qz);
	a32 = 2.0f * (qx * qz - qw * qy);
	a33 = qw * qw - qx * qx - qy * qy + qz * qz;
	rpy[0] = atan2f(a31, a33);
	rpy[1] = -asinf(a32);
	rpy[2] = atan2f(a12, a22);
	rpy[0] *= 180.0f / M_PI;
	rpy[1] *= 180.0f / M_PI;
	rpy[2] *= 180.0f / M_PI;
	rpy[2] += MagneticDeclination;
	if (rpy[2] >= +180.f)
		rpy[2] -= 360.f;
	else if (rpy[2] < -180.f)
		rpy[2] += 360.f;
	/*
                lin_acc[0] = a[0] + a31;
                lin_acc[1] = a[1] + a32;
                lin_acc[2] = a[2] - a33;*/
}




//




//
//
//void I2C_Init()
//{
//	  __HAL_RCC_SYSCFG_CLK_ENABLE();
//	  __HAL_RCC_PWR_CLK_ENABLE();
//
//	  hi2c1.Instance = I2C1;
//	  __HAL_RCC_GPIOB_CLK_ENABLE();
//
//	  GPIO_InitTypeDef GPIO_InitStruct = {0};
//	  if(hi2c1.Instance==I2C1)
//	  {
//	  /* USER CODE BEGIN I2C1_MspInit 0 */
//
//	  /* USER CODE END I2C1_MspInit 0 */
//
//	    __HAL_RCC_GPIOB_CLK_ENABLE();
//	    /**I2C1 GPIO Configuration
//	    PB6     ------> I2C1_SCL
//	    PB7     ------> I2C1_SDA
//	    */
//	    GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7;
//	    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
//	    GPIO_InitStruct.Pull = GPIO_NOPULL;
//	    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
//	    GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
//	    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
//
//	    /* Peripheral clock enable */
//	    __HAL_RCC_I2C1_CLK_ENABLE();
//	  /* USER CODE BEGIN I2C1_MspInit 1 */
//
//	  /* USER CODE END I2C1_MspInit 1 */
//	  }
//
//	 // hi2c1.Instance = I2C1;
//	  hi2c1.Init.ClockSpeed = 100000;
//	  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
//	  hi2c1.Init.OwnAddress1 = 0;
//	  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
//	  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
//	  hi2c1.Init.OwnAddress2 = 0;
//	  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
//	  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
//	  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
//	  {
//		  while (1)
//		  {
//		  }
//	  }
//
//
//}

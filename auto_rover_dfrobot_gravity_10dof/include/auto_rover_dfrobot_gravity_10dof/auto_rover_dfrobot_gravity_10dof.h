/*
 * MIT License
 *
 * Copyright (c) 2019 Donald R. Poole, Jr.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in al
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#include <iostream>
#include <string>
#include <vector>
#include <ros/ros.h>
#include "auto_rover_dfrobot_gravity_10dof/I2C.h"

#define BMP280_CALIB_SIZE 24
#define BIAS_PARAM_SIZE   3

// BMP280 registers
#define BMP280_TEMP_XLSB  0xFC
#define BMP280_TEMP_LSB   0xFB
#define BMP280_TEMP_MSB   0xFA
#define BMP280_PRESS_XLSB 0xF9
#define BMP280_PRESS_LSB  0xF8
#define BMP280_PRESS_MSB  0xF7
#define BMP280_CONFIG     0xF5
#define BMP280_CTRL_MEAS  0xF4
#define BMP280_STATUS     0xF3
#define BMP280_RESET      0xE0
#define BMP280_ID         0xD0  // should be 0x58
#define BMP280_CALIB00    0x88

// BNO055 Register Map
// http://ae-bst.resource.bosch.com/media/products/dokumente/bno055/BST_BNO055_DS000_10_Release.pdf
//
// BNO055 Page 0
#define BNO055_CHIP_ID          0x00    // should be 0xA0
#define BNO055_ACC_ID           0x01    // should be 0xFB
#define BNO055_MAG_ID           0x02    // should be 0x32
#define BNO055_GYRO_ID          0x03    // should be 0x0F
#define BNO055_SW_REV_ID_LSB    0x04
#define BNO055_SW_REV_ID_MSB    0x05
#define BNO055_BL_REV_ID        0x06
#define BNO055_PAGE_ID          0x07
#define BNO055_ACC_DATA_X_LSB   0x08
#define BNO055_ACC_DATA_X_MSB   0x09
#define BNO055_ACC_DATA_Y_LSB   0x0A
#define BNO055_ACC_DATA_Y_MSB   0x0B
#define BNO055_ACC_DATA_Z_LSB   0x0C
#define BNO055_ACC_DATA_Z_MSB   0x0D
#define BNO055_MAG_DATA_X_LSB   0x0E
#define BNO055_MAG_DATA_X_MSB   0x0F
#define BNO055_MAG_DATA_Y_LSB   0x10
#define BNO055_MAG_DATA_Y_MSB   0x11
#define BNO055_MAG_DATA_Z_LSB   0x12
#define BNO055_MAG_DATA_Z_MSB   0x13
#define BNO055_GYR_DATA_X_LSB   0x14
#define BNO055_GYR_DATA_X_MSB   0x15
#define BNO055_GYR_DATA_Y_LSB   0x16
#define BNO055_GYR_DATA_Y_MSB   0x17
#define BNO055_GYR_DATA_Z_LSB   0x18
#define BNO055_GYR_DATA_Z_MSB   0x19
#define BNO055_EUL_HEADING_LSB  0x1A
#define BNO055_EUL_HEADING_MSB  0x1B
#define BNO055_EUL_ROLL_LSB     0x1C
#define BNO055_EUL_ROLL_MSB     0x1D
#define BNO055_EUL_PITCH_LSB    0x1E
#define BNO055_EUL_PITCH_MSB    0x1F
#define BNO055_QUA_DATA_W_LSB   0x20
#define BNO055_QUA_DATA_W_MSB   0x21
#define BNO055_QUA_DATA_X_LSB   0x22
#define BNO055_QUA_DATA_X_MSB   0x23
#define BNO055_QUA_DATA_Y_LSB   0x24
#define BNO055_QUA_DATA_Y_MSB   0x25
#define BNO055_QUA_DATA_Z_LSB   0x26
#define BNO055_QUA_DATA_Z_MSB   0x27
#define BNO055_LIA_DATA_X_LSB   0x28
#define BNO055_LIA_DATA_X_MSB   0x29
#define BNO055_LIA_DATA_Y_LSB   0x2A
#define BNO055_LIA_DATA_Y_MSB   0x2B
#define BNO055_LIA_DATA_Z_LSB   0x2C
#define BNO055_LIA_DATA_Z_MSB   0x2D
#define BNO055_GRV_DATA_X_LSB   0x2E
#define BNO055_GRV_DATA_X_MSB   0x2F
#define BNO055_GRV_DATA_Y_LSB   0x30
#define BNO055_GRV_DATA_Y_MSB   0x31
#define BNO055_GRV_DATA_Z_LSB   0x32
#define BNO055_GRV_DATA_Z_MSB   0x33
#define BNO055_TEMP             0x34
#define BNO055_CALIB_STAT       0x35
#define BNO055_ST_RESULT        0x36
#define BNO055_INT_STATUS       0x37
#define BNO055_SYS_CLK_STATUS   0x38
#define BNO055_SYS_STATUS       0x39
#define BNO055_SYS_ERR          0x3A
#define BNO055_UNIT_SEL         0x3B
#define BNO055_OPR_MODE         0x3D
#define BNO055_PWR_MODE         0x3E
#define BNO055_SYS_TRIGGER      0x3F
#define BNO055_TEMP_SOURCE      0x40
#define BNO055_AXIS_MAP_CONFIG  0x41
#define BNO055_AXIS_MAP_SIGN    0x42
#define BNO055_ACC_OFFSET_X_LSB 0x55
#define BNO055_ACC_OFFSET_X_MSB 0x56
#define BNO055_ACC_OFFSET_Y_LSB 0x57
#define BNO055_ACC_OFFSET_Y_MSB 0x58
#define BNO055_ACC_OFFSET_Z_LSB 0x59
#define BNO055_ACC_OFFSET_Z_MSB 0x5A
#define BNO055_MAG_OFFSET_X_LSB 0x5B
#define BNO055_MAG_OFFSET_X_MSB 0x5C
#define BNO055_MAG_OFFSET_Y_LSB 0x5D
#define BNO055_MAG_OFFSET_Y_MSB 0x5E
#define BNO055_MAG_OFFSET_Z_LSB 0x5F
#define BNO055_MAG_OFFSET_Z_MSB 0x60
#define BNO055_GYR_OFFSET_X_LSB 0x61
#define BNO055_GYR_OFFSET_X_MSB 0x62
#define BNO055_GYR_OFFSET_Y_LSB 0x63
#define BNO055_GYR_OFFSET_Y_MSB 0x64
#define BNO055_GYR_OFFSET_Z_LSB 0x65
#define BNO055_GYR_OFFSET_Z_MSB 0x66
#define BNO055_ACC_RADIUS_LSB   0x67
#define BNO055_ACC_RADIUS_MSB   0x68
#define BNO055_MAG_RADIUS_LSB   0x69
#define BNO055_MAG_RADIUS_MSB   0x6A
//
// BNO055 Page 1
#define BNO055_PAGE_ID          0x07
#define BNO055_ACC_CONFIG       0x08
#define BNO055_MAG_CONFIG       0x09
#define BNO055_GYRO_CONFIG_0    0x0A
#define BNO055_GYRO_CONFIG_1    0x0B
#define BNO055_ACC_SLEEP_CONFIG 0x0C
#define BNO055_GYR_SLEEP_CONFIG 0x0D
#define BNO055_INT_MSK          0x0F
#define BNO055_INT_EN           0x10
#define BNO055_ACC_AM_THRES     0x11
#define BNO055_ACC_INT_SETTINGS 0x12
#define BNO055_ACC_HG_DURATION  0x13
#define BNO055_ACC_HG_THRESH    0x14
#define BNO055_ACC_NM_THRESH    0x15
#define BNO055_ACC_NM_SET       0x16
#define BNO055_GYR_INT_SETTINGS 0x17
#define BNO055_GYR_HR_X_SET     0x18
#define BNO055_GYR_DUR_X        0x19
#define BNO055_GYR_HR_Y_SET     0x1A
#define BNO055_GYR_DUR_Y        0x1B
#define BNO055_GYR_HR_Z_SET     0x1C
#define BNO055_GYR_DUR_Z        0x1D
#define BNO055_GYR_AM_THRESH    0x1E
#define BNO055_GYR_AM_SET       0x1F


// Using the BNO055_BMP280 breakout board/Teensy 3.1 Add-On Shield, ADO is set to 1 by default
#define ADO 1
#if ADO
#define BNO055_ADDRESS 0x29   //  Device address of BNO055 when ADO = 1
#define BMP280_ADDRESS 0x77   // Address of BMP280
#else
#define BNO055_ADDRESS 0x28   //  Device address of BNO055 when ADO = 0
#define BMP280_ADDRESS 0x77   // Address of BMP280
#endif


namespace auto_rover_dfrobot_gravity_10dof
{
	// Set initial input parameters
	enum Ascale
	{  // ACC Full Scale
		AFS_2G = 0,
		AFS_4G,
		AFS_8G,
		AFS_18G
	};

	enum Abw
	{ // ACC Bandwidth
		ABW_7_81Hz = 0,
		ABW_15_63Hz,
		ABW_31_25Hz,
		ABW_62_5Hz,
		ABW_125Hz,
		ABW_250Hz,
		ABW_500Hz,
		ABW_1000Hz,    //0x07
	};

	enum APwrMode
	{ // ACC Pwr Mode
		NormalA = 0,
		SuspendA,
		LowPower1A,
		StandbyA,
		LowPower2A,
		DeepSuspendA
	};

	enum Gscale
	{  // gyro full scale
		GFS_2000DPS = 0,
		GFS_1000DPS,
		GFS_500DPS,
		GFS_250DPS,
		GFS_125DPS    // 0x04
	};

	enum GPwrMode
	{ // GYR Pwr Mode
		NormalG = 0,
		FastPowerUpG,
		DeepSuspendedG,
		SuspendG,
		AdvancedPowerSaveG
	};

	enum Gbw
	{ // gyro bandwidth
		GBW_523Hz = 0,
		GBW_230Hz,
		GBW_116Hz,
		GBW_47Hz,
		GBW_23Hz,
		GBW_12Hz,
		GBW_64Hz,
		GBW_32Hz
	};

	enum OPRMode
	{  // BNO-55 operation modes
		CONFIGMODE = 0x00,

		// Sensor Mode
		ACCONLY,
		MAGONLY,
		GYROONLY,
		ACCMAG,
		ACCGYRO,
		MAGGYRO,
		AMG,            // 0x07

		// Fusion Mode
		IMU,
		COMPASS,
		M4G,
		NDOF_FMC_OFF,
		NDOF            // 0x0C
	};

	enum PWRMode
	{
		Normalpwr = 0,
		Lowpower,
		Suspendpwr
	};

	enum Modr
	{         // magnetometer output data rate
		MODR_2Hz = 0,
		MODR_6Hz,
		MODR_8Hz,
		MODR_10Hz,
		MODR_15Hz,
		MODR_20Hz,
		MODR_25Hz,
		MODR_30Hz
	};

	enum MOpMode
	{ // MAG Op Mode
		LowPower = 0,
		Regular,
		EnhancedRegular,
		HighAccuracy
	};

	enum MPwrMode
	{ // MAG power mode
		Normal = 0,
		Sleep,
		Suspend,
		ForceMode
	};

	enum Posr
	{
		P_OSR_00 = 0,  // no op
		P_OSR_01,
		P_OSR_02,
		P_OSR_04,
		P_OSR_08,
		P_OSR_16
	};

	enum Tosr
	{
		T_OSR_00 = 0,  // no op
		T_OSR_01,
		T_OSR_02,
		T_OSR_04,
		T_OSR_08,
		T_OSR_16
	};

	enum IIRFilter
	{
		full = 0,  // bandwidth at full sample rate
		BW0_223ODR,
		BW0_092ODR,
		BW0_042ODR,
		BW0_021ODR // bandwidth at 0.021 x sample rate
	};

	enum Mode
	{
		BMP280Sleep = 0,
		forced,
		forced2,
		normal
	};

	enum SBy
	{
		t_00_5ms = 0,
		t_62_5ms,
		t_125ms,
		t_250ms,
		t_500ms,
		t_1000ms,
		t_2000ms,
		t_4000ms,
	};

	typedef struct _DFRobotGravity10DoFConfig
	{
		// Specify BMP280 configuration
		uint8_t p_osr = P_OSR_16;
		uint8_t t_osr = T_OSR_01;
		uint8_t mode = normal;
		uint8_t iir_filter = BW0_042ODR;
		uint8_t sby = t_62_5ms;          // Set pressure and temperature output data rate

		// t_fine carries fine temperature as global value for BMP280
		int32_t t_fine;

		uint8_t g_pwr_mode = NormalG;   // Gyro power mode
		uint8_t g_scale = GFS_250DPS;   // Gyro full scale
		//uint8_t Godr = GODR_250Hz;    // Gyro sample rate
		uint8_t g_bw = GBW_23Hz;        // Gyro bandwidth

		uint8_t a_scale = AFS_2G;       // Accel full scale
		//uint8_t Aodr = AODR_250Hz;    // Accel sample rate
		uint8_t a_pwr_mode = NormalA;    // Accel power mode
		uint8_t a_bw = ABW_31_25Hz;     // Accel bandwidth, accel sample rate divided by ABW_divx

		//uint8_t Mscale = MFS_4Gauss;  // Select magnetometer full-scale resolution
		uint8_t m_op_mode = Regular;    // Select magnetometer perfomance mode
		uint8_t m_pwr_mode = Normal;    // Select magnetometer power mode
		uint8_t m_odr = MODR_10Hz;      // Select magnetometer ODR when in BNO055 bypass mode

		uint8_t pwr_mode = Normalpwr;   // Select BNO055 power mode
		uint8_t opr_mode = NDOF;        // specify operation mode for sensors
		uint8_t status;                 // BNO055 data status register
		double a_res;
		double g_res;
		double m_res;                   // scale resolutions per LSB for the sensors
	} DFRobotGravity10DoFConfig;

	typedef struct _BMP280CompensationParams
	{
		uint16_t dig_t1;
		uint16_t dig_p1;
		int16_t dig_t2;
		int16_t dig_t3;
		int16_t dig_p2;
		int16_t dig_p3;
		int16_t dig_p4;
		int16_t dig_p5;
		int16_t dig_p6;
		int16_t dig_p7;
		int16_t dig_p8;
		int16_t dig_p9;
	} BMP280CompensationParams;

	class AutoRoverDFRobotGravity10DoF
	{
	public:
		explicit AutoRoverDFRobotGravity10DoF(const uint8_t &bus, const uint8_t& device);
		~AutoRoverDFRobotGravity10DoF();

		void Initialize();

		void SetConfiguration(DFRobotGravity10DoFConfig &config);

		bool InitBNO055();

		bool InitBMP280();

		bool AccelGyroCalBNO055();

		bool MagCalBNO055();

		int32_t ReadBMP280Temperature();

		int32_t ReadBMP280Pressure();

		uint32_t CompensatePressure(int32_t& adc);

		uint32_t CompensateTemperature(int32_t& adc);

		void ReadAccelData(vector<int16_t>& destination);

		void ReadGyroData(vector<int16_t>& destination);

		int8_t ReadGyroTempData();

		void ReadMagData(vector<int16_t>& destination);

		void ReadQuatData(vector<int16_t>& destination);

		void ReadEulData(vector<int16_t>& destination);

		void ReadLIAData(vector<int16_t>& destination);

		void ReadGRVData(vector<int16_t>& destination);

	private:
		I2C i2c_;
		vector<uint8_t> bmp_calib_;
		vector<float> accel_bias_;
		vector<float> gyro_bias_;
		vector<float> mag_bias_;
		DFRobotGravity10DoFConfig config_;
		BMP280CompensationParams bmp_comp_params_;
	};
}

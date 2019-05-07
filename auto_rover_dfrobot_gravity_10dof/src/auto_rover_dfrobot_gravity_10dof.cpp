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

#include <thread>
#include <chrono>
#include <ros/console.h>
#include <auto_rover_dfrobot_gravity_10dof/auto_rover_dfrobot_gravity_10dof.h>

#define BUFFSIZE 120

using namespace auto_rover_dfrobot_gravity_10dof;

AutoRoverDFRobotGravity10DoF::AutoRoverDFRobotGravity10DoF(const uint8_t &bus, const uint8_t &device) :
	i2c_(bus, device),
	bmp_calib_(BMP280_CALIB_SIZE, 0x00),
	accel_bias_(BIAS_PARAM_SIZE, 0.0),
	gyro_bias_(BIAS_PARAM_SIZE, 0.0),
	mag_bias_(BIAS_PARAM_SIZE, 0.0)
{
}

AutoRoverDFRobotGravity10DoF::~AutoRoverDFRobotGravity10DoF()
{
}

void AutoRoverDFRobotGravity10DoF::SetConfiguration(DFRobotGravity10DoFConfig& config)
{
    config_ = config;
}

bool AutoRoverDFRobotGravity10DoF::Initialize()
{
	uint8_t self_test;
	bool retval = false;

    ROS_INFO("Checking self-test results");

    if ( i2c_.ReadByte(BNO055_ST_RESULT, self_test) )
    {
	    if (self_test & 0x01)
	    {
		    ROS_INFO("Accelerometer passed self-test");
		    retval = true;
	    }
	    else
	    {
		    ROS_ERROR("Accelerometer failed self-test");
		    retval = false;
	    }

	    if (self_test & 0x02)
	    {
		    ROS_INFO("Magnetometer passed self-test");
		    retval = true;
	    }
	    else
	    {
		    ROS_ERROR("Magnetometer failed self-test");
		    retval = false;
	    }

	    if (self_test & 0x04)
	    {
		    ROS_INFO("Gyroscope passed self-test");
		    retval = true;
	    }
	    else
	    {
		    ROS_ERROR("Gyroscope failed self-test");
		    retval = false;
	    }

	    if (self_test & 0x08)
	    {
		    ROS_INFO("MCU passed self-test");
		    retval = true;
	    }
	    else
	    {
		    ROS_ERROR("MCU failed self-test");
		    retval = false;
	    }
    }
    else
    {
    	ROS_ERROR("Failed to execute device self-test");
	    retval = false;
    }


    return retval;
}

bool AutoRoverDFRobotGravity10DoF::InitBNO055()
{
    uint8_t data;

    if ( !i2c_.IsConnected() )
    {
        ROS_ERROR("Not connected to Gravity 10DoF. Aborting BNO055 initialization.");
        return false;
    }

    ROS_INFO("Initializing BNO055");

    // Select BNO055 config mode
    if ( !i2c_.WriteByte(BNO055_OPR_MODE, CONFIGMODE) )
    {
        ROS_ERROR("Failed to set BNO055 config mode.");
        return false;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(25));

    // Select page 1 to configure sensors
    if ( !i2c_.WriteByte(BNO055_PAGE_ID, 0x01) )
    {
        ROS_ERROR("Failed to select page 1 to configure sensors");
        return false;
    }

    // Configure ACC
    data = config_.a_pwr_mode << 5 | config_.a_bw << 2 | config_.a_scale;
    if ( !i2c_.WriteByte(BNO055_ACC_CONFIG, data) )
    {
        ROS_ERROR("Failed to configure accelerometer");
        return false;
    }

    // Configure GYR
    data = config_.g_bw << 3 | config_.g_scale;
    if ( !i2c_.WriteByte(BNO055_GYRO_CONFIG_0, data) )
    {
        ROS_ERROR("Failed to configure gyroscope");
        return false;
    }
    if ( !i2c_.WriteByte(BNO055_GYRO_CONFIG_1, config_.g_pwr_mode) )
    {
        ROS_ERROR("Failed to configure gyroscope");
        return false;
    }

    // Configure MAG
    data = config_.m_pwr_mode << 5 | config_.m_op_mode << 3 | config_.m_odr;
    if ( !i2c_.WriteByte(BNO055_MAG_CONFIG, data) )
    {
        ROS_ERROR("Failed to configure magnetometer");
        return false;
    }

    // Select page 0 to read sensors
    if ( !i2c_.WriteByte(BNO055_PAGE_ID, 0x00) )
    {
        ROS_ERROR("Failed to select page 0 to read sensors");
        return false;
    }

    // Select BNO055 gyro temperature source 
    if ( !i2c_.WriteByte(BNO055_TEMP_SOURCE, 0x01 ) )
    {
        ROS_ERROR("Failed to select gyroscope temperature source");
        return false;
    }

    // Select BNO055 sensor units (temperature in degrees C, rate in dps, accel in mg)
    if ( !i2c_.WriteByte(BNO055_UNIT_SEL, 0x01 ) )
    {
        ROS_ERROR("Failed to set BNO055 sensor units to degrees C, rate in dps and accel in mg");
        return false;
    }

    // Select BNO055 system power mode
    if ( !i2c_.WriteByte(BNO055_PWR_MODE, config_.pwr_mode ) )
    {
        ROS_ERROR("Failed to set BNO055 system power mode");
        return false;
    }

    // Select BNO055 system operation mode
    if ( !i2c_.WriteByte(BNO055_OPR_MODE, config_.opr_mode) )
    {
        ROS_ERROR("Failed to set BNO055 operation mode.");
        return false;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(25));

    ROS_INFO("BNO055 initialization complet!");

    return 0;
}

bool AutoRoverDFRobotGravity10DoF::InitBMP280()
{
    uint8_t data;

    if ( !i2c_.IsConnected() )
    {
        ROS_ERROR("Not connected to Gravity 10DoF. Aborting BMP280 initialization.");
        return false;
    }

    ROS_INFO("Initializing BMP280 altimeter and pressure sensor");

    // Reset before initialization
    if ( !i2c_.WriteByte(BMP280_RESET, 0xB6) )
    {
        ROS_ERROR("Failed to reset BMP280");
        return false;
    }

    // Set T and P oversampling rates and sensor mode
    data = config_.t_osr << 5 | config_.p_osr << 2 | config_.mode;
    if ( !i2c_.WriteByte(BMP280_CTRL_MEAS, data) )
    {
        ROS_ERROR("Failed to set BMP280 oversampling rates and sensor mode");
        return false;
    }

    // Set standby time interval in normal mode and bandwidth
    data = config_.sby << 5 | config_.iir_filter << 2;
    if ( !i2c_.WriteByte(BMP280_CONFIG, data) )
    {
        ROS_ERROR("Failed to set BMP280 standby time interval for normal mode and bandwidth");
        return false;
    }

    // Read and store calibration data
    if ( i2c_.ReadBytes(BMP280_CALIB00, bmp_calib_, BMP280_CALIB_SIZE) )
    {
        ROS_ERROR("Failed to calibrate the BMP280");
        return false;
    }

    bmp_comp_params_.dig_t1 = (uint16_t)(((uint16_t) bmp_calib_[1] << 8) | bmp_calib_[0]);
    bmp_comp_params_.dig_t2 = ( int16_t)((( int16_t) bmp_calib_[3] << 8) | bmp_calib_[2]);
    bmp_comp_params_.dig_t3 = ( int16_t)((( int16_t) bmp_calib_[5] << 8) | bmp_calib_[4]);
    bmp_comp_params_.dig_p1 = (uint16_t)(((uint16_t) bmp_calib_[7] << 8) | bmp_calib_[6]);
    bmp_comp_params_.dig_p2 = ( int16_t)((( int16_t) bmp_calib_[9] << 8) | bmp_calib_[8]);
    bmp_comp_params_.dig_p3 = ( int16_t)((( int16_t) bmp_calib_[11] << 8) | bmp_calib_[10]);
    bmp_comp_params_.dig_p4 = ( int16_t)((( int16_t) bmp_calib_[13] << 8) | bmp_calib_[12]);
    bmp_comp_params_.dig_p5 = ( int16_t)((( int16_t) bmp_calib_[15] << 8) | bmp_calib_[14]);
    bmp_comp_params_.dig_p6 = ( int16_t)((( int16_t) bmp_calib_[17] << 8) | bmp_calib_[16]);
    bmp_comp_params_.dig_p7 = ( int16_t)((( int16_t) bmp_calib_[19] << 8) | bmp_calib_[18]);
    bmp_comp_params_.dig_p8 = ( int16_t)((( int16_t) bmp_calib_[21] << 8) | bmp_calib_[20]);
    bmp_comp_params_.dig_p9 = ( int16_t)((( int16_t) bmp_calib_[23] << 8) | bmp_calib_[22]);

    ROS_INFO("BMP280 initialization complete!");

    return 0;
}

bool AutoRoverDFRobotGravity10DoF::AccelGyroCalBNO055()
{
    uint8_t tmp;
    vector<uint8_t> data(6, 0x00); // Temp array for accelerometer & gyro x, y, z data
    vector<int32_t> bias(3, 0);
    uint16_t sample_cnt = 256;

    ROS_INFO("Accel/Gyro Calibration: Put device on a level surface and keep motionless! Wait...");
    this_thread::sleep_for(chrono::milliseconds(4000));

    if ( !i2c_.WriteByte(BNO055_PAGE_ID, 0x00) )
    {
        ROS_ERROR("Failed to select page 0 to read sensors");
        return false;
    }

    if ( !i2c_.WriteByte(BNO055_OPR_MODE, config_.mode) )
    {
        ROS_ERROR("Failed to set BNO055 into config mode");
	    return false;
    }
    this_thread::sleep_for(chrono::milliseconds(25));


    if ( !i2c_.WriteByte(BNO055_OPR_MODE, AMG) )
    {
        ROS_ERROR("Failed to set BNO055 operation mode to Accelerometer, Magnetometer, Gyroscope");
	    return false;
    }

    //
	// In NDF fusion mode, accel full scale is at +/- 4g, ODR is 62.5 Hz, set it the same here
	//
    tmp = config_.a_pwr_mode << 5 | config_.a_bw << 2 | AFS_4G;
    if ( !i2c_.WriteByte(BNO055_ACC_CONFIG, tmp) )
    {
        ROS_ERROR("Failed to configure accelerometer");
	    return false;
    }

    vector<int16_t> temp(3, 0x00);
    for (int i = 0; i < sample_cnt; i++)
    {
    	if ( !i2c_.ReadBytes(BNO055_ACC_DATA_X_LSB, data, data.size()) )
	    {
		    ROS_WARN("Multi-byte read failed during accelerometer calibration. Results may be skewed");
	    }
	    else
	    {
	    	temp[0] = static_cast<int16_t>(data[1] << 8) | data[0];
		    temp[1] = static_cast<int16_t>(data[3] << 8) | data[2];
		    temp[2] = static_cast<int16_t>(data[5] << 8) | data[4];

		    bias[0] += static_cast<int32_t>(temp[0]);
		    bias[1] += static_cast<int32_t>(temp[1]);
		    bias[2] += static_cast<int32_t>(temp[2]);

		    this_thread::sleep_for(chrono::milliseconds(20)); // at 62.5 Hz ODR, new accel data is available every 16 ms
	    }
    }

    // Get average accel bias in mg
    bias[0] /= static_cast<int32_t>(sample_cnt);
	bias[1] /= static_cast<int32_t>(sample_cnt);
	bias[2] /= static_cast<int32_t>(sample_cnt);

	// Remove gravity from z
	if (bias[2] > 0L)
		bias[2] -= 1000;
	else
		bias[2] += 1000;

	// Save accel bias
	accel_bias_[0] = static_cast<float>(bias[0]);
	accel_bias_[1] = static_cast<float>(bias[1]);
	accel_bias_[2] = static_cast<float>(bias[2]);

	//
	// In NDF fusion mode, gyro full scale is at +/- 2000 dps, ODR is 32 Hz
	//
	if ( !i2c_.WriteByte(BNO055_GYRO_CONFIG_0, config_.g_bw << 3 | GFS_2000DPS) )
	{
		ROS_ERROR("Failed to configure gyro");
		return false;
	}

	if ( !i2c_.WriteByte(BNO055_GYRO_CONFIG_1, config_.g_pwr_mode) )
	{
		ROS_ERROR("Failed to configure gyro");
		return false;
	}

	temp.clear(); data.clear(); bias.clear();
	temp.resize(3, 0x00); data.resize(6, 0x00); bias.resize(3, 0);
	for (int i = 0; i < sample_cnt; i++)
	{
		if ( !i2c_.ReadBytes(BNO055_GYR_DATA_X_LSB, data, data.size()) )
		{
			ROS_WARN("Multi-byte read failed during gyroscope calibration. Results may be skewed");
		}
		else
		{
			temp[0] = static_cast<int16_t>(data[1] << 8) | data[0];
			temp[1] = static_cast<int16_t>(data[3] << 8) | data[2];
			temp[2] = static_cast<int16_t>(data[5] << 8) | data[4];

			bias[0] += static_cast<int32_t>(temp[0]);
			bias[1] += static_cast<int32_t>(temp[1]);
			bias[2] += static_cast<int32_t>(temp[2]);

			this_thread::sleep_for(chrono::milliseconds(35)); // at 32 Hz ODR, new gyro data available every 31 ms
		}
	}

	// Get average gyro bias in counts
	bias[0] /= static_cast<int32_t>(sample_cnt);
	bias[1] /= static_cast<int32_t>(sample_cnt);
	bias[2] /= static_cast<int32_t>(sample_cnt);

	// Save gyro bias for later use
	// Gyro data is 16 LSB/dps
	gyro_bias_[0] = static_cast<float>(bias[0] / 16.);
	gyro_bias_[1] = static_cast<float>(bias[1] / 16.);
	gyro_bias_[2] = static_cast<float>(bias[2] / 16.);

	return true;
}


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

void AutoRoverDFRobotGravity10DoF::SetConfiguration(DFRobotGravity10DoFConfig &config)
{
	config_ = config;
}

void AutoRoverDFRobotGravity10DoF::Initialize()
{
	uint8_t self_test;

	ROS_INFO("Checking self-test results");

	if (i2c_.ReadByte(BNO055_ST_RESULT, self_test))
	{
		if (self_test & 0x01)
		{
			ROS_INFO("Accelerometer passed self-test");
		}
		else
		{
			ROS_WARN("Accelerometer failed self-test");
		}

		if (self_test & 0x02)
		{
			ROS_INFO("Magnetometer passed self-test");
		}
		else
		{
			ROS_WARN("Magnetometer failed self-test");
		}

		if (self_test & 0x04)
		{
			ROS_INFO("Gyroscope passed self-test");
		}
		else
		{
			ROS_WARN("Gyroscope failed self-test");
		}

		if (self_test & 0x08)
		{
			ROS_INFO("MCU passed self-test");
		}
		else
		{
			ROS_WARN("MCU failed self-test");
		}
	}
	else
	{
		ROS_WARN("Failed to read device self-test results");
	}
}

bool AutoRoverDFRobotGravity10DoF::InitBNO055()
{
	uint8_t data;

	if (!i2c_.IsConnected())
	{
		ROS_ERROR("Not connected to Gravity 10DoF. Aborting BNO055 initialization.");
		return false;
	}

	ROS_INFO("Initializing BNO055");

	// Select BNO055 config mode
	if (!i2c_.WriteByte(BNO055_OPR_MODE, CONFIGMODE))
	{
		ROS_ERROR("Failed to set BNO055 config mode.");
		return false;
	}
	std::this_thread::sleep_for(std::chrono::milliseconds(25));

	// Select page 1 to configure sensors
	if (!i2c_.WriteByte(BNO055_PAGE_ID, 0x01))
	{
		ROS_ERROR("Failed to select page 1 to configure sensors");
		return false;
	}

	// Configure ACC
	data = config_.a_pwr_mode << 5 | config_.a_bw << 2 | config_.a_scale;
	if (!i2c_.WriteByte(BNO055_ACC_CONFIG, data))
	{
		ROS_ERROR("Failed to configure accelerometer");
		return false;
	}

	// Configure GYR
	data = config_.g_bw << 3 | config_.g_scale;
	if (!i2c_.WriteByte(BNO055_GYRO_CONFIG_0, data))
	{
		ROS_ERROR("Failed to configure gyroscope");
		return false;
	}
	if (!i2c_.WriteByte(BNO055_GYRO_CONFIG_1, config_.g_pwr_mode))
	{
		ROS_ERROR("Failed to configure gyroscope");
		return false;
	}

	// Configure MAG
	data = config_.m_pwr_mode << 5 | config_.m_op_mode << 3 | config_.m_odr;
	if (!i2c_.WriteByte(BNO055_MAG_CONFIG, data))
	{
		ROS_ERROR("Failed to configure magnetometer");
		return false;
	}

	// Select page 0 to read sensors
	if (!i2c_.WriteByte(BNO055_PAGE_ID, 0x00))
	{
		ROS_ERROR("Failed to select page 0 to read sensors");
		return false;
	}

	// Select BNO055 gyro temperature source
	if (!i2c_.WriteByte(BNO055_TEMP_SOURCE, 0x01))
	{
		ROS_ERROR("Failed to select gyroscope temperature source");
		return false;
	}

	// Select BNO055 sensor units (temperature in degrees F, rate in rps, accel in m/s^2)
	if (!i2c_.WriteByte(BNO055_UNIT_SEL, 0x16))
	{
		ROS_ERROR("Failed to set BNO055 sensor units to degrees F, rate in rps and accel in m/s^2");
		return false;
	}

	// Select BNO055 system power mode
	if (!i2c_.WriteByte(BNO055_PWR_MODE, config_.pwr_mode))
	{
		ROS_ERROR("Failed to set BNO055 system power mode");
		return false;
	}

	// Select BNO055 system operation mode
	if (!i2c_.WriteByte(BNO055_OPR_MODE, config_.opr_mode))
	{
		ROS_ERROR("Failed to set BNO055 operation mode.");
		return false;
	}
	std::this_thread::sleep_for(std::chrono::milliseconds(25));

	ROS_INFO("BNO055 initialization complete!");

	return true;
}

bool AutoRoverDFRobotGravity10DoF::InitBMP280()
{
	uint8_t data;

	if (!i2c_.IsConnected())
	{
		ROS_ERROR("Not connected to Gravity 10DoF. Aborting BMP280 initialization.");
		return false;
	}

	ROS_INFO("Initializing BMP280 altimeter and pressure sensor");

	// Reset before initialization
	if (!i2c_.WriteByte(BMP280_RESET, 0xB6))
	{
		ROS_ERROR("Failed to reset BMP280");
		return false;
	}

	// Set T and P oversampling rates and sensor mode
	data = config_.t_osr << 5 | config_.p_osr << 2 | config_.mode;
	if (!i2c_.WriteByte(BMP280_CTRL_MEAS, data))
	{
		ROS_ERROR("Failed to set BMP280 oversampling rates and sensor mode");
		return false;
	}

	// Set standby time interval in normal mode and bandwidth
	data = config_.sby << 5 | config_.iir_filter << 2;
	if (!i2c_.WriteByte(BMP280_CONFIG, data))
	{
		ROS_ERROR("Failed to set BMP280 standby time interval for normal mode and bandwidth");
		return false;
	}

	// Read and store calibration data
	if (!i2c_.ReadBytes(BMP280_CALIB00, bmp_calib_, BMP280_CALIB_SIZE))
	{
		ROS_ERROR("Failed to calibrate the BMP280");
		return false;
	}

	bmp_comp_params_.dig_t1 = static_cast<uint16_t>((static_cast<uint16_t>(bmp_calib_[1]) << 8) | bmp_calib_[0]);
	bmp_comp_params_.dig_t2 = static_cast<int16_t>((static_cast<int16_t>(bmp_calib_[3]) << 8) | bmp_calib_[2]);
	bmp_comp_params_.dig_t3 = static_cast<int16_t>((static_cast<int16_t>(bmp_calib_[5]) << 8) | bmp_calib_[4]);
	bmp_comp_params_.dig_p1 = static_cast<uint16_t>((static_cast<uint16_t>(bmp_calib_[7]) << 8) | bmp_calib_[6]);
	bmp_comp_params_.dig_p2 = static_cast<int16_t>((static_cast<int16_t>(bmp_calib_[9]) << 8) | bmp_calib_[8]);
	bmp_comp_params_.dig_p3 = static_cast<int16_t>((static_cast<int16_t>(bmp_calib_[11]) << 8) | bmp_calib_[10]);
	bmp_comp_params_.dig_p4 = static_cast<int16_t>((static_cast<int16_t>(bmp_calib_[13]) << 8) | bmp_calib_[12]);
	bmp_comp_params_.dig_p5 = static_cast<int16_t>((static_cast<int16_t>(bmp_calib_[15]) << 8) | bmp_calib_[14]);
	bmp_comp_params_.dig_p6 = static_cast<int16_t>((static_cast<int16_t>(bmp_calib_[17]) << 8) | bmp_calib_[16]);
	bmp_comp_params_.dig_p7 = static_cast<int16_t>((static_cast<int16_t>(bmp_calib_[19]) << 8) | bmp_calib_[18]);
	bmp_comp_params_.dig_p8 = static_cast<int16_t>((static_cast<int16_t>(bmp_calib_[21]) << 8) | bmp_calib_[20]);
	bmp_comp_params_.dig_p9 = static_cast<int16_t>((static_cast<int16_t>(bmp_calib_[23]) << 8) | bmp_calib_[22]);

	ROS_INFO("BMP280 initialization complete!");

	return true;
}

bool AutoRoverDFRobotGravity10DoF::AccelGyroCalBNO055()
{
	uint8_t tmp;
	vector<uint8_t> data(6, 0x00); // Temp array for accelerometer & gyro x, y, z data
	vector<int32_t> bias(3, 0);
	uint16_t sample_cnt = 256;

	ROS_INFO("Accel/Gyro Calibration: Put device on a level surface and keep motionless! Wait...");
	this_thread::sleep_for(chrono::milliseconds(4000));

	if (!i2c_.WriteByte(BNO055_PAGE_ID, 0x00))
	{
		ROS_ERROR("Failed to select page 0 to read sensors");
		return false;
	}

	if (!i2c_.WriteByte(BNO055_OPR_MODE, config_.mode))
	{
		ROS_ERROR("Failed to set BNO055 into config mode");
		return false;
	}
	this_thread::sleep_for(chrono::milliseconds(25));


	if (!i2c_.WriteByte(BNO055_OPR_MODE, AMG))
	{
		ROS_ERROR("Failed to set BNO055 operation mode to Accelerometer, Magnetometer, Gyroscope");
		return false;
	}

	//
	// In NDF fusion mode, accel full scale is at +/- 4g, ODR is 62.5 Hz, set it the same here
	//
	tmp = config_.a_pwr_mode << 5 | config_.a_bw << 2 | AFS_4G;
	if (!i2c_.WriteByte(BNO055_ACC_CONFIG, tmp))
	{
		ROS_ERROR("Failed to configure accelerometer");
		return false;
	}

	vector<int16_t> temp(3, 0x00);
	for (int i = 0; i < sample_cnt; i++)
	{
		if (!i2c_.ReadBytes(BNO055_ACC_DATA_X_LSB, data, data.size()))
		{
			ROS_WARN("Multi-byte read failed during accelerometer calibration. Results may be skewed");
		}
		else
		{
			temp[0] = static_cast<int16_t>((static_cast<int16_t>(data[1]) << 8) | data[0]);
			temp[1] = static_cast<int16_t>((static_cast<int16_t>(data[3]) << 8) | data[2]);
			temp[2] = static_cast<int16_t>((static_cast<int16_t>(data[5]) << 8) | data[4]);

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
	accel_bias_[0] = static_cast<double>(bias[0]);
	accel_bias_[1] = static_cast<double>(bias[1]);
	accel_bias_[2] = static_cast<double>(bias[2]);

	//
	// In NDF fusion mode, gyro full scale is at +/- 2000 dps, ODR is 32 Hz
	//
	if (!i2c_.WriteByte(BNO055_GYRO_CONFIG_0, config_.g_bw << 3 | GFS_2000DPS))
	{
		ROS_ERROR("Failed to configure gyro");
		return false;
	}

	if (!i2c_.WriteByte(BNO055_GYRO_CONFIG_1, config_.g_pwr_mode))
	{
		ROS_ERROR("Failed to configure gyro");
		return false;
	}

	temp.clear();
	data.clear();
	bias.clear();
	temp.resize(3, 0x00);
	data.resize(6, 0x00);
	bias.resize(3, 0);
	for (int i = 0; i < sample_cnt; i++)
	{
		if (!i2c_.ReadBytes(BNO055_GYR_DATA_X_LSB, data, data.size()))
		{
			ROS_WARN("Multi-byte read failed during gyroscope calibration. Results may be skewed");
		}
		else
		{
			temp[0] = static_cast<int16_t>((static_cast<int16_t>(data[1]) << 8) | data[0]);
			temp[1] = static_cast<int16_t>((static_cast<int16_t>(data[3]) << 8) | data[2]);
			temp[2] = static_cast<int16_t>((static_cast<int16_t>(data[5]) << 8) | data[4]);

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
	gyro_bias_[0] = static_cast<double>(bias[0] / 16.);
	gyro_bias_[1] = static_cast<double>(bias[1] / 16.);
	gyro_bias_[2] = static_cast<double>(bias[2] / 16.);

	// Return to config mode to write accelerometer biases in offset register
	// This offset register is only used while in fusion mode when accelerometer full-scale is +/- 4g

	if (!i2c_.WriteByte(BNO055_OPR_MODE, CONFIGMODE))
	{
		ROS_ERROR("Failed to put BNO055 into config mode.");
		return false;
	}
	this_thread::sleep_for(chrono::milliseconds(25));

	//
	// Write accelerometer biases to offset register
	//
	if (!i2c_.WriteByte(BNO055_ACC_OFFSET_X_LSB, static_cast<int16_t>(accel_bias_[0]) & 0xFF))
	{
		ROS_WARN("Failed to write accelerometer offset X LSB");
	}

	if (!i2c_.WriteByte(BNO055_ACC_OFFSET_X_MSB, (static_cast<int16_t>(accel_bias_[0]) >> 8) & 0xFF))
	{
		ROS_WARN("Failed to write accelerometer offset X MSB");
	}

	if (!i2c_.WriteByte(BNO055_ACC_OFFSET_Y_LSB, static_cast<int16_t>(accel_bias_[1]) & 0xFF))
	{
		ROS_WARN("Failed to write accelerometer offset X LSB");
	}

	if (!i2c_.WriteByte(BNO055_ACC_OFFSET_Y_MSB, (static_cast<int16_t>(accel_bias_[1]) >> 8) & 0xFF))
	{
		ROS_WARN("Failed to write accelerometer offset X MSB");
	}

	if (!i2c_.WriteByte(BNO055_ACC_OFFSET_Z_LSB, static_cast<int16_t>(accel_bias_[2]) & 0xFF))
	{
		ROS_WARN("Failed to write accelerometer offset X LSB");
	}

	if (!i2c_.WriteByte(BNO055_ACC_OFFSET_Z_MSB, (static_cast<int16_t>(accel_bias_[2]) >> 8) & 0xFF))
	{
		ROS_WARN("Failed to write accelerometer offset X MSB");
	}

    // Check that offsets were properly written to offset registers
    uint8_t byte;
    int16_t sensor_bias;
    i2c_.ReadByte(BNO055_ACC_OFFSET_X_MSB, byte);
    sensor_bias = byte << 8;
    i2c_.ReadByte(BNO055_ACC_OFFSET_X_LSB, byte);
    sensor_bias |= byte;
    ROS_INFO("Average accelerometer X bias = %i", sensor_bias);

    i2c_.ReadByte(BNO055_ACC_OFFSET_Y_MSB, byte);
    sensor_bias = byte << 8;
    i2c_.ReadByte(BNO055_ACC_OFFSET_Y_LSB, byte);
    sensor_bias |= byte;
    ROS_INFO("Average accelerometer Y bias = %i", sensor_bias);

    i2c_.ReadByte(BNO055_ACC_OFFSET_Z_MSB, byte);
    sensor_bias = byte << 8;
    i2c_.ReadByte(BNO055_ACC_OFFSET_Z_LSB, byte);
    sensor_bias |= byte;
    ROS_INFO("Average accelerometer Z bias = %i", sensor_bias);

	//
	// Write gyroscope biases to offset register
	//
	if (!i2c_.WriteByte(BNO055_GYR_OFFSET_X_LSB, static_cast<int16_t>(gyro_bias_[0]) & 0xFF))
	{
		ROS_WARN("Failed to write gyroscope offset X LSB");
	}

	if (!i2c_.WriteByte(BNO055_GYR_OFFSET_X_MSB, (static_cast<int16_t>(gyro_bias_[0]) >> 8) & 0xFF))
	{
		ROS_WARN("Failed to write gyroscope offset X MSB");
	}

	if (!i2c_.WriteByte(BNO055_GYR_OFFSET_Y_LSB, static_cast<int16_t>(gyro_bias_[1]) & 0xFF))
	{
		ROS_WARN("Failed to write gyroscope offset X LSB");
	}

	if (!i2c_.WriteByte(BNO055_GYR_OFFSET_Y_MSB, (static_cast<int16_t>(gyro_bias_[1]) >> 8) & 0xFF))
	{
		ROS_WARN("Failed to write gyroscope offset X MSB");
	}

	if (!i2c_.WriteByte(BNO055_GYR_OFFSET_Z_LSB, static_cast<int16_t>(gyro_bias_[2]) & 0xFF))
	{
		ROS_WARN("Failed to write gyroscope offset X LSB");
	}

	if (!i2c_.WriteByte(BNO055_GYR_OFFSET_Z_MSB, (static_cast<int16_t>(gyro_bias_[2]) >> 8) & 0xFF))
	{
		ROS_WARN("Failed to write gyroscope offset X MSB");
	}

    i2c_.ReadByte(BNO055_GYR_OFFSET_X_MSB, byte);
    sensor_bias = byte << 8;
    i2c_.ReadByte(BNO055_GYR_OFFSET_X_LSB, byte);
    sensor_bias |= byte;
    ROS_INFO("Average gyroscope X bias = %i", sensor_bias);

    i2c_.ReadByte(BNO055_GYR_OFFSET_Y_MSB, byte);
    sensor_bias = byte << 8;
    i2c_.ReadByte(BNO055_GYR_OFFSET_Y_LSB, byte);
    sensor_bias |= byte;
    ROS_INFO("Average gyroscope Y bias = %i", sensor_bias);

    i2c_.ReadByte(BNO055_GYR_OFFSET_Z_MSB, byte);
    sensor_bias = byte << 8;
    i2c_.ReadByte(BNO055_GYR_OFFSET_Z_LSB, byte);
    sensor_bias |= byte;
    ROS_INFO("Average gyroscope Z bias = %i", sensor_bias);

	// Select BNO055 system operation mode
	if (!i2c_.WriteByte(BNO055_OPR_MODE, config_.opr_mode))
	{
		ROS_ERROR("Failed to set BNO055 to NDOF operation mode");
		return false;
	}

	ROS_INFO("Accelerometer / Gyroscope calibration done!");

	return true;
}

bool AutoRoverDFRobotGravity10DoF::MagCalBNO055()
{
	vector<uint8_t> data(6, 0); // data array to hold mag x, y, z, data
	uint16_t ii = 0, sample_count = 0;
	vector<int32_t> mag_bias(3, 0);
	vector<int16_t> mag_max(3, 0), mag_min(3, 0);

	ROS_INFO("Mag Calibration: Wave device in a figure eight until done!");
	this_thread::sleep_for(chrono::milliseconds(4000));

	// Select page 0 to read sensors
	if (!i2c_.WriteByte(BNO055_PAGE_ID, 0x00))
	{
		ROS_ERROR("Failed to select page 0 for magnetometer calibration");
		return false;
	}

	// Select BNO055 system operation mode as NDOF for calibration
	if (!i2c_.WriteByte(BNO055_OPR_MODE, CONFIGMODE))
	{
		ROS_ERROR("Failed to set BNO055 operation mode to NDOF");
		return false;
	}
	this_thread::sleep_for(chrono::milliseconds(25));

	if (!i2c_.WriteByte(BNO055_OPR_MODE, AMG))
	{
		ROS_ERROR("Failed to magnetometer mode");
		return false;
	}

	// In NDF fusion mode, mag data is in 16 LSB/microTesla, ODR is 20 Hz in forced mode
	sample_count = 256;
	vector<int16_t> mag_temp(3, 0);
	for (ii = 0; ii < sample_count; ii++)
	{
		if (!i2c_.ReadBytes(BNO055_MAG_DATA_X_LSB, data,
		                    data.size()))  // Read the six raw data registers into data array
		{
			ROS_WARN("Failed to read mag data for calibration. Skipping sample");
			continue;
		}

		mag_temp[0] = static_cast<int16_t>((static_cast<int16_t>(data[1]) << 8) |
		                                   data[0]);   // Form signed 16-bit integer for each sample in FIFO
		mag_temp[1] = static_cast<int16_t>((static_cast<int16_t>(data[3]) << 8) | data[2]);
		mag_temp[2] = static_cast<int16_t>((static_cast<int16_t>(data[5]) << 8) | data[4]);

		for (int jj = 0; jj < 3; jj++)
		{
			if (ii == 0)
			{
				mag_max[jj] = mag_temp[jj]; // Offsets may be large enough that mag_temp[i] may not be bipolar!
				mag_min[jj] = mag_temp[jj]; // This prevents max or min being pinned to 0 if the values are unipolar...
			}
			else
			{
				if (mag_temp[jj] > mag_max[jj]) mag_max[jj] = mag_temp[jj];
				if (mag_temp[jj] < mag_min[jj]) mag_min[jj] = mag_temp[jj];
			}
		}

		this_thread::sleep_for(chrono::milliseconds(105)); // at 10 Hz ODR, new mag data is available every 100 ms
	}

    ROS_INFO("Mag X min/max: %i, %i", mag_min[0], mag_max[0]);
    ROS_INFO("Mag Y min/max: %i, %i", mag_min[1], mag_max[1]);
    ROS_INFO("Mag Z min/max: %i, %i", mag_min[2], mag_max[2]);

	mag_bias[0] = (mag_max[0] + mag_min[0]) / 2;  // get average x mag bias in counts
	mag_bias[1] = (mag_max[1] + mag_min[1]) / 2;  // get average y mag bias in counts
	mag_bias[2] = (mag_max[2] + mag_min[2]) / 2;  // get average z mag bias in counts

	mag_bias_[0] = (double) mag_bias[0] / 1.6;  // save mag biases in mG for use in main program
	mag_bias_[1] = (double) mag_bias[1] / 1.6;  // mag data is 1.6 LSB/mg
	mag_bias_[2] = (double) mag_bias[2] / 1.6;

	// Return to config mode to write mag biases in offset register
	// This offset register is only used while in fusion mode when magnetometer sensitivity is 16 LSB/microTesla
	if (!i2c_.WriteByte(BNO055_OPR_MODE, CONFIGMODE))
	{
		ROS_ERROR("Failed to put BNO055 in to configuration mode");
		return false;
	}
	this_thread::sleep_for(chrono::milliseconds(25));

	//
	// Write biases to accelerometer offset registers as 16 LSB/microTesla
	//
	if (!i2c_.WriteByte(BNO055_MAG_OFFSET_X_LSB, static_cast<int16_t>(mag_bias_[0]) & 0xFF))
	{
		ROS_WARN("Failed to write magnetometer offset X LSB");
	}

	if (!i2c_.WriteByte(BNO055_MAG_OFFSET_X_MSB, (static_cast<int16_t>(mag_bias_[0]) >> 8) & 0xFF))
	{
		ROS_WARN("Failed to write magnetometer offset X MSB");
	}

	if (!i2c_.WriteByte(BNO055_MAG_OFFSET_Y_LSB, static_cast<int16_t>(mag_bias_[1]) & 0xFF))
	{
		ROS_WARN("Failed to write magnetometer offset X LSB");
	}

	if (!i2c_.WriteByte(BNO055_MAG_OFFSET_Y_MSB, (static_cast<int16_t>(mag_bias_[1]) >> 8) & 0xFF))
	{
		ROS_WARN("Failed to write magnetometer offset X MSB");
	}

	if (!i2c_.WriteByte(BNO055_MAG_OFFSET_Z_LSB, static_cast<int16_t>(mag_bias_[2]) & 0xFF))
	{
		ROS_WARN("Failed to write magnetometer offset X LSB");
	}

	if (!i2c_.WriteByte(BNO055_MAG_OFFSET_Z_MSB, (static_cast<int16_t>(mag_bias_[2]) >> 8) & 0xFF))
	{
		ROS_WARN("Failed to write magnetometer offset X MSB");
	}

	// Select BNO055 system operation mode
	if (!i2c_.WriteByte(BNO055_OPR_MODE, config_.opr_mode))
	{
		ROS_ERROR("Failed to set BNO055 to NDOF operation mode");
		return false;
	}
	this_thread::sleep_for(chrono::milliseconds(25));

	ROS_INFO("Magnetometer Calibration done!");

    return true;
}

uint32_t AutoRoverDFRobotGravity10DoF::CompensateTemperature(int32_t &adc)
{
	int32_t var1, var2, T;
	var1 = ((((adc >> 3) - ((int32_t) bmp_comp_params_.dig_t1 << 1))) * ((int32_t) bmp_comp_params_.dig_t2)) >> 11;
	var2 = (((((adc >> 4) - ((int32_t) bmp_comp_params_.dig_t1)) * ((adc >> 4) - ((int32_t) bmp_comp_params_.dig_t1)))
			>> 12) * ((int32_t) bmp_comp_params_.dig_t3)) >> 14;
	config_.t_fine = var1 + var2;
	T = (config_.t_fine * 5 + 128) >> 8;
	return T;
}

uint32_t AutoRoverDFRobotGravity10DoF::CompensatePressure(int32_t &adc)
{
	long long var1, var2, p;
	var1 = ((long long) config_.t_fine) - 128000;
	var2 = var1 * var1 * (long long) bmp_comp_params_.dig_p6;
	var2 = var2 + ((var1 * (long long) bmp_comp_params_.dig_p5) << 17);
	var2 = var2 + (((long long) bmp_comp_params_.dig_p4) << 35);
	var1 = ((var1 * var1 * (long long) bmp_comp_params_.dig_p3) >> 8) +
	       ((var1 * (long long) bmp_comp_params_.dig_p2) << 12);
	var1 = (((((long long) 1) << 47) + var1)) * ((long long) bmp_comp_params_.dig_p1) >> 33;

	if (var1 == 0)
	{
		return 0;
		// avoid exception caused by division by zero
	}
	p = 1048576 - adc;
	p = (((p << 31) - var2) * 3125) / var1;
	var1 = (((long long) bmp_comp_params_.dig_p9) * (p >> 13) * (p >> 13)) >> 25;
	var2 = (((long long) bmp_comp_params_.dig_p8) * p) >> 19;
	p = ((p + var1 + var2) >> 8) + (((long long) bmp_comp_params_.dig_p7) << 4);
	return (uint32_t) p;
}

int32_t AutoRoverDFRobotGravity10DoF::ReadBMP280Temperature()
{
	vector<uint8_t> rawData(3); // 20-bit temperature register data stored here
	if (!i2c_.ReadBytes(BMP280_TEMP_MSB, rawData, rawData.size()))
	{
		ROS_ERROR("Failed to read raw temperature data");
		return INFINITY;
	}
	return static_cast<int32_t>(((static_cast<int32_t>(rawData[0]) << 16 | static_cast<int32_t>(rawData[1]) << 8 |
	                              static_cast<int32_t>(rawData[2])) >> 4));
}

int32_t AutoRoverDFRobotGravity10DoF::ReadBMP280Pressure()
{
	vector<uint8_t> rawData(3);  // 20-bit pressure register data stored here
	if (!i2c_.ReadBytes(BMP280_PRESS_MSB, rawData, rawData.size()))
	{
		ROS_ERROR("Failed to read raw temperature data");
		return INFINITY;
	}
	return static_cast<int32_t>(((static_cast<int32_t>(rawData[0]) << 16 | static_cast<int32_t>(rawData[1]) << 8 |
	                              static_cast<int32_t>(rawData[2])) >> 4));
}

void AutoRoverDFRobotGravity10DoF::ReadAccelData(vector<double> &destination)
{
	vector<uint8_t> rawData(6, 0);  // x/y/z accel register data stored here

	destination.clear();
	destination.resize(3, 0);

	// Read the six raw data registers into data array
	if (!i2c_.ReadBytes(BNO055_ACC_DATA_X_LSB, rawData, rawData.size()))
	{
		ROS_ERROR("Failed to read accelerometer x/y/z");
		return;
	}

	// Turn the MSB and LSB into a signed 16-bit value
	destination[0] = static_cast<double>((static_cast<int16_t>(rawData[1]) << 8) | rawData[0]) / 100.0;
	destination[1] = static_cast<double>((static_cast<int16_t>(rawData[3]) << 8) | rawData[2]) / 100.0;
	destination[2] = static_cast<double>((static_cast<int16_t>(rawData[5]) << 8) | rawData[4]) / 100.0;
}


void AutoRoverDFRobotGravity10DoF::ReadGyroData(vector<double> &destination)
{
	vector<uint8_t> rawData(6, 0);  // x/y/z gyro register data stored here

	destination.clear();
	destination.resize(3, 0);

	// Read the six raw data registers into data array
	if (!i2c_.ReadBytes(BNO055_GYR_DATA_X_LSB, rawData, rawData.size()))
	{
		ROS_ERROR("Failed to read gyroscope x/y/z");
		return;
	}

	// Turn the MSB and LSB into a signed 16-bit value
	destination[0] = static_cast<double>((static_cast<int16_t>(rawData[1]) << 8) | rawData[0]) / 16.0;
	destination[1] = static_cast<double>((static_cast<int16_t>(rawData[3]) << 8) | rawData[2]) / 16.0;
	destination[2] = static_cast<double>((static_cast<int16_t>(rawData[5]) << 8) | rawData[4]) / 16.0;
}

int8_t AutoRoverDFRobotGravity10DoF::ReadGyroTempData()
{
	uint8_t data;

	if (!i2c_.ReadByte(BNO055_TEMP, data))
	{
		ROS_ERROR("Failed to read raw temperature data");
		return INFINITY;
	}

	return data;  // Read the two raw data registers sequentially into data array
}

void AutoRoverDFRobotGravity10DoF::ReadMagData(vector<double> &destination)
{
	vector<uint8_t> rawData(6, 0);  // x/y/z mag register data stored here

	destination.clear();
	destination.resize(3, 0);

	// Read the six raw data registers into data array
	if (!i2c_.ReadBytes(BNO055_MAG_DATA_X_LSB, rawData, rawData.size()))
	{
		ROS_ERROR("Failed to read magnetometer x/y/z");
		return;
	}

	// Turn the MSB and LSB into a signed 16-bit value
	destination[0] = static_cast<double>((static_cast<int16_t>(rawData[1]) << 8) | rawData[0]) / 1.6;
	destination[1] = static_cast<double>((static_cast<int16_t>(rawData[3]) << 8) | rawData[2]) / 1.6;
	destination[2] = static_cast<double>((static_cast<int16_t>(rawData[5]) << 8) | rawData[4]) / 1.6;
}

void AutoRoverDFRobotGravity10DoF::ReadQuatData(vector<double> &destination)
{
	vector<uint8_t> rawData(8, 0);  // w/x/y/z quaternion register data stored here

	destination.clear();
	destination.resize(4, 0);

	// Read the six raw data registers into data array
	if (!i2c_.ReadBytes(BNO055_QUA_DATA_W_LSB, rawData, rawData.size()))
	{
		ROS_ERROR("Failed to read quaternion data");
		return;
	}

	// Turn the MSB and LSB into a signed 16-bit value
	destination[0] = static_cast<double>((static_cast<int16_t>(rawData[2]) << 8) | rawData[0]) / 16384.0 ;
	destination[1] = static_cast<double>((static_cast<int16_t>(rawData[3]) << 8) | rawData[2]) / 16384.0;
	destination[2] = static_cast<double>((static_cast<int16_t>(rawData[5]) << 8) | rawData[4]) / 16384.0;
	destination[3] = static_cast<double>((static_cast<int16_t>(rawData[7]) << 8) | rawData[6]) / 16384.0;
}

void AutoRoverDFRobotGravity10DoF::ReadEulData(vector<double> &destination)
{
	vector<uint8_t> rawData(6, 0);  // x/y/z euler register data stored here

	destination.clear();
	destination.resize(3, 0);

	// Read the six raw data registers into data array
	if (!i2c_.ReadBytes(BNO055_EUL_HEADING_LSB, rawData, rawData.size()))
	{
		ROS_ERROR("Failed to read euler heading/pitch/yaw");
		return;
	}

	// Turn the MSB and LSB into a signed 16-bit value
	destination[0] = static_cast<double>((static_cast<int16_t>(rawData[1]) << 8) | rawData[0]) / 900.0;
	destination[1] = static_cast<double>((static_cast<int16_t>(rawData[3]) << 8) | rawData[2]) / 900.0;
	destination[2] = static_cast<double>((static_cast<int16_t>(rawData[5]) << 8) | rawData[4]) / 900.0;
}

void AutoRoverDFRobotGravity10DoF::ReadLIAData(vector<double> &destination)
{
	vector<uint8_t> rawData(6, 0);  // x/y/z linear accel register data stored here

	destination.clear();
	destination.resize(3, 0);

	// Read the six raw data registers into data array
	if (!i2c_.ReadBytes(BNO055_LIA_DATA_X_LSB, rawData, rawData.size()))
	{
		ROS_ERROR("Failed to read linear acceleration x/y/z");
		return;
	}

	// Turn the MSB and LSB into a signed 16-bit value
	destination[0] = static_cast<double>((static_cast<int16_t>(rawData[1]) << 8) | rawData[0]) / 100.0;
	destination[1] = static_cast<double>((static_cast<int16_t>(rawData[3]) << 8) | rawData[2]) / 100.0;
	destination[2] = static_cast<double>((static_cast<int16_t>(rawData[5]) << 8) | rawData[4]) / 100.0;
}

void AutoRoverDFRobotGravity10DoF::ReadGRVData(vector<double> &destination)
{
	vector<uint8_t> rawData(6, 0);  // x/y/z gravity vector register data stored here

	destination.clear();
	destination.resize(3, 0);

	// Read the six raw data registers into data array
	if (!i2c_.ReadBytes(BNO055_GRV_DATA_X_LSB, rawData, rawData.size()))
	{
		ROS_ERROR("Failed to read gravity vector x/y/z");
		return;
	}

	// Turn the MSB and LSB into a signed 16-bit value
	destination[0] = static_cast<double>((static_cast<int16_t>(rawData[1]) << 8) | rawData[0]) / 100.0;
	destination[1] = static_cast<double>((static_cast<int16_t>(rawData[3]) << 8) | rawData[2]) / 100.0;
	destination[2] = static_cast<double>((static_cast<int16_t>(rawData[5]) << 8) | rawData[4]) / 100.0;
}

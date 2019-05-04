#include <cstring>
#include <cerrno>
#include <thread>
#include <chrono>
#include <linux/i2c-dev.h>
#include <i2c/smbus.h>

#include <auto_rover_dfrobot_gravity_10dof/auto_rover_dfrobot_gravity_10dof.h>

#define BUFFSIZE 120

using namespace auto_rover_dfrobot_gravity_10dof;

AutoRoverDFRobotGravity10DoF::~AutoRoverDFRobotGravity10DoF()
{
    close( fd_ );
}

AutoRoverDFRobotGravity10DoF::SetConfiguration(DFRobotGravity10DoFConfig& config)
{
    config_ = config;
}

int AutoRoverDFRobotGravity10DoF::Connect(const uint8_t addr)
{
    char buf[BUFFSIZE];
    uint8_t self_test;

    memset(buf, 0, BUFFSIZE);
    std::snprintf(buf, BUFFSIZE-1, "/dev/i2c-%d", bus_);
    if ((fd_ = open(buf, O_RDWR)) < 0)
    {
        memset(buf, 0, BUFFSIZE);
        snprintf(buf, BUFFSIZE-1, " [ERROR %d] Failed to open I2C bus /dev/i2c-%d", errno, bus_);
        perror(buf);
        ROS_ERROR(buf);

        return errno;
    }
    else if (ioctl(buf, I2C_SLAVE, addr) < 0)
    {
        memset(buf, 0, BUFFSIZE);
        snprintf(buf, BUFFSIZE-1, " [ERROR %d] Failed to connect to device 0x%X on I2C bus /dev/i2c-%d", errno, addr, bus_);
        perror(buf);
        ROS_ERROR(buf);

        return errno;
    }

    ROS_INFO("Checking self-test results");
    self_test = static_cast<uint8_t>(i2c_smbus_read_byte_data(fd_, BNO055_ST_RESULT));
    if (self_test & 0x01)
    {
        ROS_INFO("Accelerometer passed self-test");
    }
    else
    {
        ROS_ERROR("Accelerometer failed self-test");
    }

    if (self_test & 0x02)
    {
        ROS_INFO("Magnetometer passed self-test");
    }
    else
    {
        ROS_ERROR("Magnetometer failed self-test");
    }

    if (self_test & 0x04)
    {
        ROS_INFO("Gyroscope passed self-test");
    }
    else
    {
        ROS_ERROR("Gyroscope failed self-test");
    }

    if (self_test & 0x08)
    {
        ROS_INFO("MCU passed self-test");
    }
    else
    {
        ROS_ERROR("MCU failed self-test");
    }

    return 0;
}

int AutoRoverDFRobotGravity10DoF::InitBNO055()
{
    uint8_t data;
    int ret;

    if (fd_ < 0)
    {
        ROS_ERROR("Not connected to Gravity 10DoF. Aborting BNO055 initialization.");
        return fd_;
    }

    // Select BNO055 config mode
    if ((ret = i2c_smbus_write_byte_data(fd_, BNO055_OPR_MODE, CONFIGMODE)) < 0)
    {
        ROS_ERROR("Failed to set BNO055 config mode.");
        return ret;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(25));

    // Select page 1 to configure sensors
    if ((ret = i2c_smbus_write_byte_data(fd_, BNO055_PAGE_ID, 0x01)) < 0)
    {
        ROS_ERROR("Failed to select page 1 to configure sensors");
        return ret;
    }

    // Configure ACC
    data = config_.a_pwr_mode << 5 | config_.a_bw << 2 | config_.a_scale;
    if ((ret = i2c_smbus_write_byte_data(fd_, BNO055_ACC_CONFIG, data)) < 0)
    {
        ROS_ERROR("Failed to configure accelerometer");
        return ret;
    }

    // Configure GYR
    data = config_.g_bw << 3 | config_.g_scale;
    if ((ret = i2c_smbus_write_byte_data(fd_, BNO055_GYRO_CONFIG_0, data)) < 0)
    {
        ROS_ERROR("Failed to configure gyroscope");
        return ret;
    }
    if ((ret = i2c_smbus_write_byte_data(fd_, BNO055_GYRO_CONFIG_1, config_.g_pwr_mode)) < 0)
    {
        ROS_ERROR("Failed to configure gyroscope");
        return ret;
    }

    // Configure MAG
    data = config_.m_pwr_mode << 5 | config_.m_op_mode << 3 | config_.m_odr;
    if ((ret = i2c_smbus_write_byte_data(fd_, BNO055_MAG_CONFIG, data)) < 0)
    {
        ROS_ERROR("Failed to configure magnetometer");
        return ret;
    }

    // Select page 0 to read sensors
    if ((ret = i2c_smbus_write_byte_data(fd_, BNO055_PAGE_ID, 0x00)) < 0)
    {
        ROS_ERROR("Failed to select page 0 to read sensors");
        return ret;
    }

    // Select BNO055 gyro temperature source 
    if ((ret = i2c_smbus_write_byte_data(fd_, BNO055_TEMP_SOURCE, 0x01 )) < 0)
    {
        ROS_ERROR("Failed to select gyroscope temperature source");
        return ret;
    }

    // Select BNO055 sensor units (temperature in degrees C, rate in dps, accel in mg)
    if ((ret = i2c_smbus_write_byte_data(fd_, BNO055_UNIT_SEL, 0x01 )) < 0)
    {
        ROS_ERROR("Failed to set BNO055 sensor units to degrees C, rate in dps and accel in mg");
        return ret;
    }

    // Select BNO055 system power mode
    if ((ret = i2c_smbus_write_byte_data(fd_, BNO055_PWR_MODE, config_.pwr_mode )) < 0)
    {
        ROS_ERROR("Failed to set BNO055 system power mode");
        return ret;
    }

    // Select BNO055 system operation mode
    if ((ret = i2c_smbus_write_byte_data(fd_, BNO055_OPR_MODE, config_.opr_mode)) < 0)
    {
        ROS_ERROR("Failed to set BNO055 operation mode.");
        return ret;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(25));

    return 0;
}

int AutoRoverDFRobotGravity10DoF::InitBMP280()
{
    uint8_t data;
    int ret;

    if (fd_ < 0)
    {
        ROS_ERROR("Not connected to Gravity 10DoF. Aborting BMP280 initialization.");
        return fd_;
    }

    // Reset before initialization
    if ((ret = i2c_smbus_write_byte_data(fd_, BMP280_RESET, 0xB6)) < 0)
    {
        ROS_ERROR("Failed to reset BMP280");
        return ret;
    }

    // Set T and P oversampling rates and sensor mode
    data = config.t_osr << 5 | config.p_osr << 2 | config.mode;
    if ((ret = i2c_smbus_write_byte_data(fd_, BMP280_CTRL_MEAS, data)) < 0)
    {
        ROS_ERROR("Failed to set BMP280 oversampling rates and sensor mode");
        return ret;
    }

    // Set standby time interval in normal mode and bandwidth
    data = config.sby << 5 | config.iir_filter << 2;
    if ((ret = i2c_smbus_write_byte_data(fd_, BMP280_CONFIG, data)) < 0)
    {
        ROS_ERROR("Failed to set BMP280 standby time interval for normal mode and bandwidth");
        return ret;
    }

    // Read and store calibration data
    if ((ret = i2c_smbus_read_i2c_block_data(fd_, BMP280_CALIB00, BMP280_CALIB_SIZE, bmp_calib_)) < 0)
    {
        ROS_ERROR("Failed to calibrate the BMP280");
        return ret;
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

    return 0;
}

void AutoRoverDFRobotGravity10DoF::AccelGyroCalBNO055()
{
}


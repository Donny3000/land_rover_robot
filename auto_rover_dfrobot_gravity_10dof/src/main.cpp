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

#include <ros/ros.h>
#include <ros/console.h>
#include <dynamic_reconfigure/server.h>
#include <sensor_msgs/Imu.h>
#include <auto_rover_dfrobot_gravity_10dof/auto_rover_dfrobot_gravity_10dof.h>
#include <auto_rover_dfrobot_gravity_10dof/Gravity10DoFConfig.h>

// Setup the IMU
auto_rover_dfrobot_gravity_10dof::AutoRoverDFRobotGravity10DoF imu_(1, 0x28);

void dyn_config_cb(auto_rover_dfrobot_gravity_10dof::Gravity10DoFConfig &config, uint32_t level)
{
	if ( config.reset )
	{
		imu_.Reset();
	}
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "auto_rover_dfrobot_gravity_10dof");

	ros::NodeHandle nh("~");

    bool calibrate_imu;
    bool calibrate_mag;
	std::vector<double> accel;
	std::vector<double> gyro;
	std::vector<double> quat;
	sensor_msgs::Imu msg;
	msg.header.frame_id = "imu";

	ros::Publisher imu_pub = nh.advertise<sensor_msgs::Imu>("Imu", 10);

	imu_.Initialize();

    if ( !imu_.InitBNO055() )
    {
        ROS_ERROR("Failed to initialize BNO055!");
        return -1;
    }

	if ( !imu_.InitBMP280() )
	{
        ROS_ERROR("Failed to initialize BMP280!");
		return -1;
	}

    if ( nh.hasParam("calibrate_imu") )
    {
        nh.getParam("calibrate_imu", calibrate_imu);
        if ( calibrate_imu )
        {
            if ( !imu_.AccelGyroCalBNO055() )
                ROS_ERROR("Failed to calibrate the Accelerometer and Gyroscope");
        }
    }

    if ( nh.hasParam("calibrate_mag") )
    {
        nh.getParam("calibrate_mag", calibrate_mag);
        if ( calibrate_mag )
        {
            if ( !imu_.MagCalBNO055() )
                ROS_ERROR("Failed to calibrate the magnetometer");
        }
    }

	dynamic_reconfigure::Server<auto_rover_dfrobot_gravity_10dof::Gravity10DoFConfig> server;
	dynamic_reconfigure::Server<auto_rover_dfrobot_gravity_10dof::Gravity10DoFConfig>::CallbackType cb;

	cb = boost::bind(&dyn_config_cb, _1, _2);
	server.setCallback(cb);

	ros::Rate loop_rate(100);
	while ( ros::ok() )
	{
		imu_.ReadLIAData(accel);
		imu_.ReadGyroData(gyro);
		imu_.ReadQuatData(quat);

		msg.angular_velocity.x = gyro[0];
		msg.angular_velocity.y = gyro[1];
		msg.angular_velocity.z = gyro[2];

		msg.linear_acceleration.x = accel[0];
		msg.linear_acceleration.y = accel[1];
		msg.linear_acceleration.z = accel[2];

		msg.orientation.w = quat[0];
		msg.orientation.x = quat[1];
		msg.orientation.y = quat[2];
		msg.orientation.z = quat[3];

		imu_pub.publish( msg );
		ros::spinOnce();

		loop_rate.sleep();
	}

	return 0;
}

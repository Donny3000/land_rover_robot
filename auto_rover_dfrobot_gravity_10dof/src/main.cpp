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
#include <sensor_msgs/Imu.h>
#include <auto_rover_dfrobot_gravity_10dof/auto_rover_dfrobot_gravity_10dof.h>

int main(int argc, char** argv)
{
	ros::init(argc, argv, "auto_rover_dfrobot_gravity_10dof");

	ros::NodeHandle nh("~");

	ros::Publisher imu_pub = nh.advertise<sensor_msgs::Imu>("Imu", 10);

	// Setup the IMU
	auto_rover_dfrobot_gravity_10dof::AutoRoverDFRobotGravity10DoF imu(1, 0x28);

	imu.Initialize();

	if ( !imu.InitBNO055() )
	{
        ROS_ERROR("Failed to initialize BNO055!");
		return -1;
	}

	if ( !imu.InitBMP280() )
	{
        ROS_ERROR("Failed to initialize BMP280!");
		return -1;
	}

	std::vector<int16_t> accel;
	std::vector<int16_t> gyro;
	std::vector<int16_t> quat;
	sensor_msgs::Imu msg;
	msg.header.frame_id = "imu";

	//ros::Rate loop_rate(100);
	while ( ros::ok() )
	{
		imu.ReadLIAData(accel);
		imu.ReadGyroData(gyro);
		imu.ReadQuatData(quat);

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

		//loop_rate.sleep();
	}

	return 0;
}

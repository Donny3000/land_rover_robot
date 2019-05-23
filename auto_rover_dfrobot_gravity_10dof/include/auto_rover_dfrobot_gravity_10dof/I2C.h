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

#ifndef AUTO_ROVER_DFROBOT_GRAVITY_10DOF_I2C_H
#define AUTO_ROVER_DFROBOT_GRAVITY_10DOF_I2C_H

#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <cstdint>
extern "C" {
#include <i2c/smbus.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
}
#include <ros/ros.h>
#include <ros/console.h>

using namespace std;

namespace auto_rover_dfrobot_gravity_10dof
{
	class I2C
	{
	public:
		I2C(const uint8_t& bus, const uint8_t& addr, bool is_ten_bit = false);
		~I2C();

		bool IsConnected() { return is_initialized_; }

	protected:
		virtual bool WriteByte(const uint8_t& data);
		virtual bool WriteByte(const uint16_t& addr, const uint8_t& data);
		virtual bool WriteBytes(const uint16_t& addr, const vector<uint8_t>& data);

		virtual bool ReadByte(uint8_t& data);
		virtual bool ReadByte(const uint16_t& addr, uint8_t& data);
		virtual bool ReadBytes(const uint16_t& addr, vector<uint8_t>& data, uint8_t len);

	private:
		int f_;
		bool is_ten_bit_;
		bool is_initialized_;
	};
}

#endif //AUTO_ROVER_DFROBOT_GRAVITY_10DOF_I2C_H

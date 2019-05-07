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

#include "auto_rover_dfrobot_gravity_10dof/I2C.h"

using namespace std;
using namespace auto_rover_dfrobot_gravity_10dof;

I2C::I2C(const uint8_t &bus, const uint8_t& addr, bool is_ten_bit) :
	is_ten_bit_(is_ten_bit)
{
	int retval;
	string base("/dev/i2c-");
	base += to_string(bus);

	f_.open(base);
	if ( !f_.is_open() )
	{
		ROS_ERROR_STREAM("Failed to open I2C bus " + base);
		is_initialized_ = false;
	}
	else
	{
		if ((retval = ioctl(f_.get(), I2C_SLAVE, addr)) < 0)
		{
			ROS_ERROR_STREAM("Failed to select device 0x" << addr << " on i2c bus " << dec << bus);
			is_initialized_ = false;
		}
		else
		{
			ROS_INFO_STREAM("Connected to device 0x" << hex << addr << " on i2c bus " << bus);
			is_initialized_ = true;
		}
	}
}

I2C::~I2C()
{
	if ( f_.is_open() )
		f_.close();
}

bool I2C::WriteByte(const uint8_t& data)
{
	int retval;

	if ( !is_initialized_ )
	{
		ROS_ERROR("Not connected to an I2C device. Aborting.");
		return false;
	}

	if ((retval = i2c_smbus_write_byte(f_.get(), data)) < 0)
	{
		ROS_ERROR("Failed to write byte data to device");
		return false;
	}

	return true;
}

bool I2C::WriteByte(const uint16_t& addr, const uint8_t& data)
{
	int retval;

	if ( !is_initialized_ )
	{
		ROS_ERROR("Not connected to an I2C device. Aborting.");
		return false;
	}

	if ((retval = i2c_smbus_write_byte_data(f_.get(), addr, data)) < 0)
	{
		ROS_ERROR_STREAM("Failed to write byte data 0x" << hex << data << " to device at address 0x" << addr);
		return false;
	}

	return true;
}

bool I2C::WriteBytes(const uint16_t& addr, const vector<uint8_t>& data)
{
	int retval;

	if ( !is_initialized_ )
	{
		ROS_ERROR("Not connected to an I2C device. Aborting.");
		return false;
	}

	if ((retval = i2c_smbus_write_block_data(f_.get(), addr, data.size(), data.data())) < 0)
	{
		ROS_ERROR_STREAM("Failed to write bytes to device at address 0x" << hex << addr);
		return false;
	}

	return true;
}

bool I2C::ReadByte(uint8_t& data)
{
	int retval;

	if ( !is_initialized_ )
	{
		ROS_ERROR("Not connected to an I2C device. Aborting.");
		return false;
	}

	if ((retval = i2c_smbus_read_byte(f_.get())) < 0)
	{
		ROS_ERROR_STREAM("Failed to read byte data from device");
		return false;
	}
	else
	{
		data = retval & 0xFF;
	}

	return true;
}

bool I2C::ReadByte(const uint16_t& addr, uint8_t& data)
{
	int retval;

	if ( !is_initialized_ )
	{
		ROS_ERROR("Not connected to an I2C device. Aborting.");
		return false;
	}

	if ((retval = i2c_smbus_read_byte_data(f_.get(), addr)) < 0)
	{
		ROS_ERROR_STREAM("Failed to read byte data from device at address 0x" << addr);
		return false;
	}
	else
	{
		data = retval & 0xFF;
	}

	return true;
}

#ifdef I2C_FUNC_SMBUS_READ_BLOCK_DATA
bool I2C::ReadBytes(const uint16_t& addr, vector<uint8_t>& data, uint8_t len)
{
	bool retval;

	if ( !is_initialized_ )
	{
		ROS_ERROR("Not connected to an I2C device. Aborting.");
		return false;
	}

	data.clear();
	data.resize(len, 0x00);
	for (int i = 0; i < len; i++)
	{
		retval = ReadByte(addr, data.at(i));
		if( !retval )
			ROS_WARN("Error occurred reading data from device.");
	}

	return true;
}
#endif
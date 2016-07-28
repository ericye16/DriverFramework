/****************************************************************************
 *
 *   Copyright (C) 2016 Julian Oes. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#include <stdint.h>
#include <string.h>
#include "math.h"
#include "DriverFramework.hpp"
#include "MPU9250.hpp"
#include "MPU9250_mag.hpp"

#define MPU9250_ONE_G	9.80665f

#define MIN(_x, _y) (_x) > (_y) ? (_y) : (_x)

// Uncomment to allow additional debug output to be generated.
// #define MPU9250_DEBUG 1

using namespace DriverFramework;

void MPU9250::set_offsets()
{
	int16_t accel_offsets[3] = {};
	_bulkRead(MPUREG_XA_OFFS_H, (uint8_t *)&accel_offsets[0], 2);
	_bulkRead(MPUREG_YA_OFFS_H, (uint8_t *)&accel_offsets[1], 2);
	_bulkRead(MPUREG_ZA_OFFS_H, (uint8_t *)&accel_offsets[2], 2);

	accel_offsets[0] = swap16(accel_offsets[0]);
	accel_offsets[1] = swap16(accel_offsets[1]);
	accel_offsets[2] = swap16(accel_offsets[2]);
	DF_LOG_ERR("accel_offsets[0] = %i", accel_offsets[0]);
	DF_LOG_ERR("accel_offsets[1] = %i", accel_offsets[1]);
	DF_LOG_ERR("accel_offsets[2] = %i", accel_offsets[2]);

	usleep(25000);

	int16_t x_reading;
	_bulkRead(MPUREG_ACCEL_XOUT_H, (uint8_t *)&x_reading, 2);
	swap16(x_reading);
	DF_LOG_ERR("reading before zeroing accel %i", x_reading);

	usleep(25000);

	_writeReg(MPUREG_XA_OFFS_H, 0);
	_writeReg(MPUREG_XA_OFFS_L, 0);

	usleep(25000);

	_bulkRead(MPUREG_ACCEL_XOUT_H, (uint8_t *)&x_reading, 2);
	swap16(x_reading);
	DF_LOG_ERR("reading after zeroing accel %i", x_reading);

	usleep(25000);

	_bulkRead(MPUREG_XA_OFFS_H, (uint8_t *)&accel_offsets[0], 2);
	_bulkRead(MPUREG_YA_OFFS_H, (uint8_t *)&accel_offsets[1], 2);
	_bulkRead(MPUREG_ZA_OFFS_H, (uint8_t *)&accel_offsets[2], 2);
	
	usleep(25000);

	accel_offsets[0] = swap16(accel_offsets[0]);
	accel_offsets[1] = swap16(accel_offsets[1]);
	accel_offsets[2] = swap16(accel_offsets[2]);
	DF_LOG_ERR("accel_offsets[0] = %i", accel_offsets[0]);
	DF_LOG_ERR("accel_offsets[1] = %i", accel_offsets[1]);
	DF_LOG_ERR("accel_offsets[2] = %i", accel_offsets[2]);

}

// Accelerometer and gyroscope self test; check calibration wrt factory settings
void MPU9250::self_test(float
			*destination)  // Should return percent deviation from factory trim values, +/- 14 or less deviation is a pass
{
	uint8_t rawData[6] = {0, 0, 0, 0, 0, 0};
	uint8_t selfTest[6];
	int32_t gAvg[3] = {0}, aAvg[3] = {0}, aSTAvg[3] = {0}, gSTAvg[3] = {0};
	float factoryTrim[6];
	uint8_t FS = 0;

	_writeReg(MPUREG_SMPLRT_DIV, 0x00);    // Set gyro sample rate to 1 kHz
	_writeReg(MPUREG_CONFIG, 0x02);        // Set gyro sample rate to 1 kHz and DLPF to 92 Hz
	_writeReg(MPUREG_GYRO_CONFIG, 1 << FS); // Set full scale range for the gyro to 250 dps
	_writeReg(MPUREG_ACCEL_CONFIG2, 0x02); // Set accelerometer rate to 1 kHz and bandwidth to 92 Hz
	_writeReg(MPUREG_ACCEL_CONFIG, 1 << FS); // Set full scale range for the accelerometer to 2 g

	for (int ii = 0; ii < 200; ii++) {  // get average current values of gyro and acclerometer
		_bulkRead(MPUREG_ACCEL_XOUT_H, &rawData[0], 6);        // Read the six raw data registers into data array
		aAvg[0] += (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]) ;  // Turn the MSB and LSB into a signed 16-bit value
		aAvg[1] += (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]) ;
		aAvg[2] += (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]) ;

		_bulkRead(MPUREG_GYRO_XOUT_H, &rawData[0], 6);       // Read the six raw data registers sequentially into data array
		gAvg[0] += (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]) ;  // Turn the MSB and LSB into a signed 16-bit value
		gAvg[1] += (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]) ;
		gAvg[2] += (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]) ;
	}

	for (int ii = 0; ii < 3; ii++) { // Get average of 200 values and store as average current readings
		aAvg[ii] /= 200;
		gAvg[ii] /= 200;
	}

	// Configure the accelerometer for self-test
	_writeReg(MPUREG_ACCEL_CONFIG, 0xE0); // Enable self test on all three axes and set accelerometer range to +/- 2 g
	_writeReg(MPUREG_GYRO_CONFIG,  0xE0); // Enable self test on all three axes and set gyro range to +/- 250 degrees/s
	usleep(25000);  // Delay a while to let the device stabilize

	for (int ii = 0; ii < 200; ii++) {  // get average self-test values of gyro and acclerometer
		_bulkRead(MPUREG_ACCEL_XOUT_H, &rawData[0], 6);  // Read the six raw data registers into data array
		aSTAvg[0] += (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]) ;  // Turn the MSB and LSB into a signed 16-bit value
		aSTAvg[1] += (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]) ;
		aSTAvg[2] += (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]) ;

		_bulkRead(MPUREG_GYRO_XOUT_H, &rawData[0], 6);  // Read the six raw data registers sequentially into data array
		gSTAvg[0] += (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]) ;  // Turn the MSB and LSB into a signed 16-bit value
		gSTAvg[1] += (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]) ;
		gSTAvg[2] += (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]) ;
	}

	for (int ii = 0; ii < 3; ii++) { // Get average of 200 values and store as average self-test readings
		aSTAvg[ii] /= 200;
		gSTAvg[ii] /= 200;
	}

	// Configure the gyro and accelerometer for normal operation
	_writeReg(MPUREG_ACCEL_CONFIG, 0x00);
	_writeReg(MPUREG_GYRO_CONFIG,  0x00);
	usleep(25000);  // Delay a while to let the device stabilize

	// Retrieve accelerometer and gyro factory Self-Test Code from USR_Reg
	_readReg(MPUREG_SELF_TEST_X_ACCEL, selfTest[0]); // X-axis accel self-test results
	DF_LOG_ERR("selfTest[0]: %i", selfTest[0]);
	_readReg(MPUREG_SELF_TEST_Y_ACCEL, selfTest[1]); // Y-axis accel self-test results
	_readReg(MPUREG_SELF_TEST_Z_ACCEL, selfTest[2]); // Z-axis accel self-test results
	_readReg(MPUREG_SELF_TEST_X_GYRO, selfTest[3]);  // X-axis gyro self-test results
	_readReg(MPUREG_SELF_TEST_Y_GYRO, selfTest[4]);  // Y-axis gyro self-test results
	_readReg(MPUREG_SELF_TEST_Z_GYRO, selfTest[5]);  // Z-axis gyro self-test results

	// Retrieve factory self-test value from self-test code reads
	factoryTrim[0] = (float)(2620 / 1 << FS) * (pow(1.01 , ((float)selfTest[0] - 1.0))); // FT[Xa] factory trim calculation
	factoryTrim[1] = (float)(2620 / 1 << FS) * (pow(1.01 , ((float)selfTest[1] - 1.0))); // FT[Ya] factory trim calculation
	factoryTrim[2] = (float)(2620 / 1 << FS) * (pow(1.01 , ((float)selfTest[2] - 1.0))); // FT[Za] factory trim calculation
	factoryTrim[3] = (float)(2620 / 1 << FS) * (pow(1.01 , ((float)selfTest[3] - 1.0))); // FT[Xg] factory trim calculation
	factoryTrim[4] = (float)(2620 / 1 << FS) * (pow(1.01 , ((float)selfTest[4] - 1.0))); // FT[Yg] factory trim calculation
	factoryTrim[5] = (float)(2620 / 1 << FS) * (pow(1.01 , ((float)selfTest[5] - 1.0))); // FT[Zg] factory trim calculation

	// Report results as a ratio of (STR - FT)/FT; the change from Factory Trim of the Self-Test Response
	// To get percent, must multiply by 100
	for (int i = 0; i < 3; i++) {
		DF_LOG_ERR("aSTAvg[%i] = %i", i, aSTAvg[i]);
		DF_LOG_ERR("aAvg[%i] = %i", i, aAvg[i]);
		DF_LOG_ERR("factoryTrim[%i] = %f", i, factoryTrim[i]);
		destination[i]   = 100.0 * (((float)(aSTAvg[i] - aAvg[i])) / factoryTrim[i]) - 100.0; // Report percent differences
		destination[i + 3] = 100.0 * (((float)(gSTAvg[i] - gAvg[i])) / factoryTrim[i + 3]) - 100.0;
	}
}

int MPU9250::mpu9250_init()
{
	// Use 1 MHz for normal registers.
	_setBusFrequency(SPI_FREQUENCY_1MHZ);

	/* Zero the struct */
	m_synchronize.lock();

	m_sensor_data.accel_m_s2_x = 0.0f;
	m_sensor_data.accel_m_s2_y = 0.0f;
	m_sensor_data.accel_m_s2_z = 0.0f;
	m_sensor_data.gyro_rad_s_x = 0.0f;
	m_sensor_data.gyro_rad_s_y = 0.0f;
	m_sensor_data.gyro_rad_s_z = 0.0f;
	m_sensor_data.mag_ga_x = 0.0f;
	m_sensor_data.mag_ga_y = 0.0f;
	m_sensor_data.mag_ga_z = 0.0f;
	m_sensor_data.temp_c = 0.0f;

	m_sensor_data.read_counter = 0;
	m_sensor_data.error_counter = 0;
	m_sensor_data.fifo_overflow_counter = 0;
	m_sensor_data.fifo_corruption_counter = 0;
	m_sensor_data.gyro_range_hit_counter = 0;
	m_sensor_data.accel_range_hit_counter = 0;

	m_sensor_data.fifo_sample_interval_us = 0;
	m_sensor_data.is_last_fifo_sample = false;

	m_synchronize.unlock();

	int result = _writeReg(MPUREG_PWR_MGMT_1, BIT_H_RESET);

	if (result != 0) {
		DF_LOG_ERR("reset failed");
	}

	usleep(100000);

	DF_LOG_INFO("Reset MPU9250");
	result = _writeReg(MPUREG_PWR_MGMT_1, 0);

	if (result != 0) {
		DF_LOG_ERR("wakeup sensor failed");
	}

	usleep(1000);

	result = _writeReg(MPUREG_PWR_MGMT_2, 0);

	if (result != 0) {
		DF_LOG_ERR("enable failed");
	}

	usleep(1000);

	// Reset I2C master and device.
	result = _writeReg(MPUREG_USER_CTRL,
			   BITS_USER_CTRL_I2C_MST_RST |
			   BITS_USER_CTRL_I2C_IF_DIS);

	if (result != 0) {
		DF_LOG_ERR("user ctrl 1 failed");
	}

	usleep(1000);

	// Reset and enable FIFO.
	result = _writeReg(MPUREG_USER_CTRL,
			   BITS_USER_CTRL_FIFO_RST |
			   BITS_USER_CTRL_FIFO_EN);

	if (result != 0) {
		DF_LOG_ERR("user ctrl 2 failed");
	}

	usleep(1000);

	if (_mag_enabled) {
		result = _writeReg(MPUREG_FIFO_EN,
				   BITS_FIFO_ENABLE_TEMP_OUT | BITS_FIFO_ENABLE_GYRO_XOUT
				   | BITS_FIFO_ENABLE_GYRO_YOUT
				   | BITS_FIFO_ENABLE_GYRO_ZOUT | BITS_FIFO_ENABLE_ACCEL
				   | BITS_FIFO_ENABLE_SLV0); // SLV0 is configured for bulk transfer of mag data over I2C

	} else {
		result = _writeReg(MPUREG_FIFO_EN,
				   BITS_FIFO_ENABLE_TEMP_OUT | BITS_FIFO_ENABLE_GYRO_XOUT
				   | BITS_FIFO_ENABLE_GYRO_YOUT
				   | BITS_FIFO_ENABLE_GYRO_ZOUT | BITS_FIFO_ENABLE_ACCEL);
		DF_LOG_INFO("initializing mpu9250 driver without mag support");
	}

	if (result != 0) {
		DF_LOG_ERR("FIFO enable failed");
	}

	usleep(1000);

	/*
	 * A samplerate_divider of 0 should give 1000Hz:
	 *
	 * sample_rate = internal_sample_rate / (1+samplerate_divider)
	 *
	 * This is only used when FCHOICE is 0b11, FCHOICE_B (inverted) 0x00,
	 * therefore commented out.
	 */
	//uint8_t samplerate_divider = 0;
	//result = _writeReg(MPUREG_FIFO_EN, samplerate_divider);
	//if (result != 0) {
	//	DF_LOG_ERR("sample rate config failed");
	//}
	//usleep(1000);

#if defined(__EDISON)
	//Setting the gyro bandwidth to 250 Hz corresponds to
	//8kHz sampling frequency which is too high for the Edison.
	//Therefore, we use the gyro bandwith of 184 Hz which corresponds to 1kHz sampling frequency.
	result = _writeReg(MPUREG_CONFIG,
			   BITS_DLPF_CFG_184HZ | BITS_CONFIG_FIFO_MODE_OVERWRITE);
#else
	result = _writeReg(MPUREG_CONFIG,
			   BITS_DLPF_CFG_250HZ | BITS_CONFIG_FIFO_MODE_OVERWRITE);
#endif

	if (result != 0) {
		DF_LOG_ERR("config failed");
	}

	usleep(1000);

	float deviation[6];
	self_test(deviation);
	DF_LOG_ERR("deviation is: %f, %f, %f, %f, %f, %f", deviation[0],
		   deviation[1], deviation[2], deviation[3],
		   deviation[4], deviation[5]);

	// set_offsets();

	usleep(1000);

	result = _writeReg(MPUREG_GYRO_CONFIG, BITS_FS_2000DPS | BITS_BW_LT3600HZ);

	if (result != 0) {
		DF_LOG_ERR("Gyro scale config failed");
	}

	usleep(1000);

	result = _writeReg(MPUREG_ACCEL_CONFIG, BITS_ACCEL_CONFIG_16G);

	usleep(25000);
	if (result != 0) {
		DF_LOG_ERR("Accel scale config failed");
	}

	usleep(1000);

	result = _writeReg(MPUREG_ACCEL_CONFIG2, BITS_ACCEL_CONFIG2_BW_1130HZ);

	if (result != 0) {
		DF_LOG_ERR("Accel scale config2 failed");
	}

	usleep(1000);

	// Initialize the magnetometer inside the IMU, if enabled by the caller.
	if (_mag_enabled && _mag == nullptr) {
		if ((_mag = new MPU9250_mag(*this, MPU9250_MAG_SAMPLE_RATE_100HZ))
		    != nullptr) {
			// Initialize the magnetometer, providing the output data rate for
			// data read from the IMU FIFO.  This is used to calculate the I2C
			// delay for reading the magnetometer.
			result = _mag->initialize(MPU9250_MEASURE_INTERVAL_US);

			if (result != 0) {
				DF_LOG_ERR("Magnetometer initialization failed");
			}

		} else {
			DF_LOG_ERR("Allocation of magnetometer object failed.");
		}
	}

	// Enable/clear the FIFO of any residual data
	reset_fifo();

	// Clear Interrupt Status
	clear_int_status();

	return 0;
}

int MPU9250::mpu9250_deinit()
{
	// Leave the IMU in a reset state (turned off).
	int result = _writeReg(MPUREG_PWR_MGMT_1, BIT_H_RESET);

	if (result != 0) {
		DF_LOG_ERR("reset failed");
	}

	// Deallocate the resources for the mag driver, if enabled.
	if (_mag_enabled && _mag != nullptr) {
		delete _mag;
		_mag = nullptr;
	}

	return 0;
}

int MPU9250::start()
{
	/* Open the device path specified in the class initialization. */
	// attempt to open device in start()
	int result = SPIDevObj::start();

	if (result != 0) {
		DF_LOG_ERR("DevObj start failed");
		DF_LOG_ERR("Unable to open the device path: %s", m_dev_path);
		return result;
	}

	/* Set the bus frequency for register get/set. */
	result = _setBusFrequency(SPI_FREQUENCY_1MHZ);

	if (result != 0) {
		DF_LOG_ERR("failed setting SPI bus frequency: %d", result);
	}

	/* Try to talk to the sensor. */
	uint8_t sensor_id;
	result = _readReg(MPUREG_WHOAMI, sensor_id);

	if (result != 0) {
		DF_LOG_ERR("Unable to communicate with the MPU9250 sensor");
		goto exit;
	}

	if (sensor_id != MPU_WHOAMI_9250) {
		DF_LOG_ERR("MPU9250 sensor WHOAMI wrong: 0x%X, should be: 0x%X",
			   sensor_id, MPU_WHOAMI_9250);
		result = -1;
		goto exit;
	}

	result = mpu9250_init();

	if (result != 0) {
		DF_LOG_ERR("error: IMU sensor initialization failed, sensor read thread not started");
		goto exit;
	}

	result = DevObj::start();

	if (result != 0) {
		DF_LOG_ERR("DevObj start failed");
		return result;
	}

exit:
	return result;
}

int MPU9250::stop()
{
	int result = mpu9250_deinit();

	if (result != 0) {
		DF_LOG_ERR(
			"error: IMU sensor de-initialization failed.");
		return result;
	}

	result = DevObj::stop();

	if (result != 0) {
		DF_LOG_ERR("DevObj stop failed");
		return result;
	}

	// We need to wait so that all measure calls are finished before
	// closing the device.
	usleep(10000);

	return 0;
}

int MPU9250::get_fifo_count()
{
	int16_t num_bytes = 0x0;

	// Use 1 MHz for normal registers.
	_setBusFrequency(SPI_FREQUENCY_1MHZ);
	int ret = _bulkRead(MPUREG_FIFO_COUNTH, (uint8_t *) &num_bytes,
			    sizeof(num_bytes));

	if (ret == 0) {

		/* TODO: add ifdef for endianness */
		num_bytes = swap16(num_bytes);

		return num_bytes;

	} else {
		DF_LOG_ERR("FIFO count read failed");
		return ret;
	}
}

void MPU9250::reset_fifo()
{
	// Use 1 MHz for normal registers.
	_setBusFrequency(SPI_FREQUENCY_1MHZ);

	int result;

	result = _modifyReg(MPUREG_USER_CTRL,
			    0,
			    BITS_USER_CTRL_FIFO_RST |
			    BITS_USER_CTRL_FIFO_EN);

	if (result != 0) {
		DF_LOG_ERR("FIFO reset failed");
	}
}

void MPU9250::clear_int_status()
{
	int result;
	uint8_t int_status = 0;

	result = _readReg(MPUREG_INT_STATUS, int_status);

	if (result != 0) {
		DF_LOG_ERR("Interrupt status clear failed");
	}
}

void MPU9250::_measure()
{
	// Use 1 MHz for normal registers.
	_setBusFrequency(SPI_FREQUENCY_1MHZ);
	uint8_t int_status = 0;
	int result = _readReg(MPUREG_INT_STATUS, int_status);

	if (result != 0) {
		m_synchronize.lock();
		++m_sensor_data.error_counter;
		m_synchronize.unlock();
		return;
	}

	if (int_status & BITS_INT_STATUS_FIFO_OVERFLOW) {
		reset_fifo();

		m_synchronize.lock();
		++m_sensor_data.fifo_overflow_counter;
		DF_LOG_ERR("FIFO overflow");
		m_synchronize.unlock();

		return;
	}

	int size_of_fifo_packet;

	if (_mag_enabled) {
		size_of_fifo_packet = sizeof(fifo_packet_with_mag);

	} else {
		size_of_fifo_packet = sizeof(fifo_packet);
	}

	// Get FIFO byte count to read and floor it to the report size.
	int bytes_to_read = get_fifo_count() / size_of_fifo_packet
			    * size_of_fifo_packet;

	// It looks like the FIFO doesn't actually deliver at 8kHz like it is supposed to.
	// Therefore, we need to adapt the interval which we pass on to the integrator.
	// The filtering is to lower the jitter that could result through the calculation
	// because of the fact that the bytes we fetch per _measure() cycle varies.
	_packets_per_cycle_filtered = (0.95f * _packets_per_cycle_filtered) + (0.05f * (bytes_to_read / size_of_fifo_packet));

	if (bytes_to_read < 0) {
		m_synchronize.lock();
		++m_sensor_data.error_counter;
		m_synchronize.unlock();
		return;
	}

	// Allocate a buffer large enough for n complete packets, read from the
	// sensor FIFO.
	const unsigned buf_len = (MPU_MAX_LEN_FIFO_IN_BYTES / size_of_fifo_packet) * size_of_fifo_packet;
	uint8_t fifo_read_buf[buf_len];

	if (bytes_to_read <= 0) {
		m_synchronize.lock();
		++m_sensor_data.error_counter;
		m_synchronize.unlock();
		return;
	}

	const unsigned read_len = MIN((unsigned)bytes_to_read, buf_len);
	memset(fifo_read_buf, 0x0, buf_len);

	// According to the protocol specs, all sensor and interrupt registers may be read at 20 MHz.
	// It is unclear what rate the FIFO register can be read at.
	// If the FIFO buffer was read at 20 MHz, two effects were seen:
	// - The buffer is off-by-one. So the report "starts" at &fifo_read_buf[i+1].
	// - Also, the FIFO buffer seemed to prone to random corruption (or shifting), unless
	//   all other sensors ran very smooth. (E.g. Changing the bus speed of the HMC5883 driver from
	//   400 kHz to 100 kHz could cause corruption because this driver wouldn't run as regularly.
	//
	// Luckily 10 MHz seems to work fine.

#if defined(__EDISON)
	//FIFO corrupt at 10MHz.
	_setBusFrequency(SPI_FREQUENCY_5MHZ);
#else
	_setBusFrequency(SPI_FREQUENCY_10MHZ);
#endif

	result = _bulkRead(MPUREG_FIFO_R_W, fifo_read_buf, read_len);

	if (result != 0) {
		m_synchronize.lock();
		++m_sensor_data.error_counter;
		m_synchronize.unlock();
		return;
	}

	for (unsigned packet_index = 0; packet_index < read_len / size_of_fifo_packet; ++packet_index) {

		fifo_packet *report = (fifo_packet *)(&fifo_read_buf[packet_index	* size_of_fifo_packet]);

		/* TODO: add ifdef for endianness */
		report->accel_x = swap16(report->accel_x);
		report->accel_y = swap16(report->accel_y);
		report->accel_z = swap16(report->accel_z);
		report->temp = swap16(report->temp);
		report->gyro_x = swap16(report->gyro_x);
		report->gyro_y = swap16(report->gyro_y);
		report->gyro_z = swap16(report->gyro_z);


		// Check if the full accel range of the accel has been used. If this occurs, it is
		// either a spike due to a crash/landing or a sign that the vibrations levels
		// measured are excessive.
		if (report->accel_x == INT16_MIN || report->accel_x == INT16_MAX ||
		    report->accel_y == INT16_MIN || report->accel_y == INT16_MAX ||
		    report->accel_z == INT16_MIN || report->accel_z == INT16_MAX) {
			m_synchronize.lock();
			++m_sensor_data.accel_range_hit_counter;
			m_synchronize.unlock();
		}

		// Also check the full gyro range, however, this is very unlikely to happen.
		if (report->gyro_x == INT16_MIN || report->gyro_x == INT16_MAX ||
		    report->gyro_y == INT16_MIN || report->gyro_y == INT16_MAX ||
		    report->gyro_z == INT16_MIN || report->gyro_z == INT16_MAX) {
			m_synchronize.lock();
			++m_sensor_data.gyro_range_hit_counter;
			m_synchronize.unlock();
		}

		const float temp_c = float(report->temp) / 361.0f + 35.0f;

		// Use the temperature field to try to detect if we (ever) fall out of sync with
		// the FIFO buffer. If the temperature changes insane amounts, reset the FIFO logic
		// and return early.
		if (!_temp_initialized) {
			// Assume that the temperature should be in a sane range of -40 to 85 deg C which is
			// the specified temperature range, at least to initialize.
			if (temp_c > -40.0f && temp_c < 85.0f) {

				// Initialize the temperature logic.
				_last_temp_c = temp_c;
				DF_LOG_INFO("IMU temperature initialized to: %f", (double) temp_c);
				_temp_initialized = true;
			}

		} else {
			// Once initialized, check for a temperature change of more than 2 degrees which
			// points to a FIFO corruption.
			if (fabsf(temp_c - _last_temp_c) > 2.0f) {
				DF_LOG_ERR(
					"FIFO corrupt, temp difference: %f, last temp: %f, current temp: %f",
					fabs(temp_c - _last_temp_c), (double)_last_temp_c, (double)temp_c);
				reset_fifo();
				_temp_initialized = false;
				m_synchronize.lock();
				++m_sensor_data.fifo_corruption_counter;
				m_synchronize.unlock();
				return;
			}

			_last_temp_c = temp_c;
		}

		m_synchronize.lock();
		m_sensor_data.accel_m_s2_x = float(report->accel_x)
					     * (MPU9250_ONE_G / 2048.0f);
		m_sensor_data.accel_m_s2_y = float(report->accel_y)
					     * (MPU9250_ONE_G / 2048.0f);
		m_sensor_data.accel_m_s2_z = float(report->accel_z)
					     * (MPU9250_ONE_G / 2048.0f);
		m_sensor_data.temp_c = temp_c;
		m_sensor_data.gyro_rad_s_x = float(report->gyro_x) * GYRO_RAW_TO_RAD_S;
		m_sensor_data.gyro_rad_s_y = float(report->gyro_y) * GYRO_RAW_TO_RAD_S;
		m_sensor_data.gyro_rad_s_z = float(report->gyro_z) * GYRO_RAW_TO_RAD_S;

		if (_mag_enabled) {
			struct fifo_packet_with_mag *report_with_mag_data = (struct fifo_packet_with_mag *)report;

			int mag_error = _mag->process((const struct mag_data &)report_with_mag_data->mag_st1,
						      m_sensor_data.mag_ga_x,
						      m_sensor_data.mag_ga_y,
						      m_sensor_data.mag_ga_z);

			if (mag_error == MAG_ERROR_DATA_OVERFLOW) {
				m_sensor_data.mag_fifo_overflow_counter++;
			}
		}

		// Pass on the sampling interval between FIFO samples at 8kHz.
		m_sensor_data.fifo_sample_interval_us = 1000000 / MPU9250_MEASURE_INTERVAL_US
							/ _packets_per_cycle_filtered;

		// Flag if this is the last sample, and _publish() should wrap up the data it has received.
		m_sensor_data.is_last_fifo_sample = ((packet_index + 1) == (read_len / size_of_fifo_packet));

		++m_sensor_data.read_counter;

		// Generate debug output every second, assuming that a sample is generated every
		// 125 usecs
#ifdef MPU9250_DEBUG

		if (++m_sensor_data.read_counter % (1000000 / m_sensor_data.fifo_sample_interval_us) == 0) {

			DF_LOG_INFO("IMU: accel: [%f, %f, %f]",
				    (double)m_sensor_data.accel_m_s2_x,
				    (double)m_sensor_data.accel_m_s2_y,
				    (double)m_sensor_data.accel_m_s2_z);
			DF_LOG_INFO("     gyro:  [%f, %f, %f]",
				    (double)m_sensor_data.gyro_rad_s_x,
				    (double)m_sensor_data.gyro_rad_s_y,
				    (double)m_sensor_data.gyro_rad_s_z);
			DF_LOG_INFO("    temp:  %f C", (double)m_sensor_data.temp_c);
		}

#endif

#ifdef MPU9250_DEBUG

		if (_mag_enabled && mag_error == 0) {
			if ((m_sensor_data.read_counter % 10000) == 0) {
				DF_LOG_INFO("     mag:  [%f, %f, %f] ga",
					    m_sensor_data.mag_ga_x, m_sensor_data.mag_ga_y, m_sensor_data.mag_ga_z);
			}
		}

#endif

		_publish(m_sensor_data);

		m_synchronize.signal();
		m_synchronize.unlock();
	}
}

int MPU9250::_publish(struct imu_sensor_data &data)
{
	// TBD
	return -1;
}

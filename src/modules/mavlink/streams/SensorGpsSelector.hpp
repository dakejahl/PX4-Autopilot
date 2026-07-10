/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
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

#ifndef SENSOR_GPS_SELECTOR_HPP
#define SENSOR_GPS_SELECTOR_HPP

#include <lib/drivers/device/Device.hpp>
#include <lib/parameters/param.h>
#include <uORB/Subscription.hpp>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/sensor_gps.h>

// Resolves SENS_GPS_PRIME to the sensor_gps uORB instance of the primary receiver,
// mirroring the vehicle_gps_position selection: 0 and 1 select the uORB instance
// directly, 2-127 select a DroneCAN receiver by node ID (matched via device_id).
// Used by GPS_RAW_INT/GPS2_RAW so the reported receiver does not depend on uORB
// instance ordering, which is a boot-order race for CAN receivers.
class SensorGpsSelector
{
public:
	SensorGpsSelector()
	{
		read_param();
	}

	// uORB instance of the primary receiver (0 or 1)
	uint8_t primary_instance()
	{
		if (_parameter_update_sub.updated()) {
			parameter_update_s parameter_update;
			_parameter_update_sub.copy(&parameter_update);
			read_param();
		}

		if ((_gps_prime == 0) || (_gps_prime == 1)) {
			return _gps_prime;
		}

		if ((_gps_prime >= 2) && (_gps_prime <= 127)) {
			// DroneCAN node ID: keep looking until the receiver has published
			if (_node_id_instance < 0) {
				_node_id_instance = find_instance_by_node_id(_gps_prime);
			}

			if (_node_id_instance >= 0) {
				return _node_id_instance;
			}
		}

		// -1 (auto) or not resolvable
		return 0;
	}

private:
	void read_param()
	{
		_gps_prime = 0;
		_node_id_instance = -1;

		if (_param_sens_gps_prime != PARAM_INVALID) {
			param_get(_param_sens_gps_prime, &_gps_prime);
		}
	}

	static int8_t find_instance_by_node_id(int32_t node_id)
	{
		for (uint8_t i = 0; i < 2; i++) {
			uORB::Subscription sensor_gps_sub{ORB_ID(sensor_gps), i};
			sensor_gps_s gps;

			if (sensor_gps_sub.copy(&gps)) {
				device::Device::DeviceId device_id{};
				device_id.devid = gps.device_id;

				if ((device_id.devid_s.bus_type == device::Device::DeviceBusType_UAVCAN)
				    && (device_id.devid_s.address == node_id)) {
					return i;
				}
			}
		}

		return -1;
	}

	uORB::Subscription _parameter_update_sub{ORB_ID(parameter_update)};
	const param_t _param_sens_gps_prime{param_find("SENS_GPS_PRIME")};
	int32_t _gps_prime{0};
	int8_t _node_id_instance{-1};
};

#endif // SENSOR_GPS_SELECTOR_HPP

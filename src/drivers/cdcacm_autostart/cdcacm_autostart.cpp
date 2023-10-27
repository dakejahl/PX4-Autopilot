/****************************************************************************
 *
 *   Copyright (c) 2023 PX4 Development Team. All rights reserved.
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

#include "cdcacm_autostart.h"

__BEGIN_DECLS
#include <arch/board/board.h>
#include <builtin/builtin.h>
#include <termios.h>

extern int sercon_main(int c, char **argv);
extern int serdis_main(int c, char **argv);
__END_DECLS

#include <px4_platform_common/shutdown.h>

#define USB_DEVICE_PATH "/dev/ttyACM0"

#if defined(CONFIG_SERIAL_PASSTHRU_UBLOX)
#  undef SERIAL_PASSTHRU_UBLOX_DEV
#  if defined(CONFIG_SERIAL_PASSTHRU_GPS1) && defined(CONFIG_BOARD_SERIAL_GPS1)
#    define SERIAL_PASSTHRU_UBLOX_DEV CONFIG_BOARD_SERIAL_GPS1
#  elif defined(CONFIG_SERIAL_PASSTHRU_GPS2)&& defined(CONFIG_BOARD_SERIAL_GPS2)
#    define SERIAL_PASSTHRU_UBLOX_DEV CONFIG_BOARD_SERIAL_GPS2
#  elif defined(CONFIG_SERIAL_PASSTHRU_GPS3)&& defined(CONFIG_BOARD_SERIAL_GPS3)
#    define SERIAL_PASSTHRU_UBLOX_DEV CONFIG_BOARD_SERIAL_GPS3
#  elif defined(CONFIG_SERIAL_PASSTHRU_GPS4)&& defined(CONFIG_BOARD_SERIAL_GPS4)
#    define SERIAL_PASSTHRU_UBLOX_DEV CONFIG_BOARD_SERIAL_GPS4
#  elif defined(CONFIG_SERIAL_PASSTHRU_GPS5) && defined(CONFIG_BOARD_SERIAL_GPS5)
#    define SERIAL_PASSTHRU_UBLOX_DEV CONFIG_BOARD_SERIAL_GPS5
#  endif
#  if !defined(SERIAL_PASSTHRU_UBLOX_DEV)
#    error "CONFIG_SERIAL_PASSTHRU_GPSn and CONFIG_BOARD_SERIAL_GPSn must be defined"
#  endif
#endif

CdcAcmAutostart::CdcAcmAutostart() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::lp_default)
{}

CdcAcmAutostart::~CdcAcmAutostart()
{
	if (_ttyacm_fd >= 0) {
		close(_ttyacm_fd);
	}

	ScheduleClear();
}

int CdcAcmAutostart::Start()
{
	PX4_INFO("Starting CDC/ACM autostart");
	UpdateParams(true);

	ScheduleNow();

	return PX4_OK;
}

void CdcAcmAutostart::Run()
{
	if (should_exit()) {
		exit_and_cleanup();
		return;
	}

	UpdateParams();

	mavlink_usb_check();
}

void CdcAcmAutostart::mavlink_usb_check()
{
	bool rescheduled = false;

	actuator_armed_s report;
	_actuator_armed_sub.copy(&report);

	const bool armed = report.armed;
	bool vbus_present = (board_read_VBUS_state() == PX4_OK);
	bool locked_out = false;

	// If the hardware support RESET lockout that has nArmed ANDed with VBUS
	// vbus_sense may drop during a param save which uses
	// BOARD_INDICATE_EXTERNAL_LOCKOUT_STATE to prevent external resets
	// while writing the params.  If we are not armed and nARMRED is low
	// we are in such a lock out so ignore changes on VBUS_SENSE during this
	// time.
#if defined(BOARD_GET_EXTERNAL_LOCKOUT_STATE)
	locked_out = BOARD_GET_EXTERNAL_LOCKOUT_STATE() == 0;

	if (locked_out) {
		vbus_present = _vbus_present_prev;
	}

#endif


	if (!armed && !locked_out) {
		switch (_state) {
		case UsbAutoStartState::disconnected:
			if (vbus_present && _vbus_present_prev) {
				if (sercon_main(0, nullptr) == EXIT_SUCCESS) {
					_state = UsbAutoStartState::connecting;
					ScheduleDelayed(100_ms);
					rescheduled = true;
				}

			} else if (vbus_present && !_vbus_present_prev) {
				// check again sooner if USB just connected
				ScheduleDelayed(100_ms);
				rescheduled = true;
			}

			break;

		case UsbAutoStartState::connecting:
			if (vbus_present) {
				if (_ttyacm_fd < 0) {
					_ttyacm_fd = ::open(USB_DEVICE_PATH, O_RDONLY | O_NONBLOCK);
				}

				if (_ttyacm_fd >= 0) {
					int bytes_available = 0;
					int retval = ::ioctl(_ttyacm_fd, FIONREAD, &bytes_available);

					if ((retval == OK) && (bytes_available >= 3)) {
						char buffer[80];

						// non-blocking read
						int nread = ::read(_ttyacm_fd, buffer, sizeof(buffer));

#if defined(DEBUG_BUILD)

						if (nread > 0) {
							fprintf(stderr, "%d bytes\n", nread);

							for (int i = 0; i < nread; i++) {
								fprintf(stderr, "|%X", buffer[i]);
							}

							fprintf(stderr, "\n");
						}

#endif // DEBUG_BUILD


						if (nread > 0) {
							// Mavlink reboot/shutdown command
							// COMMAND_LONG (#76) with command MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN (246)
							static constexpr int MAVLINK_COMMAND_LONG_MIN_LENGTH = 41;

							if (nread >= MAVLINK_COMMAND_LONG_MIN_LENGTH) {
								// scan buffer for mavlink COMMAND_LONG
								for (int i = 0; i < nread - MAVLINK_COMMAND_LONG_MIN_LENGTH; i++) {
									if ((buffer[i] == 0xFE)        // Mavlink v1 start byte
									    && (buffer[i + 5] == 76)   //  76=0x4C COMMAND_LONG
									    && (buffer[i + 34] == 246) // 246=0xF6 MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN
									   ) {
										// mavlink v1 COMMAND_LONG
										//  buffer[0]: start byte (0xFE for mavlink v1)
										//  buffer[3]: SYSID
										//  buffer[4]: COMPID
										//  buffer[5]: message id (COMMAND_LONG 76=0x4C)
										//  buffer[6-10]: COMMAND_LONG param 1 (little endian float)
										//  buffer[34]: COMMAND_LONG command MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN (246/0xF6)
										float param1_raw = 0;
										memcpy(&param1_raw, &buffer[i + 6], 4);
										int param1 = roundf(param1_raw);

										PX4_INFO("%s: Mavlink MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN param 1: %d (SYSID:%d COMPID:%d)",
											 USB_DEVICE_PATH, param1, buffer[i + 3], buffer[i + 4]);

										if (param1 == 1) {
											// 1: Reboot autopilot
											px4_reboot_request(false, 0);

										} else if (param1 == 2) {
											// 2: Shutdown autopilot
#if defined(BOARD_HAS_POWER_CONTROL)
											px4_shutdown_request(0);
#endif // BOARD_HAS_POWER_CONTROL

										} else if (param1 == 3) {
											// 3: Reboot autopilot and keep it in the bootloader until upgraded.
											px4_reboot_request(true, 0);
										}
									}
								}
							}


							bool launch_mavlink = false;
							bool launch_nshterm = false;
							bool launch_passthru = false;
							struct termios uart_config;
							static constexpr int MAVLINK_HEARTBEAT_MIN_LENGTH = 9;

							if (nread >= MAVLINK_HEARTBEAT_MIN_LENGTH) {
								// scan buffer for mavlink HEARTBEAT (v1 & v2)
								for (int i = 0; i < nread - MAVLINK_HEARTBEAT_MIN_LENGTH; i++) {
									if ((buffer[i] == 0xFE) && (buffer[i + 1] == 9) && (buffer[i + 5] == 0)) {
										// mavlink v1 HEARTBEAT
										//  buffer[0]: start byte (0xFE for mavlink v1)
										//  buffer[1]: length (9 for HEARTBEAT)
										//  buffer[3]: SYSID
										//  buffer[4]: COMPID
										//  buffer[5]: mavlink message id (0 for HEARTBEAT)
										PX4_INFO("%s: launching mavlink (HEARTBEAT v1 from SYSID:%d COMPID:%d)",
											 USB_DEVICE_PATH, buffer[i + 3], buffer[i + 4]);
										launch_mavlink = true;
										break;

									} else if ((buffer[i] == 0xFD) && (buffer[i + 1] == 9)
										   && (buffer[i + 7] == 0) && (buffer[i + 8] == 0) && (buffer[i + 9] == 0)) {
										// mavlink v2 HEARTBEAT
										//  buffer[0]: start byte (0xFD for mavlink v2)
										//  buffer[1]: length (9 for HEARTBEAT)
										//  buffer[5]: SYSID
										//  buffer[6]: COMPID
										//  buffer[7:9]: mavlink message id (0 for HEARTBEAT)
										PX4_INFO("%s: launching mavlink (HEARTBEAT v2 from SYSID:%d COMPID:%d)",
											 USB_DEVICE_PATH, buffer[i + 5], buffer[i + 6]);
										launch_mavlink = true;
										break;
									}
								}
							}

							if (!launch_mavlink && (nread >= 3)) {
								// nshterm (3 carriage returns)
								// scan buffer looking for 3 consecutive carriage returns (0xD)
								for (int i = 1; i < nread - 1; i++) {
									if (buffer[i - 1] == 0xD && buffer[i] == 0xD && buffer[i + 1] == 0xD) {
										PX4_INFO("%s: launching nshterm", USB_DEVICE_PATH);
										launch_nshterm = true;
										break;
									}
								}
							}

#if defined(CONFIG_SERIAL_PASSTHRU_UBLOX)

							if (!launch_mavlink && !launch_nshterm && (nread >= 4)) {
								// passthru Ublox
								// scan buffer looking for 0xb5 0x62
								for (int i = 0; i < nread; i++) {
									bool ub = buffer[i] == 0xb5 && buffer[i + 1] == 0x62;

									if (ub && ((buffer[i + 2 ] == 0x6 && (buffer[i + 3 ] == 0xb8 || buffer[i + 3 ] == 0x13)) ||
										   (buffer[i + 2 ] == 0xa && buffer[i + 3 ] == 0x4))) {
										PX4_INFO("%s: launching serial_passthru", USB_DEVICE_PATH);
										launch_passthru = true;
										break;
									}
								}
							}

#endif

							if (launch_mavlink || launch_nshterm || launch_passthru) {

								// Get the current settings
								tcgetattr(_ttyacm_fd, &uart_config);

								// cleanup serial port
								close(_ttyacm_fd);
								_ttyacm_fd = -1;

								char mavlink_mode_string[3];
								snprintf(mavlink_mode_string, sizeof(mavlink_mode_string), "%ld", _mav_usb_mode.get());

								static const char *mavlink_argv[] {"mavlink", "start", "-d", USB_DEVICE_PATH, "-m", mavlink_mode_string, nullptr};
								static const char *nshterm_argv[] {"nshterm", USB_DEVICE_PATH, nullptr};
#if defined(CONFIG_SERIAL_PASSTHRU_UBLOX)
								speed_t baudrate = cfgetspeed(&uart_config);
								char baudstring[16];
								snprintf(baudstring, sizeof(baudstring), "%ld", baudrate);
								static const char *gps_argv[] {"gps", "stop", nullptr};

								static const char *passthru_argv[] {"serial_passthru", "start", "-t", "-b", baudstring, "-e", USB_DEVICE_PATH, "-d", SERIAL_PASSTHRU_UBLOX_DEV,   nullptr};
#endif
								char **exec_argv = nullptr;

								if (launch_nshterm) {
									exec_argv = (char **)nshterm_argv;

								} else if (launch_mavlink) {
									exec_argv = (char **)mavlink_argv;
								}

#if defined(CONFIG_SERIAL_PASSTHRU_UBLOX)

								else if (launch_passthru) {
									sched_lock();
									exec_argv = (char **)gps_argv;
									exec_builtin(exec_argv[0], exec_argv, nullptr, 0);
									sched_unlock();
									exec_argv = (char **)passthru_argv;
								}

#endif

								sched_lock();

								if (exec_builtin(exec_argv[0], exec_argv, nullptr, 0) > 0) {
									_state = UsbAutoStartState::connected;

								} else {
									_state = UsbAutoStartState::disconnecting;
								}

								sched_unlock();
							}
						}
					}
				}

			} else {
				// cleanup
				if (_ttyacm_fd >= 0) {
					close(_ttyacm_fd);
					_ttyacm_fd = -1;
				}

				_state = UsbAutoStartState::disconnecting;
			}

			break;

		case UsbAutoStartState::connected:
			if (!vbus_present && !_vbus_present_prev) {
				sched_lock();
				static const char app[] {"mavlink"};
				static const char *stop_argv[] {"mavlink", "stop", "-d", USB_DEVICE_PATH, NULL};
				exec_builtin(app, (char **)stop_argv, NULL, 0);
				sched_unlock();

				_state = UsbAutoStartState::disconnecting;
			}

			break;

		case UsbAutoStartState::disconnecting:
			// serial disconnect if unused
			serdis_main(0, NULL);
			_state = UsbAutoStartState::disconnected;
			break;
		}
	}

	_vbus_present_prev = vbus_present;

	if (!rescheduled) {
		ScheduleDelayed(500_ms);
	}
}

int CdcAcmAutostart::task_spawn(int argc, char *argv[])
{
#if defined(CONFIG_SYSTEM_CDCACM)
	CdcAcmAutostart *instance = new CdcAcmAutostart();

	if (!instance) {
		PX4_ERR("alloc failed");
		return -1;
	}

	int ret = instance->Start();

	if (ret != PX4_OK) {
		delete instance;
		return ret;
	}

	_object.store(instance);
	_task_id = task_id_is_work_queue;

	return ret;
#else
	PX4_ERR("CDC/ACM not supported");
#endif
}

void CdcAcmAutostart::UpdateParams(const bool force)
{
	if (_parameter_update_sub.updated() || force) {
		// clear update
		parameter_update_s param_update;
		_parameter_update_sub.copy(&param_update);

		// update parameters from storage
		ModuleParams::updateParams();
	}
}

int CdcAcmAutostart::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int CdcAcmAutostart::print_usage(const char *reason)
{
	if (reason) {
		printf("%s\n\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
This module listens on USB and auto-configures the protocol depending on the bytes received.
The supported protocols are: MAVLink, nsh, and ublox serial passthrough.
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("cdcacm_autostart", "system");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int cdcacm_autostart_main(int argc, char *argv[])
{
	return CdcAcmAutostart::main(argc, argv);
}

/****************************************************************************
 *
 * Copyright (C) 2019-2021 PX4 Development Team. All rights reserved.
 * Author: Igor Misic <igy1000mb@gmail.com>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *  notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *  notice, this list of conditions and the following disclaimer in
 *  the documentation and/or other materials provided with the
 *  distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *  used to endorse or promote products derived from this software
 *  without specific prior written permission.
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


#if (CONFIG_STM32_HAVE_IP_DMA_V1)
//Do nothing. IP DMA V1 MCUs are not supported.
#else

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/micro_hal.h>
#include <stm32_dma.h>
#include <stm32_tim.h>
#include <px4_arch/dshot.h>
#include <px4_arch/io_timer.h>
#include <drivers/drv_dshot.h>

#include <px4_platform_common/log.h>
#include <stdio.h>
#include <drivers/drv_input_capture.h>

// DShot protocol definitions
#define ONE_MOTOR_DATA_SIZE         16u
#define MOTOR_PWM_BIT_1             14u
#define MOTOR_PWM_BIT_0             7u
#define DSHOT_THROTTLE_POSITION     5u
#define DSHOT_TELEMETRY_POSITION    4u
#define NIBBLES_SIZE                4u
#define DSHOT_NUMBER_OF_NIBBLES     3u
#define MAX_NUM_CHANNELS_PER_TIMER  4u // CCR1-CCR4

// DMA stream configuration registers
#define DSHOT_DMA_SCR (DMA_SCR_PRIHI | DMA_SCR_MSIZE_32BITS | DMA_SCR_PSIZE_32BITS | DMA_SCR_MINC | \
		       DMA_SCR_DIR_M2P | DMA_SCR_TCIE | DMA_SCR_TEIE | DMA_SCR_DMEIE)

// 16-bit because not all of the General Purpose Timers support 32-bit
#define DSHOT_BIDIRECTIONAL_DMA_SCR (DMA_SCR_PRIHI | DMA_SCR_MSIZE_16BITS | DMA_SCR_PSIZE_16BITS | DMA_SCR_MINC | \
				     DMA_SCR_DIR_P2M | DMA_SCR_TCIE | DMA_SCR_TEIE | DMA_SCR_DMEIE)

#if defined(CONFIG_ARMV7M_DCACHE)
#  define DMA_BUFFER_MASK    (ARMV7M_DCACHE_LINESIZE - 1)
#  define DMA_ALIGN_UP(n)    (((n) + DMA_BUFFER_MASK) & ~DMA_BUFFER_MASK)
#else
#define DMA_ALIGN_UP(n) (n)
#endif

#define CHANNEL_OUTPUT_BUFF_SIZE    17u
#define CHANNEL_CAPTURE_BUFF_SIZE   32u

#define DSHOT_OUTPUT_BUFFER_SIZE(channel_count) (DMA_ALIGN_UP(sizeof(uint32_t) * CHANNEL_OUTPUT_BUFF_SIZE * channel_count))
#define DSHOT_CAPTURE_BUFFER_SIZE(channel_count) (DMA_ALIGN_UP(sizeof(uint16_t)* CHANNEL_CAPTURE_BUFF_SIZE * channel_count))

// Holds DMA handles (UP / CH1-4)
typedef struct timer_dma_handles_t {
	DMA_HANDLE dma_up_handle;       // DMA stream for DMA update
	DMA_HANDLE dma_ch_handle[4];    // DMA streams for bidi capture compare
} timer_dma_handles_t;

static timer_dma_handles_t timer_dma_handles[MAX_IO_TIMERS] = {};

// Output buffer of interleaved motor output bytes
static uint8_t dshot_burst_buffer_array[MAX_IO_TIMERS * DSHOT_OUTPUT_BUFFER_SIZE(MAX_NUM_CHANNELS_PER_TIMER)]
px4_cache_aligned_data() = {};
static uint32_t *dshot_output_buffer[MAX_IO_TIMERS] = {};

// Array of buffers for timer/channel erpm data
static uint16_t dshot_capture_buffer[MAX_IO_TIMERS][MAX_NUM_CHANNELS_PER_TIMER][CHANNEL_CAPTURE_BUFF_SIZE]
px4_cache_aligned_data() = {};

static void dma_start_callback(DMA_HANDLE handle, uint8_t status, void *arg);
static void cc_callback_timer_index_0(void *arg);
static void cc_callback_timer_index_1(void *arg);
static void cc_callback_timer_index_2(void *arg);

static void process_capture_results(uint8_t timer_index);
static unsigned calculate_period(uint8_t timer_index, uint8_t channel_index);

static bool     _bidirectional = false;
static uint32_t _dshot_frequency = 0;
static int      _timers_init_mask = 0;
static int      _channels_init_mask = 0;

static uint8_t _timer_index_array[MAX_IO_TIMERS] = {};
static int32_t _erpms[MAX_IO_TIMERS][MAX_NUM_CHANNELS_PER_TIMER] = {};

// We need local permanent memory for indices, channels, and callbacks for each output
static struct hrt_call _dma_capture_callback_call[MAX_TIMER_IO_CHANNELS];

// Separate HRT callbacks for each timer
typedef void (*hrt_callback_t)(void *arg);
static hrt_callback_t _cc_callbacks[3] = {
	cc_callback_timer_index_0,
	cc_callback_timer_index_1,
	cc_callback_timer_index_2,
};

// decoding status
static uint32_t read_ok = 0;
static uint32_t read_fail_nibble = 0;
static uint32_t read_fail_crc = 0;
static uint32_t read_fail_zero = 0;
static uint32_t delay_hrt = 0;

int up_dshot_init(uint32_t channel_mask, unsigned dshot_pwm_freq, bool enable_bidirectional_dshot)
{
	_bidirectional = enable_bidirectional_dshot;
	_dshot_frequency = dshot_pwm_freq;

	// Initialize the timer channels
	for (unsigned output_channel = 0; output_channel < MAX_TIMER_IO_CHANNELS; output_channel++) {
		if (channel_mask & (1 << output_channel)) {
			uint8_t timer_index = timer_io_channels[output_channel].timer_index;

			if (io_timers[timer_index].dshot.dma_base == 0) {
				// board does not configure dshot on this timer
				continue;
			}

			io_timer_channel_mode_t mode = _bidirectional ? IOTimerChanMode_DshotInverted : IOTimerChanMode_Dshot;
			int ret = io_timer_channel_init(output_channel, mode, NULL, NULL);

			if (ret != OK) {
				PX4_INFO("io_timer_channel_init %u failed", output_channel);
				return ret;
			}

			_channels_init_mask |= (1 << output_channel);
			_timers_init_mask |= (1 << timer_index);
		}
	}

	// For each dshot timer, test the DMA UP configuration.
	for (uint8_t timer_index = 0; timer_index < MAX_IO_TIMERS; timer_index++) {
		if (_timers_init_mask & (1 << timer_index)) {
			if (timer_dma_handles[timer_index].dma_up_handle == NULL) {

				// Set the DMA buffer size to hold all DMA data
				uint32_t channel_count = io_timers_channel_mapping.element[timer_index].channel_count_including_gaps;

				// Configure timer in dshot mode
				io_timer_set_dshot_burst_mode(timer_index, _dshot_frequency, channel_count);

				// Configure DMA UP handle
				timer_dma_handles[timer_index].dma_up_handle = stm32_dmachannel(io_timers[timer_index].dshot.dma_map_up);

				if (timer_dma_handles[timer_index].dma_up_handle == NULL) {
					PX4_INFO("could not allocate timer %u DMA UP handle", timer_index);
					return -ENOSR;
				}

				// This was just for testing. We will initialize again at run time.
				stm32_dmafree(timer_dma_handles[timer_index].dma_up_handle);
				timer_dma_handles[timer_index].dma_up_handle = NULL;
			}
		}
	}

	// For each dshot channel, test the input DMA Capture Compare configuration
	for (uint8_t output_channel = 0; output_channel < MAX_TIMER_IO_CHANNELS; output_channel++) {
		if (_channels_init_mask & (1 << output_channel)) {
			uint8_t timer_index = timer_io_channels[output_channel].timer_index;
			uint8_t timer_channel_index = timer_io_channels[output_channel].timer_channel - 1;

			DMA_HANDLE *dma_handle = &timer_dma_handles[timer_index].dma_ch_handle[timer_channel_index];
			uint32_t dma_map_ch = io_timers[timer_index].dshot.dma_map_ch[timer_channel_index];

			if (dma_map_ch) {
				*dma_handle = stm32_dmachannel(dma_map_ch);

				if (*dma_handle == NULL) {
					PX4_INFO("could not allocate timer %u DMA CH%u handle, output_channel %u", timer_index, timer_channel_index,
						 output_channel);
					return -ENOSR;
				}

				// This was just for testing. We will initialize again at run time.
				stm32_dmafree(*dma_handle);
				*dma_handle = NULL;
			}
		}
	}

	// TODO: can we define this differently?
	unsigned output_buffer_offset = 0;

	for (unsigned timer_index = 0; timer_index < MAX_IO_TIMERS; timer_index++) {
		if (_timers_init_mask & (1 << timer_index)) {
			if (io_timers[timer_index].base == 0) { // no more timers configured
				break;
			}

			// we know the uint8_t* cast to uint32_t* is fine, since we're aligned to cache line size
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wcast-align"
			dshot_output_buffer[timer_index] = (uint32_t *) &dshot_burst_buffer_array[output_buffer_offset];

#pragma GCC diagnostic pop
			uint32_t channel_count = io_timers_channel_mapping.element[timer_index].channel_count_including_gaps;
			output_buffer_offset += DSHOT_OUTPUT_BUFFER_SIZE(channel_count);

			if (output_buffer_offset > sizeof(dshot_burst_buffer_array)) {
				return -EINVAL; // something is wrong with the board configuration or some other logic
			}
		}
	}

	return _channels_init_mask;
}

// Kicks off a DMA transmit for each configured timer and the associated channels
void up_dshot_trigger()
{
	// Enable DShot inverted on all channels
	io_timer_set_enable(true, _bidirectional ? IOTimerChanMode_DshotInverted : IOTimerChanMode_Dshot,
			    IO_TIMER_ALL_MODES_CHANNELS);

	// For each timer, begin DMA transmit
	for (uint8_t timer_index = 0; timer_index < MAX_IO_TIMERS; timer_index++) {
		if (_timers_init_mask & (1 << timer_index)) {

			// Set timer in dshot mode
			uint32_t channel_count = io_timers_channel_mapping.element[timer_index].channel_count_including_gaps;
			io_timer_set_dshot_burst_mode(timer_index, _dshot_frequency, channel_count);

			// Allocate DMA if necessary
			if (timer_dma_handles[timer_index].dma_up_handle == NULL) {

				timer_dma_handles[timer_index].dma_up_handle = stm32_dmachannel(io_timers[timer_index].dshot.dma_map_up);

				if (timer_dma_handles[timer_index].dma_up_handle == NULL) {
					PX4_INFO("DMA allocation for timer %u failed", timer_index);
					return;
				}
			}

			// Flush cache so DMA sees the data
			up_clean_dcache((uintptr_t) dshot_output_buffer[timer_index],
					(uintptr_t) dshot_output_buffer[timer_index] +
					DSHOT_OUTPUT_BUFFER_SIZE(channel_count));

			px4_stm32_dmasetup(timer_dma_handles[timer_index].dma_up_handle,
					   io_timers[timer_index].base + STM32_GTIM_DMAR_OFFSET,
					   (uint32_t)(dshot_output_buffer[timer_index]),
					   channel_count * CHANNEL_OUTPUT_BUFF_SIZE,
					   DSHOT_DMA_SCR);

			// Clean UDE flag before DMA is started
			io_timer_update_dma_req(timer_index, false);

			// Trigger DMA (DShot Outputs)
			_timer_index_array[timer_index] = timer_index;

			if (_bidirectional) {
				stm32_dmastart(timer_dma_handles[timer_index].dma_up_handle, dma_start_callback, &_timer_index_array[timer_index],
					       false);

			} else {
				stm32_dmastart(timer_dma_handles[timer_index].dma_up_handle,  NULL, NULL, false);
			}

			// Enable DMA update request
			io_timer_update_dma_req(timer_index, true);
			px4_arch_gpiowrite(GPIO_FMU_CH8, true);
		}
	}
}

void dma_start_callback(DMA_HANDLE handle, uint8_t status, void *arg)
{
	// DEBUGGING
	px4_arch_gpiowrite(GPIO_FMU_CH8, false);

	uint8_t timer_index = *((uint8_t *)arg);

	// Clean DMA UP configuration
	if (timer_dma_handles[timer_index].dma_up_handle != NULL) {
		stm32_dmastop(timer_dma_handles[timer_index].dma_up_handle);
		stm32_dmafree(timer_dma_handles[timer_index].dma_up_handle);
		timer_dma_handles[timer_index].dma_up_handle = NULL;
	}

	// Disable DMA update request
	io_timer_update_dma_req(timer_index, false);

	// De-allocate timer
	io_timer_unallocate_timer(timer_index);

	// Disable / Init timer channels first
	for (uint8_t output_channel = 0; output_channel < MAX_TIMER_IO_CHANNELS; output_channel++) {

		bool is_this_timer = timer_index == timer_io_channels[output_channel].timer_index;
		bool channel_enabled = _channels_init_mask & (1 << output_channel);

		if (is_this_timer && channel_enabled) {
			io_timer_unallocate_channel(output_channel);
			io_timer_channel_init(output_channel, IOTimerChanMode_CaptureDMA, NULL, NULL);
		}
	}

	// Allocate DMA for all channels on this timer
	for (uint8_t output_channel = 0; output_channel < MAX_TIMER_IO_CHANNELS; output_channel++) {

		bool is_this_timer = timer_index == timer_io_channels[output_channel].timer_index;
		bool channel_enabled = _channels_init_mask & (1 << output_channel);
		uint8_t timer_channel_index = timer_io_channels[output_channel].timer_channel - 1;

		if (is_this_timer && channel_enabled) {

			// Allocate DMA
			if (timer_dma_handles[timer_index].dma_ch_handle[timer_channel_index] == NULL) {
				timer_dma_handles[timer_index].dma_ch_handle[timer_channel_index] = stm32_dmachannel(
							io_timers[timer_index].dshot.dma_map_ch[timer_channel_index]);
			}

			// If DMA handler is valid, start DMA
			if (timer_dma_handles[timer_index].dma_ch_handle[timer_channel_index] == NULL) {
				PX4_INFO("failed to allocate dma for timer %u channel %u", timer_index, timer_channel_index);
				continue;
			}

			// Choose which CC register for this DMA stream
			uint32_t periph_addr = io_timers[timer_index].base + STM32_GTIM_CCR1_OFFSET + (4 * timer_channel_index);

			// Setup DMA for this channel
			px4_stm32_dmasetup(timer_dma_handles[timer_index].dma_ch_handle[timer_channel_index],
					   periph_addr,
					   (uint32_t) dshot_capture_buffer[timer_index][timer_channel_index],
					   CHANNEL_CAPTURE_BUFF_SIZE,
					   DSHOT_BIDIRECTIONAL_DMA_SCR);
		}
	}

	// Flush cache so DMA sees the data
	memset(dshot_capture_buffer[timer_index], 0, sizeof(dshot_capture_buffer[timer_index]));
	up_clean_dcache((uintptr_t) dshot_capture_buffer[timer_index],
			(uintptr_t) dshot_capture_buffer[timer_index] + DSHOT_CAPTURE_BUFFER_SIZE(MAX_NUM_CHANNELS_PER_TIMER));

	// TODO: timer channels mask, right now we enable all 4 channels
	// Enable CaptComp for the channels
	io_timer_set_dshot_capture_mode(timer_index, _dshot_frequency);

	// TODO: timer channels mask, right now we enable all 4 channels
	// Enable CaptureDMA on this timer
	io_timer_capture_dma_req(timer_index, true);

	// Enable CaptureDMA and on all channels
	io_timer_set_enable(true, IOTimerChanMode_CaptureDMA, IO_TIMER_ALL_MODES_CHANNELS);

	// Start DMA on all the channels
	for (uint8_t output_channel = 0; output_channel < MAX_TIMER_IO_CHANNELS; output_channel++) {

		bool is_this_timer = timer_index == timer_io_channels[output_channel].timer_index;
		bool channel_enabled = _channels_init_mask & (1 << output_channel);
		uint8_t timer_channel_index = timer_io_channels[output_channel].timer_channel - 1;

		if (is_this_timer && channel_enabled) {
			// NOTE: we can't use DMA callback since GCR encoding creates a variable length pulse train
			stm32_dmastart(timer_dma_handles[timer_index].dma_ch_handle[timer_channel_index], NULL, NULL, false);
		}
	}

	// 30us to switch regardless of DShot frequency + eRPM frame time + 10us for good measure
	hrt_abstime frame_us = (16 * 1000000) / _dshot_frequency; // 16 bits * us_per_s / bits_per_s
	hrt_abstime delay = 30 + frame_us + 10;
	delay_hrt = delay;
	hrt_call_after(&_dma_capture_callback_call[timer_index], delay, _cc_callbacks[timer_index], arg);

	// DEBUGGING
	px4_arch_gpiowrite(GPIO_FMU_CH8, true);
}

static void capture_complete(uint8_t timer_index)
{
	// DEBUGGING
	px4_arch_gpiowrite(GPIO_FMU_CH8, false);

	for (uint8_t output_channel = 0; output_channel < MAX_TIMER_IO_CHANNELS; output_channel++) {

		bool is_this_timer = timer_index == timer_io_channels[output_channel].timer_index;
		bool channel_enabled = _channels_init_mask & (1 << output_channel);
		uint8_t timer_channel_index = timer_io_channels[output_channel].timer_channel - 1;

		if (is_this_timer && channel_enabled) {
			if (timer_dma_handles[timer_index].dma_ch_handle[timer_channel_index] != NULL) {
				stm32_dmastop(timer_dma_handles[timer_index].dma_ch_handle[timer_channel_index]);
				stm32_dmafree(timer_dma_handles[timer_index].dma_ch_handle[timer_channel_index]);
				timer_dma_handles[timer_index].dma_ch_handle[timer_channel_index] = NULL;
			}

			io_timer_unallocate_channel(output_channel);
		}
	}

	// Disable capture DMA
	io_timer_capture_dma_req(timer_index, false);

	// Unallocate the timer as CaptureDMA
	io_timer_unallocate_timer(timer_index);

	// Invalidate the dcache to ensure most recent data is available
	up_invalidate_dcache((uintptr_t) dshot_capture_buffer[timer_index],
			     (uintptr_t) dshot_capture_buffer[timer_index] + DSHOT_CAPTURE_BUFFER_SIZE(MAX_NUM_CHANNELS_PER_TIMER));

	// Process eRPM frames from all channels on this timer
	process_capture_results(timer_index);

	// Initialize back to DShotInverted to bring IO back to the expected idle state
	for (uint8_t output_channel = 0; output_channel < MAX_TIMER_IO_CHANNELS; output_channel++) {

		bool is_this_timer = timer_index == timer_io_channels[output_channel].timer_index;
		bool channel_enabled = _channels_init_mask & (1 << output_channel);

		if (is_this_timer && channel_enabled) {
			io_timer_channel_init(output_channel, IOTimerChanMode_DshotInverted, NULL, NULL);
		}
	}

	// Enable all channels on this timer configured as DShotInverted
	io_timer_set_enable(true, IOTimerChanMode_DshotInverted, IO_TIMER_ALL_MODES_CHANNELS);
}

static void cc_callback_timer_index_0(void *arg)
{
	uint8_t timer_index = *((uint8_t *)arg);
	capture_complete(timer_index);
}

static void cc_callback_timer_index_1(void *arg)
{
	uint8_t timer_index = *((uint8_t *)arg);
	capture_complete(timer_index);
}

static void cc_callback_timer_index_2(void *arg)
{
	uint8_t timer_index = *((uint8_t *)arg);
	capture_complete(timer_index);
}

void process_capture_results(uint8_t timer_index)
{
	for (uint8_t channel_index = 0; channel_index < MAX_NUM_CHANNELS_PER_TIMER; channel_index++) {

		// Calculate the period for each channel
		const unsigned period = calculate_period(timer_index, channel_index);

		if (period == 0) {
			// If the parsing failed, set the eRPM to 0
			_erpms[timer_index][channel_index] = 0;

		} else if (period == 65408) {
			// Special case for zero motion (e.g., stationary motor)
			_erpms[timer_index][channel_index] = 0;

		} else {
			// Convert the period to eRPM
			_erpms[timer_index][channel_index] = (1000000 * 60) / period;
		}
	}
}

/**
* bits  1-11    - throttle value (0-47 are reserved, 48-2047 give 2000 steps of throttle resolution)
* bit   12      - dshot telemetry enable/disable
* bits  13-16   - XOR checksum
**/
void dshot_motor_data_set(unsigned channel, uint16_t throttle, bool telemetry)
{
	if (!(_channels_init_mask & (1 << channel))) {
		// this channel is not configured for dshot
		return;
	}

	uint16_t packet = 0;
	uint16_t checksum = 0;

	packet |= throttle << DSHOT_THROTTLE_POSITION;
	packet |= ((uint16_t)telemetry & 0x01) << DSHOT_TELEMETRY_POSITION;

	uint16_t csum_data = packet;

	/* XOR checksum calculation */
	csum_data >>= NIBBLES_SIZE;

	for (unsigned i = 0; i < DSHOT_NUMBER_OF_NIBBLES; i++) {
		checksum ^= (csum_data & 0x0F); // XOR data by nibbles
		csum_data >>= NIBBLES_SIZE;
	}

	if (_bidirectional) {
		packet |= ((~checksum) & 0x0F);

	} else {
		packet |= ((checksum) & 0x0F);
	}

	unsigned timer = timer_io_channels[channel].timer_index;

	uint32_t *buffer = dshot_output_buffer[timer];
	const io_timers_channel_mapping_element_t *mapping = &io_timers_channel_mapping.element[timer];
	unsigned num_motors = mapping->channel_count_including_gaps;
	unsigned timer_channel = timer_io_channels[channel].timer_channel - mapping->lowest_timer_channel;

	for (unsigned motor_data_index = 0; motor_data_index < ONE_MOTOR_DATA_SIZE; motor_data_index++) {
		buffer[motor_data_index * num_motors + timer_channel] =
			(packet & 0x8000) ? MOTOR_PWM_BIT_1 : MOTOR_PWM_BIT_0;  // MSB first
		packet <<= 1;
	}
}

int up_dshot_arm(bool armed)
{
	return io_timer_set_enable(armed, _bidirectional ? IOTimerChanMode_DshotInverted : IOTimerChanMode_Dshot,
				   IO_TIMER_ALL_MODES_CHANNELS);
}

int up_bdshot_get_erpm(uint8_t output_channel, int *erpm)
{
	uint8_t timer_index = timer_io_channels[output_channel].timer_index;
	uint8_t timer_channel_index = timer_io_channels[output_channel].timer_channel - 1;

	if (_channels_init_mask & (1 << output_channel)) {
		*erpm = _erpms[timer_index][timer_channel_index];
		return PX4_OK;
	}

	// this channel is not configured for dshot
	return PX4_ERROR;
}

int up_bdshot_channel_status(uint8_t channel)
{
	// TODO: track that each channel is communicating
	if (_channels_init_mask & (1 << channel)) {
		return 1;
	}

	return 0;
}

void up_bdshot_status(void)
{
	PX4_INFO("dshot driver stats: read %lu, failed nibble %lu, failed CRC %lu, invalid/zero %lu",
		 read_ok, read_fail_nibble, read_fail_crc, read_fail_zero);

	PX4_INFO("delay %lu", delay_hrt);
}

uint8_t nibbles_from_mapped(uint8_t mapped)
{
	switch (mapped) {
	case 0x19:
		return 0x00;

	case 0x1B:
		return 0x01;

	case 0x12:
		return 0x02;

	case 0x13:
		return 0x03;

	case 0x1D:
		return 0x04;

	case 0x15:
		return 0x05;

	case 0x16:
		return 0x06;

	case 0x17:
		return 0x07;

	case 0x1a:
		return 0x08;

	case 0x09:
		return 0x09;

	case 0x0A:
		return 0x0A;

	case 0x0B:
		return 0x0B;

	case 0x1E:
		return 0x0C;

	case 0x0D:
		return 0x0D;

	case 0x0E:
		return 0x0E;

	case 0x0F:
		return 0x0F;

	default:
		// Unknown mapped
		return 0xFF;
	}
}

unsigned calculate_period(uint8_t timer_index, uint8_t channel_index)
{
	uint32_t value = 0;
	uint32_t high = 1; // We start off with high
	unsigned shifted = 0;
	unsigned previous = 0;

	// Loop through the capture buffer for the specified channel
	for (unsigned i = 1; i < CHANNEL_CAPTURE_BUFF_SIZE; ++i) {

		// Ignore the first data point, which is the pulse before it starts
		if (i > 1) {

			if (dshot_capture_buffer[timer_index][channel_index][i] == 0) {
				// Once we hit zeros, the data has ended
				break;
			}

			// Quantization logic for DShot 150, 300, 600, 1200
			const uint32_t bits = (dshot_capture_buffer[timer_index][channel_index][i] - previous + 5) / 20;

			// Shift the bits into the value
			for (unsigned bit = 0; bit < bits; ++bit) {
				value = (value << 1) | high;
				++shifted;
			}

			// Toggle the high/low state on the next edge
			high = !high;
		}

		previous = dshot_capture_buffer[timer_index][channel_index][i];
	}

	if (shifted == 0) {
		// If no data was shifted, the read failed
		++read_fail_zero;
		return 0;
	}

	// Ensure we've shifted 21 times; otherwise, we pad with zeros
	value <<= (21 - shifted);

	// Gray code (GCR) to eRPM conversion logic
	unsigned gcr = (value ^ (value >> 1));

	uint32_t data = 0;

	// Decode 4 nibbles (5-bit gray code -> 4-bit nibbles)
	for (unsigned i = 0; i < 4; ++i) {
		uint32_t nibble = nibbles_from_mapped(gcr & 0x1F) << (4 * i);

		if (nibble == 0xFF) {
			++read_fail_nibble;
			return 0;
		}

		data |= nibble;
		gcr >>= 5;
	}

	unsigned shift = (data & 0xE000) >> 13;
	unsigned period = ((data & 0x1FF0) >> 4) << shift;
	unsigned crc = data & 0xF;

	unsigned payload = (data & 0xFFF0) >> 4;
	unsigned calculated_crc = (~(payload ^ (payload >> 4) ^ (payload >> 8))) & 0x0F;

	if (crc != calculated_crc) {
		++read_fail_crc;
		return 0;
	}

	++read_ok;
	return period;
}

#endif

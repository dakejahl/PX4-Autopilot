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
 *	notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *	notice, this list of conditions and the following disclaimer in
 *	the documentation and/or other materials provided with the
 *	distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *	used to endorse or promote products derived from this software
 *	without specific prior written permission.
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
#include <drivers/drv_hrt.h>

#define MOTOR_PWM_BIT_1				14u
#define MOTOR_PWM_BIT_0				7u
#define DSHOT_TIMERS				MAX_IO_TIMERS
#define MOTORS_NUMBER				DIRECT_PWM_OUTPUT_CHANNELS
#define ONE_MOTOR_DATA_SIZE			16u
#define CHANNEL_OUTPUT_BUFF_SIZE			17u
#define ALL_MOTORS_BUF_SIZE			(MOTORS_NUMBER * CHANNEL_OUTPUT_BUFF_SIZE)
#define DSHOT_THROTTLE_POSITION		5u
#define DSHOT_TELEMETRY_POSITION	4u
#define NIBBLES_SIZE 				4u
#define DSHOT_NUMBER_OF_NIBBLES		3u
#define DSHOT_END_OF_STREAM 		16u
#define MAX_NUM_CHANNELS_PER_TIMER	4u // CCR1-CCR4

#define DSHOT_DMA_SCR (DMA_SCR_PRIHI | DMA_SCR_MSIZE_32BITS | DMA_SCR_PSIZE_32BITS | DMA_SCR_MINC | \
		       DMA_SCR_DIR_M2P | DMA_SCR_TCIE | DMA_SCR_HTIE | DMA_SCR_TEIE | DMA_SCR_DMEIE)

// TODO: why 16bit?
#define DSHOT_BIDIRECTIONAL_DMA_SCR (DMA_SCR_PRIHI | DMA_SCR_MSIZE_16BITS | DMA_SCR_PSIZE_16BITS | DMA_SCR_MINC | \
				     DMA_SCR_DIR_P2M | DMA_SCR_TCIE | DMA_SCR_TEIE | DMA_SCR_DMEIE)

typedef struct dshot_handler_t {
	DMA_HANDLE		dma_up_handle; // DMA stream for DMA update
	DMA_HANDLE		dma_ch_handle[4]; // DMA streams for bidi capture compare
	uint32_t 		dma_ch_count;
	uint32_t		dma_size;
} dshot_handler_t;

#if defined(CONFIG_ARMV7M_DCACHE)
#  define DMA_BUFFER_MASK    (ARMV7M_DCACHE_LINESIZE - 1)
#  define DMA_ALIGN_UP(n)    (((n) + DMA_BUFFER_MASK) & ~DMA_BUFFER_MASK)
#else
#define DMA_ALIGN_UP(n) (n)
#endif
#define DSHOT_BURST_BUFFER_SIZE(channel_count) (DMA_ALIGN_UP(sizeof(uint32_t)*CHANNEL_OUTPUT_BUFF_SIZE*channel_count))

static dshot_handler_t dshot_handler[DSHOT_TIMERS] = {};
static uint8_t dshot_burst_buffer_array[DSHOT_TIMERS * DSHOT_BURST_BUFFER_SIZE(MAX_NUM_CHANNELS_PER_TIMER)]
px4_cache_aligned_data() = {};
static uint32_t *dshot_burst_buffer[DSHOT_TIMERS] = {};

static uint16_t dshot_capture_buffer[32] px4_cache_aligned_data() = {};

// TODO: audit local variables
static struct hrt_call _call;

static void dma_callback_begin_capture(DMA_HANDLE handle, uint8_t status, void *arg);
static void dma_callback_capture_complete(DMA_HANDLE handle, uint8_t status, void *arg);
static void reinitialize_timers_callback(void *arg);

static void process_capture_results(void *arg);
static unsigned calculate_period(void);

static uint32_t read_ok = 0;
static uint32_t read_fail_nibble = 0;
static uint32_t read_fail_crc = 0;
static uint32_t read_fail_zero = 0;

static uint32_t jakechannelcounter[8] = {};


static bool _birectional = false;
static uint32_t _dshot_frequency = 0;
static int _timers_init_mask = 0;
static int _channels_init_mask = 0;

// We only support capture on the first timer (usually 4 channels) for now.
static uint32_t _motor_to_capture = 0;
static int32_t _erpms[16] = {};


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

unsigned calculate_period(void)
{
	uint32_t value = 0;

	// We start off with high
	uint32_t high = 1;

	unsigned shifted = 0;
	unsigned previous = 0;

	for (unsigned i = 1; i < (32); ++i) {

		// We can ignore the very first data point as it's the pulse before it starts.
		if (i > 1) {

			if (dshot_capture_buffer[i] == 0) {
				// Once we get zeros we're through.
				break;
			}

			// This seemss to work with dshot 150, 300, 600, 1200
			// The values were found by trial and error to get the quantization just right.
			const uint32_t bits = (dshot_capture_buffer[i] - previous + 5) / 20;

			for (unsigned bit = 0; bit < bits; ++bit) {
				value = value << 1;
				value |= high;
				++shifted;
			}

			// The next edge toggles.
			high = !high;
		}

		previous = dshot_capture_buffer[i];
	}

	if (shifted == 0) {
		// no data yet, or this time
		++read_fail_zero;
		return 0;
	}

	// We need to make sure we shifted 21 times. We might have missed some low "pulses" at the very end.
	value = value << (21 - shifted);

	// Note: At 0 throttle, the value is 0x1AD6AE, so 0b110101101011010101110

	// From GCR to eRPM according to:
	// https://brushlesswhoop.com/dshot-and-bidirectional-dshot/#erpm-transmission
	unsigned gcr = (value ^ (value >> 1));

	uint32_t data = 0;

	// 20bits -> 5 mapped -> 4 nibbles
	for (unsigned i = 0; i < 4; ++i) {
		uint32_t nibble = nibbles_from_mapped(gcr & (0x1F)) << (4 * i);

		if (nibble == 0xff) {
			++read_fail_nibble;
			return 0;
		}

		data |= nibble;
		gcr = gcr >> 5;
	}

	unsigned shift = (data & 0xE000) >> 13;
	unsigned period = ((data & 0x1FF0) >> 4) << shift;
	unsigned crc = (data & 0xf);

	unsigned payload = (data & 0xFFF0) >> 4;
	unsigned calculated_crc = (~(payload ^ (payload >> 4) ^ (payload >> 8))) & 0x0F;

	if (crc != calculated_crc) {
		++read_fail_crc;
		return 0;
	}

	++read_ok;
	return period;
}

int up_dshot_init(uint32_t channel_mask, unsigned dshot_pwm_freq, bool enable_bidirectional_dshot)
{
	_birectional = enable_bidirectional_dshot;
	_dshot_frequency = dshot_pwm_freq;

	// Initialize the timer channels
	for (unsigned output_channel = 0; output_channel < MAX_TIMER_IO_CHANNELS; output_channel++) {
		if (channel_mask & (1 << output_channel)) {
			uint8_t timer_index = timer_io_channels[output_channel].timer_index;

			// TODO: is there a smarter way to do this?
			if (io_timers[timer_index].dshot.dma_base == 0) {
				// board does not configure dshot on this timer
				continue;
			}

			io_timer_channel_mode_t mode = _birectional ? IOTimerChanMode_DshotInverted : IOTimerChanMode_Dshot;
			int ret = io_timer_channel_init(output_channel, mode, NULL, NULL);
			if (ret != OK) {
				PX4_INFO("io_timer_channel_init %u failed", output_channel);
				return ret;
			}

			_channels_init_mask |= (1 << output_channel);
			_timers_init_mask |= (1 << timer_index);
		}
	}

	// For each dshot timer:
	// - enable dshot mode
	// - allocate DMA for Update (transmit)
	// - allocate DMA for Capture Compare (receive)
	for (uint8_t timer_index = 0; timer_index < MAX_IO_TIMERS; timer_index++) {
		if (_timers_init_mask & (1 << timer_index)) {
			if (dshot_handler[timer_index].dma_up_handle == NULL) {

				// Set the DMA buffer size to hold all DMA data
				uint32_t channel_count = io_timers_channel_mapping.element[timer_index].channel_count_including_gaps;
				dshot_handler[timer_index].dma_size = channel_count * CHANNEL_OUTPUT_BUFF_SIZE;

				// Configure timer in dshot mode
				io_timer_set_dshot_mode(timer_index, _dshot_frequency, channel_count);

				// Configure DMA UP handle
				dshot_handler[timer_index].dma_up_handle = stm32_dmachannel(io_timers[timer_index].dshot.dma_map_up);

				if (dshot_handler[timer_index].dma_up_handle == NULL) {
					// TODO: how to log this?
					PX4_INFO("could not allocate timer %u DMA UP handle", timer_index);
					return -ENOSR;
				}

				// This was just for testing. We will initialize again at run time.
				stm32_dmafree(dshot_handler[timer_index].dma_up_handle);
				dshot_handler[timer_index].dma_up_handle = NULL;
				PX4_INFO("allocated and freed DMA for timer %u UP", timer_index);
			}
		}
	}

	// For each dshot channel:
	// - allocate DMA for Capture Compare (receive)
	for (uint8_t output_channel = 0; output_channel < MAX_TIMER_IO_CHANNELS; output_channel++) {
		if (_channels_init_mask & (1 << output_channel)) {
			uint8_t timer_index = timer_io_channels[output_channel].timer_index;
			uint8_t timer_channel_index = timer_io_channels[output_channel].timer_channel - 1;

			DMA_HANDLE* dma_handle = &dshot_handler[timer_index].dma_ch_handle[timer_channel_index];
			uint32_t dma_map_ch = io_timers[timer_index].dshot.dma_map_ch[timer_channel_index];
			if (dma_map_ch) {
				PX4_INFO("allocating timer %u DMA CH%u handle, output_channel %u", timer_index, timer_channel_index, output_channel);
				*dma_handle = stm32_dmachannel(dma_map_ch);
				if (*dma_handle == NULL) {
					PX4_INFO("could not allocate timer %u DMA CH%u handle, output_channel %u", timer_index, timer_channel_index, output_channel);
					return -ENOSR;
				}

				// This was just for testing. We will initialize again at run time.
				PX4_INFO("allocated timer %u DMA CH%u handle, output_channel %u", timer_index, timer_channel_index, output_channel);
				stm32_dmafree(*dma_handle);
				*dma_handle = NULL;
				PX4_INFO("FREEEEE");
			}
		}
	}

	px4_usleep(500000);

	// TODO: what are we doing here and why can't this be defined at compile time?
	unsigned buffer_offset = 0;

	for (unsigned timer_index = 0; timer_index < DSHOT_TIMERS; timer_index++) {
		if (_timers_init_mask & (1 << timer_index)) {
			if (io_timers[timer_index].base == 0) { // no more timers configured
				break;
			}

			// we know the uint8_t* cast to uint32_t* is fine, since we're aligned to cache line size
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wcast-align"
			dshot_burst_buffer[timer_index] = (uint32_t *) &dshot_burst_buffer_array[buffer_offset];
#pragma GCC diagnostic pop
			uint32_t channel_count = io_timers_channel_mapping.element[timer_index].channel_count_including_gaps;
			buffer_offset += DSHOT_BURST_BUFFER_SIZE(channel_count);

			if (buffer_offset > sizeof(dshot_burst_buffer_array)) {
				return -EINVAL; // something is wrong with the board configuration or some other logic
			}
		}
	}

	return _channels_init_mask;
}

// Kicks off a DMA transmit for each configured timer and the associated channels
void up_dshot_trigger(void)
{
	// For bidirectional dshot we need to re-initialize the timers every time here.
	// In normal mode, we just do it once.
	if (_birectional) {
		for (uint8_t output_channel = 0; output_channel < MAX_TIMER_IO_CHANNELS; output_channel++) {
			if (_channels_init_mask & (1 << output_channel)) {
				io_timer_unallocate_channel(output_channel);
				int ret = io_timer_channel_init(output_channel, IOTimerChanMode_DshotInverted, NULL, NULL);
				if (ret != OK) {
					PX4_INFO("io_timer_channel_init %u failed", output_channel);
					return;
				}
			}
		}
	}

	// Enable all timers configured as dshot (weird way of doing this...)
	io_timer_set_enable(true, _birectional ? IOTimerChanMode_DshotInverted : IOTimerChanMode_Dshot,
			    IO_TIMER_ALL_MODES_CHANNELS);

	// Clear capture_buffer cache
	if (_birectional) {
		uintptr_t buf = (uintptr_t)dshot_capture_buffer;
		size_t size = sizeof(dshot_capture_buffer);
		memset(dshot_capture_buffer, 0, size);
		up_clean_dcache(buf, buf + size);
	}

	// For each timer, begin DMA transmit
	for (uint8_t timer_index = 0; timer_index < DSHOT_TIMERS; timer_index++) {
		if (_timers_init_mask & (1 << timer_index)) {

			uint32_t channel_count = io_timers_channel_mapping.element[timer_index].channel_count_including_gaps;

			// Allocate DMA if necessary
			if (dshot_handler[timer_index].dma_up_handle == NULL) {
				dshot_handler[timer_index].dma_size = channel_count * CHANNEL_OUTPUT_BUFF_SIZE;

				io_timer_set_dshot_mode(timer_index, _dshot_frequency, channel_count);

				dshot_handler[timer_index].dma_up_handle = stm32_dmachannel(io_timers[timer_index].dshot.dma_map_up);

				if (dshot_handler[timer_index].dma_up_handle == NULL) {
					PX4_INFO("DMA allocation for timer %u failed", timer_index);
					return;
				}
			}

			// Flush cache so DMA sees the data
			up_clean_dcache((uintptr_t) dshot_burst_buffer[timer_index],
					(uintptr_t) dshot_burst_buffer[timer_index] +
					DSHOT_BURST_BUFFER_SIZE(channel_count));

			px4_stm32_dmasetup(dshot_handler[timer_index].dma_up_handle,
					   io_timers[timer_index].base + STM32_GTIM_DMAR_OFFSET,
					   (uint32_t)(dshot_burst_buffer[timer_index]),
					   dshot_handler[timer_index].dma_size,
					   DSHOT_DMA_SCR);

			// Clean UDE flag before DMA is started
			io_timer_disable_update_dma_req(timer_index);

			// Trigger DMA (DShot Outputs)
			stm32_dmastart(dshot_handler[timer_index].dma_up_handle, _birectional ? dma_callback_begin_capture : NULL, &timer_index, false);

			// Enable DMA update request
			io_timer_enable_update_dma_req(timer_index);
		}
	}
}

void dma_callback_begin_capture(DMA_HANDLE handle, uint8_t status, void *arg)
{
	(void)handle;
	(void)status;

	uint8_t timer_index = *((uint8_t*)arg);

	// Clear DMA UP configuration
	if (dshot_handler[timer_index].dma_up_handle != NULL) {
		stm32_dmastop(dshot_handler[timer_index].dma_up_handle);
		stm32_dmafree(dshot_handler[timer_index].dma_up_handle);
		dshot_handler[timer_index].dma_up_handle = NULL;
	}

	// Disable DMA capture request
	io_timer_disable_capture_dma_req(timer_index);

	// Allocate DMA for all channels on this timer

	for (uint8_t output_channel = 0; output_channel < MAX_TIMER_IO_CHANNELS; output_channel++) {

		bool is_this_timer = timer_index == timer_io_channels[output_channel].timer_index;
		bool channel_enabled = _channels_init_mask & (1 << output_channel);

		if (is_this_timer && channel_enabled) {
			uint8_t timer_channel_index = timer_io_channels[output_channel].timer_channel -1;
			dshot_handler[timer_index].dma_ch_handle[timer_channel_index] = stm32_dmachannel(io_timers[timer_index].dshot.dma_map_ch[timer_channel_index]);

			// Setup DMA for this channel
			px4_stm32_dmasetup(dshot_handler[timer_index].dma_ch_handle[timer_channel_index],
					   io_timers[timer_index].base + STM32_GTIM_DMAR_OFFSET, // DMA address for burst mode (16-bit, TIM2-5 only)
					   (uint32_t) dshot_capture_buffer,
					   sizeof(dshot_capture_buffer),
					   DSHOT_BIDIRECTIONAL_DMA_SCR);

			// Reset / Enable timer channel
			io_timer_unallocate_channel(output_channel);
			io_timer_channel_init(output_channel, IOTimerChanMode_CaptureDMA, NULL, NULL);
			io_timer_set_enable(true, IOTimerChanMode_CaptureDMA, 1 << output_channel);

			// Enable input capture on this channel
			up_input_capture_set(output_channel, Both, 0, NULL, NULL);

			// Setup dshot capture for this channel
			io_timer_set_dshot_capture_mode(timer_index, _dshot_frequency, output_channel);

			// Start DMA with callback if bidi dshot is enabled
			stm32_dmastart(dshot_handler[timer_index].dma_up_handle, dma_callback_capture_complete, &output_channel, false);
		}
	}

	// Enable DMA capture compare on this timer
	io_timer_enable_capture_dma_req(timer_index);
}

static void dma_callback_capture_complete(DMA_HANDLE handle, uint8_t status, void *arg)
{
	unsigned output_channel = *((unsigned*)arg);

	jakechannelcounter[output_channel]++;

	hrt_call_after(&_call, 1, reinitialize_timers_callback, &output_channel);
}

void reinitialize_timers_callback(void *arg)
{
	unsigned output_channel = *((unsigned*)arg);

	uint8_t timer_index = timer_io_channels[output_channel].timer_index;
	uint8_t timer_channel_index = timer_io_channels[output_channel].timer_channel - 1;

	stm32_dmastop(dshot_handler[timer_index].dma_ch_handle[timer_channel_index]);
	stm32_dmafree(dshot_handler[timer_index].dma_ch_handle[timer_channel_index]);
	dshot_handler[timer_index].dma_ch_handle[timer_channel_index] = NULL;

	io_timer_unallocate_channel(output_channel);
	io_timer_channel_init(output_channel, IOTimerChanMode_DshotInverted, NULL, NULL);
}

void process_capture_results(void *arg)
{
	(void)arg;

	// In case DMA is still set up from the last capture, we clear that.
	for (unsigned timer = 0; timer < DSHOT_TIMERS; ++timer) {
		if (_timers_init_mask & (1 << timer)) {
			if (dshot_handler[timer].dma_up_handle != NULL) {
				stm32_dmastop(dshot_handler[timer].dma_up_handle);
				stm32_dmafree(dshot_handler[timer].dma_up_handle);
				dshot_handler[timer].dma_up_handle = NULL;
			}
		}
	}

	up_invalidate_dcache((uintptr_t)dshot_capture_buffer,
			     (uintptr_t)dshot_capture_buffer +
			     sizeof(dshot_capture_buffer));

	const unsigned period = calculate_period();

	if (period == 0) {
		// If the parsing failed, we get 0.
		_erpms[_motor_to_capture] = 0;

	} else if (period == 65408) {
		// For still, we get this magic 65408 value.
		_erpms[_motor_to_capture] = 0;

	} else {
		// from period in us to eRPM
		_erpms[_motor_to_capture] = 1000000 * 60 / period;
	}

	for (unsigned channel = 0; channel < MAX_TIMER_IO_CHANNELS; channel++) {
		if (_channels_init_mask & (1 << channel)) {
			io_timer_unallocate_channel(channel);
			io_timer_channel_init(channel, IOTimerChanMode_DshotInverted, NULL, NULL);
		}
	}

	// if (_erpm_callback != NULL) {
	// 	// Only publish every 4th time once all measurements have come in.
	// 	if (_motor_to_capture == 3) {
	// 		_erpm_callback(_erpms, 4, _erpm_callback_context);
	// 	}
	// }

	_motor_to_capture = (_motor_to_capture + 1) % 4;
}

/**
* bits 	1-11	- throttle value (0-47 are reserved, 48-2047 give 2000 steps of throttle resolution)
* bit 	12		- dshot telemetry enable/disable
* bits 	13-16	- XOR checksum
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

	if (_birectional) {
		packet |= ((~checksum) & 0x0F);

	} else {
		packet |= ((checksum) & 0x0F);
	}

	unsigned timer = timer_io_channels[channel].timer_index;

	uint32_t *buffer = dshot_burst_buffer[timer];
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
	return io_timer_set_enable(armed, _birectional ? IOTimerChanMode_DshotInverted : IOTimerChanMode_Dshot,
				   IO_TIMER_ALL_MODES_CHANNELS);
}

int up_bdshot_get_erpm(uint8_t channel, int *erpm)
{
	if (_channels_init_mask & (1 << channel)) {
		*erpm = _erpms[channel];
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

	PX4_INFO("ch0 %lu, ch1 %lu, ch2 %lu, ch3 %lu,", jakechannelcounter[0], jakechannelcounter[1], jakechannelcounter[2], jakechannelcounter[3]);
}

#endif

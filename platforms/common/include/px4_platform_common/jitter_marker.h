/****************************************************************************
 * Bench-only jitter instrumentation.
 *
 * Emits a short pulse burst on a spare FMU pad so a logic analyzer capturing
 * the DShot outputs can attribute output gaps to the responsible code path.
 * Compiles to a no-op unless the board defines BOARD_JITTER_MARKER_GPIO
 * (see boards/ark/fpv/src/board_config.h).
 *
 * Burst ids (number of pulses):
 *   2 = arm transition          (Commander::arm success)
 *   3 = disarm transition       (Commander::disarm success)
 *   4 = param flash save start  (param_save_default, flash backend)
 *   5 = param flash save end
 *   6 = param sector erase start (flashfs32 wrap: 128 KB bank-2 sector erase)
 *   7 = param sector erase end
 ****************************************************************************/

#pragma once

#if defined(__PX4_NUTTX)
#include <px4_platform_common/px4_config.h>
#endif

#if defined(__PX4_NUTTX) && defined(BOARD_JITTER_MARKER_GPIO)

#include <nuttx/arch.h>
#include <px4_arch/micro_hal.h>

static inline void jitter_marker_burst(int pulse_count)
{
	/* Re-assert the pad config: the boot-time setup can be overridden once the output
	 * timers configure (per-translation-unit static — reconfiguring is idempotent). */
	static bool configured = false;

	if (!configured) {
		px4_arch_configgpio(BOARD_JITTER_MARKER_GPIO);
		configured = true;
	}

	for (int i = 0; i < pulse_count; i++) {
		px4_arch_gpiowrite(BOARD_JITTER_MARKER_GPIO, true);
		up_udelay(5);
		px4_arch_gpiowrite(BOARD_JITTER_MARKER_GPIO, false);
		up_udelay(5);
	}
}

#else

static inline void jitter_marker_burst(int pulse_count) { (void)pulse_count; }

#endif

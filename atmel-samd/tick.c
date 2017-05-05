#include "autoreload.h"

#include <peripheral_clk_config.h>

#include "hal/include/hal_timer.h"
#include "hpl/pm/hpl_pm_base.h"
#include "hpl/tc/hpl_tc_base.h"
#include "hpl/gclk/hpl_gclk_base.h"

#include "tick.h"

// Global millisecond tick count
volatile uint64_t ticks_ms = 0;

struct timer_descriptor ms_timer;
static struct timer_task task;

void timer_tick(const struct timer_task *const timer_task) {
    // SysTick interrupt handler called when the SysTick timer reaches zero
    // (every millisecond).
    ticks_ms += 1;

    #ifdef CIRCUITPY_AUTORELOAD_DELAY_MS
        autoreload_tick();
    #endif
}

void tick_init() {
    _pm_enable_bus_clock(PM_BUS_APBC, TC5);
    _gclk_enable_channel(TC5_GCLK_ID, CONF_GCLK_TC5_SRC);

    timer_init(&ms_timer, TC5, _tc_get_timer());

    timer_set_clock_cycles_per_tick(&ms_timer, CONF_GCLK_TC5_FREQUENCY / 1000 - 1);
    task.cb       = timer_tick;
    task.interval = 1;
    task.mode     = TIMER_TASK_REPEAT;
    timer_add_task(&ms_timer, &task);

    timer_start(&ms_timer);
}

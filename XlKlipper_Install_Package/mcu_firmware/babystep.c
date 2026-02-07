// Real-time babystepping for Z-axis live adjustment.
//
// This module provides immediate Z-axis micro-adjustments using existing
// stepper OIDs instead of configuring separate pins (which would conflict
// with Klipper's pin reservation system).
//
// Copyright (C) 2024  Your Name
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include "autoconf.h" // CONFIG_*
#include "basecmd.h" // oid_lookup
#include "board/gpio.h" // gpio_out_write, gpio_out_toggle_noirq
#include "board/irq.h" // irq_disable, irq_enable
#include "board/misc.h" // timer_read_time, timer_from_us
#include "command.h" // DECL_COMMAND
#include "sched.h" // struct timer

// Forward declaration of command_config_stepper from stepper.c
// This allows us to use oid_lookup() to find stepper objects
void command_config_stepper(uint32_t *args);

// Mirror of trsync_signal struct from trsync.h
// struct trsync_signal {
//     struct trsync_signal *next;
//     trsync_callback_t func;
// };
struct babystep_trsync_signal {
    void *next;
    void *func;
};

// Mirror the stepper struct layout from stepper.c
// IMPORTANT: This struct MUST match the layout in stepper.c exactly up to
// and including the 'flags' field. If stepper.c changes, update this!
//
// From stepper.c:
//   struct stepper {
//       struct timer time;
//       uint32_t interval;
//       int16_t add;
//       uint32_t count;
//       uint32_t next_step_time, step_pulse_ticks;
//       struct gpio_out step_pin, dir_pin;
//       uint32_t position;
//       struct move_queue_head mq;
//       struct trsync_signal stop_signal;
//       uint8_t flags : 8;
//   };
struct stepper {
    struct timer time;
    uint32_t interval;
    int16_t add;
    uint32_t count;
    uint32_t next_step_time, step_pulse_ticks;
    struct gpio_out step_pin, dir_pin;
    uint32_t position;
    struct move_queue_head mq;
    struct babystep_trsync_signal stop_signal;
    uint8_t flags;
};

// Stepper flags from stepper.c that we need
enum {
    SF_LAST_DIR=1<<0, SF_NEXT_DIR=1<<1, SF_INVERT_STEP=1<<2, SF_NEED_RESET=1<<3,
    SF_SINGLE_SCHED=1<<4, SF_OPTIMIZED_PATH=1<<5, SF_HAVE_ADD=1<<6
};

// Babystep configuration structure
struct babystep {
    struct stepper *stepper;   // Pointer to the stepper we're controlling
    uint8_t stepper_oid;       // OID of the stepper
    uint32_t step_pulse_ticks; // Minimum pulse width in timer ticks
    uint8_t invert_step;       // Whether step pin is inverted
};

static struct babystep babystep_state;

// Configure babystep to use an existing stepper's pins
void
command_config_babystep(uint32_t *args)
{
    uint8_t stepper_oid = args[0];

    // Look up the stepper by its OID
    struct stepper *s = oid_lookup(stepper_oid, command_config_stepper);

    babystep_state.stepper = s;
    babystep_state.stepper_oid = stepper_oid;
    babystep_state.step_pulse_ticks = s->step_pulse_ticks;
    babystep_state.invert_step = (s->flags & SF_INVERT_STEP) ? 1 : 0;
}
DECL_COMMAND(command_config_babystep,
             "config_babystep stepper_oid=%c");

// Execute a single babystep pulse
// direction: 0 = one direction, 1 = other direction
// The actual physical direction depends on wiring and config
static void
babystep_pulse(uint8_t direction)
{
    struct stepper *s = babystep_state.stepper;
    if (!s)
        return;

    uint8_t invert = babystep_state.invert_step;
    uint8_t step_active = invert ? 0 : 1;
    uint8_t step_inactive = invert ? 1 : 0;

    irq_disable();

    // Set direction pin
    gpio_out_write(s->dir_pin, direction);

    // Direction setup time - most stepper drivers need 200ns+ setup time
    // We use 1us to be safe across all drivers
    uint32_t start = timer_read_time();
    uint32_t setup_ticks = timer_from_us(1);
    while (timer_read_time() - start < setup_ticks)
        ;

    // Pulse the step pin to active state
    gpio_out_write(s->step_pin, step_active);

    // Hold for minimum pulse width (from stepper config)
    start = timer_read_time();
    while (timer_read_time() - start < babystep_state.step_pulse_ticks)
        ;

    // Return step pin to inactive state
    gpio_out_write(s->step_pin, step_inactive);

    irq_enable();
}

// Command to execute multiple babysteps
// count: number of steps (signed - positive/negative for direction)
// direction: base direction (0 or 1), inverted if count is negative
void
command_babystep(uint32_t *args)
{
    int16_t count = args[0];
    uint8_t direction = args[1];

    if (!babystep_state.stepper)
        return;

    // Handle negative counts by inverting direction
    if (count < 0) {
        count = -count;
        direction = !direction;
    }

    // Execute the babysteps with inter-step delay
    for (int16_t i = 0; i < count; i++) {
        babystep_pulse(direction);

        // Inter-step delay to prevent overrunning the driver
        // 250us gives ~4kHz step rate - slower to prevent skipped steps
        uint32_t start = timer_read_time();
        uint32_t delay_ticks = timer_from_us(250);
        while (timer_read_time() - start < delay_ticks)
            ;
    }
}
DECL_COMMAND(command_babystep, "babystep count=%hi direction=%c");

// Command for a single immediate babystep
void
command_babystep_single(uint32_t *args)
{
    uint8_t direction = args[0];
    babystep_pulse(direction);
}
DECL_COMMAND(command_babystep_single, "babystep_single direction=%c");

// Note: Status tracking is handled by the Python host module
// No query command needed - Python tracks configuration state



#include "buttons.h"
#include "dali.h"
#include <hardware/flash.h>
#include <hardware/regs/addressmap.h>
#include <pico/multicore.h>
#include <pico/stdlib.h>
#include <pico/sync.h>
#include <stdio.h>
#include <string.h>
// #include "log.h"
#include "modbus.h"

#define BUTTON_SER_PIN 6
#define BUTTON_CLK_PIN 7
#define ROW_BASE_PIN 8

// How often we scan each button. By slowing down scanning we allow for
// debouncing.
#define SCAN_PERIOD_MS 10

#define MS_TO_COUNTDOWN(ms) (ms / SCAN_PERIOD_MS)

// We are assuming 2MB of flash, and we use the last sector (4Kb).  The very
// last value will be set to a magic value.
#define FLASH_CONFIG_OFFSET ((2 * 1024 * 1024) - FLASH_SECTOR_SIZE)

// This must be at least NUM_FIXTURES * NUM_BUTTONS_PER_FIXTURE + 1
#define NUM_BINDINGS 256

// Must be a multiple of 256 (the flash PAGE size) - Ensure that NUM_BINDINGS is
// set so that this is true.
#define CONFIG_SZ (NUM_BINDINGS * sizeof(uint32_t))
static const uint32_t *bindings =
    (const uint32_t *)(XIP_BASE + FLASH_CONFIG_OFFSET);

button_ctx_t button_ctx[NUM_FIXTURES * NUM_BUTTONS_PER_FIXTURE];

static int next_fixture_poll = 0;
static button_ctx_t *ctx = button_ctx;
static absolute_time_t next_fixture_scan_time;

critical_section_t binding_access_lock;

static const char *tag = "Button";

static uint32_t encode_binding(binding_t *binding) {
  uint32_t v = binding->type & 0x03;         // top two most bits
  v = v << 14 | (binding->device & 0x3FFF);  // next 14 bits
  v = v << 16 | (binding->address & 0xFFFF); // Lower 16 bits.
  return v;
}

static inline void lock() {
  critical_section_enter_blocking(&binding_access_lock);
}

static inline void unlock() { critical_section_exit(&binding_access_lock); }

static void decode_binding(uint32_t encoded, binding_t *binding) {
  binding->address = encoded & 0xFFFF;
  encoded >>= 16;
  binding->device = encoded & 0x3FFF;
  encoded >>= 14;
  binding->type = encoded & 0x3;
}

#define MAGIC_VALUE (('M' << 24) | ('E' << 16) | ('C' << 8) | 'H')
static inline bool flash_bindings_invalid() {
    return bindings[NUM_BINDINGS - 1] != MAGIC_VALUE;
}

void get_binding_at_index(uint index, binding_t *binding) {
  lock();
  if (flash_bindings_invalid()) {
    // Flash isn't initialised.  Treat it as all NONE
    binding->type = BINDING_TYPE_NONE;
    binding->device = 0;
    binding->address = 0;
  } else {
    decode_binding(bindings[index], binding);
  }
  unlock();
}

uint32_t get_binding(uint fixture, uint button) {
  assert(fixture < NUM_FIXTURES && button < NUM_BUTTONS_PER_FIXTURE);
  lock();
  uint32_t val = bindings[fixture * NUM_BUTTONS_PER_FIXTURE + button];
  unlock();
  return val;
}

/**
 * NOTE this function must only be called from the second core.
 */
void set_and_persist_binding(uint fixture, uint button, binding_t *binding) {
  assert(fixture < NUM_FIXTURES && button < NUM_BUTTONS_PER_FIXTURE);
  uint32_t sector[NUM_BINDINGS];
  if (flash_bindings_invalid()) {
    // Its a first write, so clear everything out.
    for (int i = 0; i < NUM_BINDINGS - 1; i++) {
      // Default to NO binding
      sector[i] = BINDING_TYPE_NONE << 30;
    }
    // Last one is a special magic value.
    sector[NUM_BINDINGS - 1] = MAGIC_VALUE;
  } else {
    // Make a copy of the existing bindings (including the magic value)
    memcpy(sector, bindings, NUM_BINDINGS * sizeof(uint32_t));
  }

  uint32_t new_binding = encode_binding(binding);
  sector[fixture * NUM_BUTTONS_PER_FIXTURE + button] = new_binding;

  // Write the values.
  lock();
  multicore_lockout_start_blocking();
  uint32_t ints = save_and_disable_interrupts();
  flash_range_erase(FLASH_CONFIG_OFFSET,
                    FLASH_SECTOR_SIZE); // Must always erase entire sectors
  flash_range_program(FLASH_CONFIG_OFFSET, (const uint8_t *)sector, CONFIG_SZ);
  restore_interrupts(ints);
  multicore_lockout_end_blocking();
  unlock();
}

static void modbus_press_done(modbus_task_state_t state, uint8_t *resp,
                              size_t sz, void *arg) {
  free(arg);
}

/**
 * Processes the (potentially repeated) pressing of a button
 *
 * This task will be deleted when the button is released, so it should take care
 * not to allocate any memory, as it won't be afforded an opportunity to clean
 * up.
 */
static void button_pressed(button_ctx_t *ctx) {
  binding_t binding;
  get_binding_at_index(ctx - button_ctx, &binding);
  dali_dev_data_t *dali_dev;
  // enqueue_device_update(EVT_BTN_PRESSED, ctx);

  int idx = ctx - button_ctx;
  int fixture = idx / NUM_BUTTONS_PER_FIXTURE;
  int button_id = idx % NUM_BUTTONS_PER_FIXTURE;

  switch (binding.type) {
  case BINDING_TYPE_DALI:
    // Dali non-fadeable toggles on press.
    dali_dev = dali_devices + binding.address;
    if (!dali_is_fadeable(dali_dev)) {
      // defer_log(tag, "Button %d/%d Pressed. Toggling non-fadeable DALI device
      // %d", fixture, button_id, dali_dev->addr);
      dali_toggle(dali_dev->addr);
    }
    break;
  case BINDING_TYPE_MODBUS:
    // Modbus always toggles upon first press.
    // defer_log(tag, "Button %d/%d Pressed. binding to MODBUS address %d",
    // fixture, button_id, binding.address);
    modbus_downstream_set_coil(1, binding.address, 0x5500, modbus_press_done);
    break;
  default:
    // Do nothing
    break;
  }

  if (binding.type == BINDING_TYPE_MODBUS ||
      (binding.type == BINDING_TYPE_DALI &&
       dali_devices[binding.address].min ==
           dali_devices[binding.address].max)) {
    // Its an toggle-on-press kind of binding.
  }

  ctx->velocity = -ctx->velocity; // Switch direction
  ctx->countdown = ctx->velocity ? MS_TO_COUNTDOWN(250) : MS_TO_COUNTDOWN(750);
}

bool is_button_pressed(int fixture, int button) {
  int index = fixture * NUM_BUTTONS_PER_FIXTURE + button;

  return !button_ctx[index].released;
}

static void button_timeout_check(button_ctx_t *ctx) {
  binding_t binding;
  dali_dev_data_t *dali_dev;

  if (ctx->countdown && --(ctx->countdown) == 0) {
    if (ctx->released) {
      // Reset button to default state.
      ctx->velocity = 0;
      ctx->num_repeats = 0;
    } else {
      ctx->num_repeats++;
      ctx->countdown = MS_TO_COUNTDOWN(250);
      if (ctx->velocity == 0) {
        // Set it to dimming downwards - Clear the DIMDIR bit and set the
        // DIMMING bit
        ctx->velocity = -1;
      }
      get_binding_at_index(ctx - button_ctx, &binding);
      // enqueue_device_update(EVT_BTN_HELD, ctx);

      // Only DALI devices are dimmable
      if (binding.type == BINDING_TYPE_DALI) {
        dali_dev = dali_devices + binding.address;
        if (dali_is_fadeable(dali_dev)) {
          // defer_log(tag, "Fading DALI device %d direction %d",
          // binding.address, ctx->velocity);
          dali_fade(binding.address, ctx->velocity);
        } else {
          if (ctx->num_repeats == 1) {
            // defer_log(tag, "Ignoring long hold for non-fadeable DALI device
            // %d", ctx-button_ctx);
          }
        }
      } else {
        if (ctx->num_repeats == 1) {
          // defer_log(tag, "Ignoring long hold of button %d", ctx-button_ctx);
        }
      }
    }
  }
}

static void button_released(button_ctx_t *ctx) {
  binding_t binding;
  dali_dev_data_t *dali_dev;

  get_binding_at_index(ctx - button_ctx, &binding);
  // enqueue_device_update(EVT_BTN_RELEASED, ctx);

  if (ctx->num_repeats == 0) {
    // We released before the countdown timer went off.
    ctx->countdown = 0;

    if (binding.type == BINDING_TYPE_DALI) {
      // log_i("Toggling Dali");
      dali_dev = dali_devices + binding.address;
      if (dali_is_fadeable(dali_dev)) {
        dali_toggle(binding.address);
      }
    }
  } else {
    // Allow for re-press within half a second to change direction.
    ctx->countdown = ctx->velocity ? MS_TO_COUNTDOWN(500) : 0;
  }
}

void buttons_init() {
  critical_section_init(&binding_access_lock);

  // test_flash_config();

  for (int i = 0; i < NUM_BUTTONS_PER_FIXTURE; i++) {
    gpio_init(ROW_BASE_PIN + i);
    gpio_set_dir(ROW_BASE_PIN + i, false);
    gpio_set_pulls(ROW_BASE_PIN + i, true, false);
  }

  gpio_init(BUTTON_SER_PIN);
  gpio_set_dir(BUTTON_SER_PIN, true);
  gpio_init(BUTTON_CLK_PIN);
  gpio_set_dir(BUTTON_CLK_PIN, true);

  gpio_put(BUTTON_CLK_PIN, 0);

  // Set up all of the fixture and button data structures
  button_ctx_t *ctx = button_ctx;
  for (int fixture_idx = 0; fixture_idx < NUM_FIXTURES; fixture_idx++) {
    for (int btn_idx = 0; btn_idx < NUM_BUTTONS_PER_FIXTURE; btn_idx++) {
      ctx->released = true; // Default to released, not dimming
      ctx->velocity = 0;
      ctx->countdown = 0;
      ctx->num_repeats = 0;
      ctx++;
    }

    // While we go, also clear out the first set of readings from the button
    // matrix. The flip flops in the shift registers start in an unknown state,
    // so we can't trust any readings uptil we've done a full loop.
    gpio_put(BUTTON_SER_PIN, fixture_idx < (NUM_FIXTURES - 1));
    sleep_us(2);
    gpio_put(BUTTON_CLK_PIN, 1);
    sleep_us(2);
    gpio_put(BUTTON_CLK_PIN, 0);
  }
  next_fixture_scan_time = get_absolute_time();
}

void buttons_poll() {
  uint32_t val;

  // Wait for our next iteration time - We want to read button once every 10ish
  // milliseconds.
  if (time_reached(next_fixture_scan_time)) {
    next_fixture_scan_time = delayed_by_us(
        next_fixture_scan_time, (SCAN_PERIOD_MS * 1000 / NUM_FIXTURES));

    // next_iter = delayed_by_us(next_iter, BUTTON_SAMPLE_PERIOD);
    // First, clock out the data for the _next_ iteration - 0 for the last one
    // and 1 for all others
    gpio_put(BUTTON_SER_PIN, next_fixture_poll == (NUM_FIXTURES - 1) ? 0 : 1);
    sleep_us(1); // We need to delay between setting data and clocking so that
                 // it will be guaranteed to be correct at the chip
    gpio_put(BUTTON_CLK_PIN, 1); // Clock out the data we just set.
    sleep_us(
        20); // Give the inputs a moment to settle into their correct values.

    // Now read in the data for one fixture.
    val = (gpio_get_all() >> ROW_BASE_PIN);
    // Turn the changes from a bit field into indexes for more easy consumption.
    for (int button = 0; button < NUM_BUTTONS_PER_FIXTURE; button++) {
      int button_val = val & 0x01;
      if (ctx->released != button_val) {
        ctx->released = button_val;
        // Cancel any running task.
        if (ctx->released) {
          button_released(ctx);
        } else {
          button_pressed(ctx);
        }
      } else {
        // See if a button timer has expired.
        button_timeout_check(ctx);
      }
      val >>= 1;
      ctx++;
    }

    // Drive the clock back down low - This forms a handy indicator of how much
    // time in each iteration we've consumed. The clock will be high for the
    // settle time plus procesisng time.
    gpio_put(BUTTON_CLK_PIN, 0);
    if (next_fixture_poll == NUM_FIXTURES - 1) {
      next_fixture_poll = 0;
      ctx = button_ctx;
    } else {
      next_fixture_poll++;
    }
  }
}

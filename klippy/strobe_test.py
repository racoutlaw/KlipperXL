# LED stroboscope for VFA visualization
# Allocates its own neopixel_spi OID and sends MCU command to start/stop
# autonomous TIM13-driven strobe on white LED (side strip driver 1, G channel)
#
# Prusa reference: 60-130 Hz, 13.7% duty (35/255)
#
# Usage:
#   STROBE_START FREQ=75        (default 75 Hz, 14% duty)
#   STROBE_START FREQ=90 DUTY=35
#   STROBE_STOP
#
# Config:
#   [strobe_test]
#   data_pin: PG14        # SPI6 MOSI / WS2812 data (shared with display)
#   mux_pin: PE9          # Mux select (HIGH = side strip)

import logging, re

class StrobeTest:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.gcode = self.printer.lookup_object('gcode')

        # Parse pin config
        ppins = self.printer.lookup_object('pins')
        data_params = ppins.parse_pin(
            config.get('data_pin', 'PG14'), can_invert=False)
        mux_params = ppins.parse_pin(
            config.get('mux_pin', 'PE9'), can_invert=False)

        self.mcu = data_params['chip']
        self._data_pin = data_params['pin']
        self._mux_pin = mux_params['pin']

        # Allocate OID for neopixel_spi
        self.oid = self.mcu.create_oid()
        self.mcu.register_config_callback(self._build_config)

        self._cmd_strobe = None

        self.gcode.register_command('STROBE_START', self.cmd_STROBE_START,
                                    desc="Start LED stroboscope")
        self.gcode.register_command('STROBE_STOP', self.cmd_STROBE_STOP,
                                    desc="Stop LED stroboscope")

    @staticmethod
    def _stm32_pin_id(pin_name):
        # Convert STM32 pin name to numeric ID
        # Matches GPIO(PORT, NUM) macro in stm32/internal.h
        m = re.match(r'P([A-I])(\d+)', pin_name)
        if not m:
            raise Exception("Invalid STM32 pin: %s" % pin_name)
        return (ord(m.group(1)) - ord('A')) * 16 + int(m.group(2))

    def _build_config(self):
        # WS2812 timing (same as neopixel.py)
        bit_max_ticks = self.mcu.seconds_to_clock(0.000004)
        reset_min_ticks = self.mcu.seconds_to_clock(0.000050)

        # Resolve pin names to numeric GPIO IDs for MCU command
        # (MCU uses gpio=%u which needs integers, not pin name strings)
        data_gpio = self._stm32_pin_id(self._data_pin)
        mux_gpio = self._stm32_pin_id(self._mux_pin)

        # 6 bytes = 2 WS2812 drivers × 3 bytes (GRB)
        self.mcu.add_config_cmd(
            "config_neopixel_spi oid=%d gpio=%d mux_gpio=%d"
            " data_size=%d bit_max_ticks=%d reset_min_ticks=%d"
            % (self.oid, data_gpio, mux_gpio,
               6, bit_max_ticks, reset_min_ticks))

        cmd_queue = self.mcu.alloc_command_queue()
        self._cmd_strobe = self.mcu.lookup_command(
            "neopixel_spi_strobe oid=%c freq=%hu duty=%c",
            cq=cmd_queue)

        logging.info("strobe_test: Configured (oid=%d)" % self.oid)

    def cmd_STROBE_START(self, gcmd):
        if self._cmd_strobe is None:
            raise gcmd.error("Strobe MCU command not available")
        freq = gcmd.get_int('FREQ', 75, minval=4, maxval=200)
        duty = gcmd.get_int('DUTY', 35, minval=1, maxval=200)
        self._cmd_strobe.send([self.oid, freq, duty])
        gcmd.respond_info(
            "Strobe: %d Hz, %.1f%% duty (STROBE_STOP to end)"
            % (freq, duty / 255.0 * 100))

    def cmd_STROBE_STOP(self, gcmd):
        if self._cmd_strobe is None:
            raise gcmd.error("Strobe MCU command not available")
        self._cmd_strobe.send([self.oid, 0, 0])
        gcmd.respond_info("Strobe stopped")

def load_config(config):
    return StrobeTest(config)

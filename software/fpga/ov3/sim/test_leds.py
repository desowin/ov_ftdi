from migen import *
from misoc.interconnect.csr_bus import CSRBank
from migen.sim import run_simulation

from ovhw.leds import LED_outputs

import unittest


class TestBench(Module):
    def __init__(self):
        self.l_0_ovr = Signal()
        self.leds_v = Signal(3)
        self.submodules.leds = LED_outputs(self.leds_v, [[self.l_0_ovr], [0], [1]])
        self.submodules.csr = CSRBank(self.leds.get_csrs())

class LEDTests(unittest.TestCase):
    def setUp(self):
        self.tb = TestBench()

    def test_write_direct(self):
        def gen():
            yield from self.tb.csr.bus.write(0, 0x7)
            yield
            lv = yield self.tb.leds_v
            self.assertEqual(lv, 7)

        run_simulation(self.tb, gen())

    def test_muxes_1(self):
        def gen():
            # Set muxes
            yield from self.tb.csr.bus.write(1, 1)
            yield
            yield from self.tb.csr.bus.write(2, 1)
            yield
            yield from self.tb.csr.bus.write(3, 1)
            yield

            # Test that the MUX worked
            lv = yield self.tb.leds_v
            self.assertEqual(lv, 0x4)

            # Test changing an LED results in writing to the mux
            yield self.tb.l_0_ovr.eq(1)
            yield
            self.assertEqual((yield self.tb.leds_v), 5)

            # Test partial mux
            yield from self.tb.csr.bus.write(3, 0)
            yield
            self.assertEqual((yield self.tb.leds_v), 1)

            yield from self.tb.csr.bus.write(0, 0x4)
            yield
            self.assertEqual((yield self.tb.leds_v), 0x5)

        run_simulation(self.tb, gen())


if __name__ == "__main__":
    unittest.main()


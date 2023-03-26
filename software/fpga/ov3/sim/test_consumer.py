from migen import *
from misoc.interconnect.stream import Endpoint
from migen.sim import run_simulation

from ovhw.whacker.consumer import Consumer
from ovhw.whacker.util import dmatpl

from ovhw.ov_types import D_LAST

import unittest

class TestBench(Module):
    def __init__(self):

        class PORT(Module):
            def __init__(self, aw, dw):
                self.adr = Signal(aw)
                self.dat_r = Signal(dw)

                self.sync += self.dat_r.eq(self.adr)

        self.submodules.port = PORT(bits_for(2048), 8)
        self.source = Endpoint(dmatpl(2048))
        self.sink = Endpoint(D_LAST)
        self.debug_discard = Signal()

        self.submodules.c = Consumer(self.port, 2048, self.debug_discard)
        self.comb += [self.source.connect(self.c.sink),
                      self.c.source.connect(self.sink)]

class TestConsumer(unittest.TestCase):
    def setUp(self):
        self.tb = TestBench()

    def testConsumer2(self):
        tests = [
            {"ts": 0xC0DE, "start":0,   "count":  4},
            {"ts": 0xDEADBEEF, "start":555, "count": 77}
        ]

        def srcgen():
            for t in tests:
                yield self.tb.source.payload.ts.eq(t["ts"])
                yield self.tb.source.payload.start.eq(t["start"])
                yield self.tb.source.payload.count.eq(t["count"])
                yield self.tb.source.stb.eq(1)
                yield
                while not (yield self.tb.source.ack):
                    yield
            yield self.tb.source.stb.eq(0)
            yield

        def read_sink():
            while not (yield self.tb.sink.stb):
                yield

            d = yield self.tb.sink.payload.d
            last = yield self.tb.sink.payload.last
            yield

            return (d, last)

        def expect_header(flags, delta_ts, size):
            delta_ts_size = max(0, delta_ts.bit_length() - 1) // 8
            header = bytearray([0xA0, flags, size & 0xFF,
                                (delta_ts_size << 5) | (size & 0x1F00) >> 8])
            for i in range(delta_ts_size + 1):
                header.append((delta_ts >> (i * 8)) & 0xFF)
            for byte in header:
                (d, last) = yield from read_sink()
                self.assertEqual(d, byte)
                self.assertFalse(last)

        def sinkgen():
            prev_ts = 0
            yield self.tb.sink.ack.eq(1)
            yield
            for test in tests:
                delta_ts = test["ts"] - prev_ts
                prev_ts = test["ts"]
                yield from expect_header(0, delta_ts, test["count"])
                for n, ck in enumerate(range(
                    test["start"], test["start"] + test["count"])):

                    (d, last) = yield from read_sink()

                    self.assertEqual(d, ck & 0xFF)
                    self.assertEqual(last, n == test["count"] - 1)
                    self.last = {"d": d, "last": last}

        run_simulation(self.tb, [srcgen(), sinkgen()])

        self.assertEqual(self.last, {"d":119, "last":1})

if __name__ == "__main__":
    unittest.main()

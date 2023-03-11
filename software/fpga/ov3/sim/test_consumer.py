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

        self.submodules.c = Consumer(self.port, 2048)
        self.comb += [self.source.connect(self.c.sink),
                      self.c.source.connect(self.sink)]

class TestConsumer(unittest.TestCase):
    def setUp(self):
        self.tb = TestBench()

    def testConsumer2(self):
        tests = [
            {"start":0,   "count":  4},
            {"start":555, "count": 77}
        ]

        def srcgen():
            for t in tests:
                yield self.tb.source.payload.start.eq(t["start"])
                yield self.tb.source.payload.count.eq(t["count"])
                yield self.tb.source.stb.eq(1)
                yield
                while not (yield self.tb.source.ack):
                    yield
            yield self.tb.source.stb.eq(0)
            yield

        def sinkgen():
            yield self.tb.sink.ack.eq(1)
            yield
            for test in tests:
                for n, ck in enumerate(range(
                    test["start"], test["start"] + test["count"])):

                    while not (yield self.tb.sink.stb):
                        yield

                    d = yield self.tb.sink.payload.d
                    last = yield self.tb.sink.payload.last
                    yield

                    self.assertEqual(d, ck & 0xFF)
                    self.assertEqual(last, n == test["count"] - 1)
                    self.last = {"d": d, "last": last}

        run_simulation(self.tb, [srcgen(), sinkgen()])

        self.assertEqual(self.last, {"d":119, "last":1})

if __name__ == "__main__":
    unittest.main()

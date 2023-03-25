
import unittest

from migen import *
from misoc.interconnect.csr_bus import CSRBank
from misoc.interconnect.stream import Endpoint
from migen.sim import run_simulation, passive

from ovhw.whacker.whacker import Whacker
from ovhw.ov_types import D_LAST, ULPI_DATA_TAG


class TestBench(Module):
    def __init__(self):
        self.submodules.w = Whacker(2048)
        self.submodules.csr = CSRBank(self.w.get_csrs())
        self.registers = {}
        offset = 0
        for csr in self.w.get_csrs():
            self.registers[csr.name.upper()] = offset
            offset += (csr.size + 7 // 8)
        self.source = Endpoint(ULPI_DATA_TAG)
        self.sink = Endpoint(D_LAST)
        self.comb += [self.source.connect(self.w.sink),
                      self.w.source.connect(self.sink)]

    def write_reg(self, name, value):
        yield from self.csr.bus.write(self.registers[name], value)

    @passive
    def reader(self, output):
        yield
        yield self.sink.ack.eq(1)
        while True:
            if (yield self.sink.stb):
                output.append((yield self.sink.payload.d))
            yield

    def gen_packet(self, ts, data):
        yield self.source.stb.eq(1)
        yield self.source.payload.ts.eq(ts)
        yield self.source.payload.is_start.eq(1)
        yield self.source.payload.is_end.eq(0)
        yield self.source.payload.is_err.eq(0)
        yield self.source.payload.is_ovf.eq(0)
        yield
        while not (yield self.source.ack):
            yield
        yield self.source.payload.is_start.eq(0)
        for d in data:
            yield self.source.payload.d.eq(d)
            yield
            while not (yield self.source.ack):
                yield
        yield self.source.payload.is_start.eq(0)
        yield self.source.payload.is_end.eq(1)
        yield
        while not (yield self.source.ack):
            yield
        yield self.source.stb.eq(0)
        yield self.source.payload.is_end.eq(0)


class TestWhacker(unittest.TestCase):
    def setUp(self):
        self.tb = TestBench()

    def test_whacker(self):
        def capture_packet(delta_ts, flags, data):
            packet = bytearray([0xA0, flags])
            ts_length = (max(1, delta_ts.bit_length()) + 7) // 8
            length = (ts_length - 1) << 13 | len(data)
            packet.extend([length & 0xFF, (length & 0xFF00) >> 8])
            for i in range(ts_length):
                packet.append((delta_ts >> (i * 8)) & 0xFF)
            packet.extend(data)
            return packet

        def gen():
            yield from self.tb.write_reg("CFG", 1)
            for i in range(10):
                yield
            for timestamp, data in packets:
                yield from self.tb.gen_packet(timestamp, data)
            yield from self.tb.write_reg("CFG", 0)
            while True:
                busy = yield self.tb.w.consumer.busy
                stb = yield self.tb.w.consumer.sink.stb
                rd = yield self.tb.w.consumer.pos
                wr = yield self.tb.w.producer.produce_write.v
                if (busy == 0) and (stb == 0) and ((rd + 1) % 2048 == wr):
                    break
                yield

        output = bytearray()
        expected = bytearray()
        packets = (
            (16, bytes.fromhex("2d0010")),
            (36, bytes.fromhex("c38006000100004000dd94")),
            (67, bytes.fromhex("d2")),
        )

        prev_ts = 0
        expected.extend(capture_packet(0, 0x10, bytes()))
        for timestamp, data in packets:
            expected.extend(capture_packet(timestamp - prev_ts, 0, data))
            prev_ts = timestamp
        expected.extend(capture_packet(0, 0x20, bytes()))

        run_simulation(self.tb, [gen(), self.tb.reader(output)], vcd_name="testwhacker.vcd")
        self.assertSequenceEqual(expected, output)


if __name__ == '__main__':
    unittest.main()

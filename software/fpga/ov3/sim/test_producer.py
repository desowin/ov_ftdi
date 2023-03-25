import unittest

from migen import *
from migen.fhdl.bitcontainer import bits_for
from misoc.interconnect.stream import Endpoint
from migen.sim import run_simulation, passive

from ovhw.ov_types import ULPI_DATA_TAG

from ovhw.constants import *
from ovhw.whacker.producer import Producer, MAX_PACKET_SIZE
from ovhw.whacker.util import *

class TestBench(Module):
    def __init__(self):
        class PORT(Module):
            def __init__(self, aw, dw):
                self.adr = Signal(aw)
                self.dat_w = Signal(dw)
                self.we = Signal(1)
                self.mem = bytearray(2**aw)

            @passive
            def gen(port):
                while True:
                    writing = yield port.we
                    if writing:
                        w_addr = yield port.adr
                        w_data = yield port.dat_w
                        port.mem[w_addr] = w_data
                    yield

        self.submodules.port = PORT(bits_for(1024), 8)
        self.source = Endpoint(ULPI_DATA_TAG)
        self.sink = Endpoint(dmatpl(1024))
        self.consume_watermark = Signal(max=1024)
        self.ena = Signal(1)

        self.submodules.p = Producer(self.port, 1024, self.consume_watermark, self.ena)
        self.comb += [self.source.connect(self.p.ulpi_sink),
                      self.p.out_addr.connect(self.sink)]

    def packet(self, size=0, st=0, end=1, timestamp=0):
        def _(**kwargs):
            jj = {"is_start":0, "is_end":0, "is_ovf":0, "is_err":0,
                  "d":0,"ts":0}
            jj.update(kwargs)
            for name,value in jj.items():
                yield getattr(self.source.payload, name).eq(value)
            yield
            while not (yield self.source.ack):
                yield

        yield self.source.stb.eq(1)

        yield from _(is_start=1, ts=timestamp)
        for i in range(size):
            yield from _(d=(i+st)&0xFF)

        yield from _(is_end=1)

        yield self.source.stb.eq(0)


class TestProducer(unittest.TestCase):
    def setUp(self):
        self.tb = TestBench()

    def test_producer(self):
        seq = [
            (530, 0, 1, 0xCAFEBA),
            (530, 0x10, 1, 0x120CDEF0),
            (10, 0x20, 4, 0x1234DE0000),
            (10, 0x30, 2, 0x123456DF0123),
            (900, 0x30, 4, 0x12345678E10320),
            (10, 0x30, 2, 0x123456789AE34567)
        ]

        def src_gen():
            # Enable capture and wait few clock cycles to make sure HF0_FIRST is
            # in stuffed packet, i.e. not set on first actual packet
            yield
            yield self.tb.ena.eq(1)
            for i in range(10):
                yield
            # Generate test packets
            for p in seq:
                yield from self.tb.packet(*p)

        # Build a reverse-mapping from bits to constant names
        import ovhw.constants
        flag_names = {}
        for k,v in ovhw.constants.__dict__.items():
            if k.startswith("HF0_"):
                flag_names[v] = k[4:]

        def sink_get_packet(sub_len, sub_base, sub_flags, timestamp):
            # Expected payload length
            calc_len = sub_len if sub_len < MAX_PACKET_SIZE else MAX_PACKET_SIZE

            while not (yield self.tb.sink.stb):
                yield

            # Long delay before packet readout to simulate blocked
            # SDRAM
            for i in range(600):
                yield

            p_timestamp = yield self.tb.sink.payload.ts
            p_first = yield self.tb.sink.payload.flag_first
            p_last = yield self.tb.sink.payload.flag_last
            p_err = yield self.tb.sink.payload.flag_err
            p_ovf = yield self.tb.sink.payload.flag_ovf
            start = yield self.tb.sink.payload.start
            count = yield self.tb.sink.payload.count

            # Reconstruct flags
            p_flags = p_first * HF0_FIRST | p_last * HF0_LAST | \
                      p_err * HF0_ERR | p_ovf * HF0_OVF | \
                      (count > MAX_PACKET_SIZE) * HF0_TRUNC

            yield self.tb.sink.ack.eq(1)
            yield
            yield self.tb.sink.ack.eq(0)

            print("DMAFROM: %04x (%02x)" % (start, count))

            mem = self.tb.port.mem

            # Check that the packet header we read out was what we were
            # expecting
            self.assertEqual(count, calc_len)
            self.assertEqual(p_timestamp, timestamp)

            # Check that the DMA request matched the packet
            self.assertEqual(count, calc_len)

            # Build and print the flags
            e = []
            for i in range(0,16):
                if p_flags & 1<<i and 1<<i in flag_names:
                    e.append(flag_names[1<<i])
            print("\tFlag: %s" % ", ".join(e))

            # Fetch and print the body
            packet = []
            for i in range(start, start + count):
                packet.append(mem[i%1024])
                yield self.tb.consume_watermark.eq(i & (1024-1))
                yield
            print("\t%s" % " ".join("%02x" % i for i in packet))

            # Update the producer watermark
            yield self.tb.consume_watermark.eq((start + count) & (1024-1))
            yield

            # Check the payload matches
            expected_payload = [(sub_base+i) & 0xFF for i in range(0, calc_len)]
            self.assertEqual(expected_payload, packet)

        def sink_gen():
            yield from sink_get_packet(0, 0, 0x10, 0)
            for p in seq:
                yield from sink_get_packet(*p)

        self.ts = 0
        self.tb.sink_gen = sink_gen()
        vcd = None
        #vcd = "test_producer.vcd"
        run_simulation(self.tb, [src_gen(), sink_gen(), self.tb.port.gen()], vcd_name=vcd)


if __name__ == '__main__':
    unittest.main()


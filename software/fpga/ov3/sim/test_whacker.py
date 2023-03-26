
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
    packets = (
        # SETUP 0.0
        (16, bytes.fromhex("2d0010"), False, False),
        # DATA0 Get Device Descriptor
        (20, bytes.fromhex("c38006000100004000dd94"), False, False),
        # NAK (should not be filtered because this is outright wrong as
        # device is not allowed to NAK 8-byte SETUP DATA0 packet)
        (31, bytes.fromhex("5a"), False, False),

        # SETUP 0.0
        (16, bytes.fromhex("2d0010"), False, False),
        # DATA0 Get Device Descriptor
        (20, bytes.fromhex("c38006000100004000dd94"), False, False),
        # ACK
        (31, bytes.fromhex("d2"), False, False),

        # IN 0.0
        (147, bytes.fromhex("690010"), False, False),
        # IN 0.0 - previous IN timed out
        (1965, bytes.fromhex("690010"), True, False),
        # NAK
        (29, bytes.fromhex("5a"), True, False),

        # IN 0.0
        (147, bytes.fromhex("690010"), True, False),
        # NAK
        (29, bytes.fromhex("5a"), True, False),

        # IN 0.0
        (6294, bytes.fromhex("690010"), False, False),
        # SOF 823 (IN transaction timed out)
        (1965, bytes.fromhex("a5377b"), False, True),
        # NAK (should not be filtered because device must not send it after
        # timeout; host never sends NAK so this is outright wrong)
        (29, bytes.fromhex("5a"), False, False),

        # EXT 41.0
        (1440, bytes.fromhex("f02910"), False, False),
        # LPM bLinkState: L1 (Sleep) BESL: 400 us bRemoteWake: Enable
        (20, bytes.fromhex("c34199"), False, False),
        # NYET
        (22, bytes.fromhex("96"), False, False),

        # OUT 41.2
        (3565, bytes.fromhex("e12939"), True, False),
        # DATA0 SCSI Inquiry LUN
        (20, bytes.fromhex("c355534243010000002400000080000612000000240000000000000000000000d659"), True, False),
        # NAK
        (55, bytes.fromhex("5a"), True, False),
        # PING 41.2
        (1239, bytes.fromhex("b42939"), True, False),
        # NAK
        (29, bytes.fromhex("5a"), True, False),
        # PING 41.2
        (1239, bytes.fromhex("b42939"), False, False),
        # ACK
        (29, bytes.fromhex("d2"), False, False),
        # OUT 41.2
        (391, bytes.fromhex("e12939"), False, False),
        # DATA0 SCSI Inquiry LUN
        (20, bytes.fromhex("c355534243010000002400000080000612000000240000000000000000000000d659"), False, False),
        # ACK
        (55, bytes.fromhex("d2"), False, False),
    )

    split_packets = (
        # Currently there is no SPLIT transaction filter so all these are
        # expected to not be filtered. The data is provided as reference in
        # case somebody comes up with SPLIT NAKed transaction filter design

        # Microsoft Corp. Comfort Curve Keyboard 2000 V1.0 SPLIT Control and
        # Interrupt transactions captured on High-Speed hub upstream port

        # NAKed Control Transaction example - SPLIT NAKed transaction filter
        # should filter out all the the packets
        # SPLIT Start Hub: 56 Port: 3 Speed: Low Endpoint Type: Control
        (1191, bytes.fromhex("78388388"), False, False),
        # IN 0.0
        (21, bytes.fromhex("690010"), False, False),
        # ACK (from hub)
        (38, bytes.fromhex("d2"), False, False),
        # SPLIT Complete Hub: 56 Port: 3 Speed: Low Endpoint Type: Control
        (1245, bytes.fromhex("78b88350"), False, False),
        # IN 0.0
        (21, bytes.fromhex("690010"), False, False),
        # NYET
        (38, bytes.fromhex("96"), False, False),
        # SPLIT Start Hub: 56 Port: 3 Speed: Low Endpoint Type: Control
        (1246, bytes.fromhex("78b88350"), False, False),
        # IN 0.0
        (21, bytes.fromhex("690010"), False, False),
        # NAK
        (39, bytes.fromhex("5a"), False, False),

        # ACKed Control Transaction example - SPLIT NAKed transaction filter
        # should pass the packets (except Complete->IN->NYET part)
        # SPLIT Start Hub: 56 Port: 3 Speed: Low Endpoint Type: Control
        (1233, bytes.fromhex("78388388"), False, False),
        # IN 0.0
        (21, bytes.fromhex("690010"), False, False),
        # ACK (from hub)
        (37, bytes.fromhex("d2"), False, False),
        # SPLIT Complete Hub: 56 Port: 3 Speed: Low Endpoint Type: Control
        (1245, bytes.fromhex("78b88350"), False, False),
        # IN 0.0
        (21, bytes.fromhex("690010"), False, False),
        # NYET
        (38, bytes.fromhex("96"), False, False),
        # SOF 831
        (662, bytes.fromhex("a53f0b"), False, True),
        # SPLIT Complete Hub: 56 Port: 3 Speed: Low Endpoint Type: Control
        (3193, bytes.fromhex("78b88350"), False, False),
        # IN 0.0
        (21, bytes.fromhex("690010"), False, False),
        # DATA1 ZLP (hub ACKed it to device, but there is no need for host
        # to ACK it for hub according to USB specification)
        (38, bytes.fromhex("4b0000"), False, False),

        # ACKed and NAKed Interrupt Transaction example - SPLIT NAKed
        # transaction filter should filter all packets related to IN 57.2
        # and pass packets related to IN 57.1
        # SPLIT Start Hub: 56 Port: 3 Speed: Low Endpoint Type: Interrupt
        (28, bytes.fromhex("7838837e"), False, False),
        # IN 57.1
        (21, bytes.fromhex("69b940"), False, False),
        # SPLIT Start Hub: 56 Port: 3 Speed: Low Endpoint Type: Interrupt
        (20, bytes.fromhex("7838837e"), False, False),
        # IN 57.2
        (21, bytes.fromhex("6939d9"), False, False),
        # SOF 1320
        (7411, bytes.fromhex("a52865"), False, True),
        # SOF 1320
        (7501, bytes.fromhex("a52865"), False, True),
        # Split Complete Hub: 56 Port: 3 Speed: Low Endpoint Type: Interrupt
        (28, bytes.fromhex("78b883a6"), False, False),
        # IN 57.1
        (21, bytes.fromhex("69b940"), False, False),
        # DATA0 (hub ACKed it to device, host doesn't ACK to hub)
        (37, bytes.fromhex("c30000170000000000bcd3"), False, False),
        # Split Complete Hub: 56 Port: 3 Speed: Low Endpoint Type: Interrupt
        (38, bytes.fromhex("78b883a6"), False, False),
        # IN 57.2
        (21, bytes.fromhex("6939d9"), False, False),
        # NYET
        (37, bytes.fromhex("96"), False, False),
        # SOF 1320
        (7319, bytes.fromhex("a52865"), False, True),
        # Split Complete Hub: 56 Port: 3 Speed: Low Endpoint Type: Interrupt
        (28, bytes.fromhex("78b883a6"), False, False),
        # IN 57.2
        (21, bytes.fromhex("6939d9"), False, False),
        # NAK
        (37, bytes.fromhex("5a"), False, False),
    )

    def setUp(self):
        self.tb = TestBench()

    def whacker_case(self, packets, debug_filter, filter_nak, filter_sof):
        def capture_packet(magic, delta_ts, flags, data):
            packet = bytearray([magic, flags])
            ts_length = (max(1, delta_ts.bit_length()) + 7) // 8
            length = (ts_length - 1) << 13 | len(data)
            packet.extend([length & 0xFF, (length & 0xFF00) >> 8])
            for i in range(ts_length):
                packet.append((delta_ts >> (i * 8)) & 0xFF)
            packet.extend(data)
            return packet

        def gen(cfg):
            yield from self.tb.write_reg("CFG", cfg)
            for i in range(10):
                yield
            timestamp = 0
            for delta_ts, data, _, _ in packets:
                timestamp += delta_ts
                yield from self.tb.gen_packet(timestamp, data)
            # Disable producer but keep the filter and debug config active.
            # Note that filter debug applies to all packets that were not yet
            # processed by consumer, i.e. it applies not only to new packets
            # but also to all packets still in fifo.
            yield from self.tb.write_reg("CFG", cfg & ~1)
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

        expected.extend(capture_packet(0xA0, 0, 0x10, bytes()))
        delta_ts = 0
        for diff_ts, data, nak_filtered, sof_filtered in packets:
            delta_ts += diff_ts
            if (filter_nak and nak_filtered) or (filter_sof and sof_filtered):
                if debug_filter:
                    expected.extend(capture_packet(0xA2, delta_ts, 0, data))
                    delta_ts = 0
            else:
                expected.extend(capture_packet(0xA0, delta_ts, 0, data))
                delta_ts = 0
        expected.extend(capture_packet(0xA0, 0, 0x20, bytes()))

        vcd = "testwhacker"
        cfg = 0x1
        if debug_filter:
            cfg |= (1 << 1)
            vcd += "+debug"
        if filter_nak:
            cfg |= (1 << 2)
            vcd += "-nak"
        if filter_sof:
            cfg |= (1 << 3)
            vcd += "-sof"
        vcd += ".vcd"

        run_simulation(self.tb, [gen(cfg), self.tb.reader(output)], vcd_name=vcd)
        self.assertSequenceEqual(expected, output)

    def test_whacker(self):
        self.whacker_case(self.packets, False, False, False)

    def test_nak_filter(self):
        self.whacker_case(self.packets, False, True, False)

    def test_sof_filter(self):
        self.whacker_case(self.packets, False, False, True)

    def test_nak_and_sof_filter(self):
        self.whacker_case(self.packets, False, True, True)

    def test_debug_filter(self):
        self.whacker_case(self.packets, True, True, True)

    def test_split_packets(self):
        self.whacker_case(self.split_packets, False, True, True)

if __name__ == '__main__':
    unittest.main()

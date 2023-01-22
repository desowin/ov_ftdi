from migen import *
from misoc.interconnect.stream import Endpoint
from migen.fhdl.bitcontainer import bits_for
from migen.genlib.fsm import FSM, NextState

from ovhw.ov_types import ULPI_DATA_TAG
from ovhw.constants import *
from ovhw.whacker.util import *

# Header format:
# A0 - magic byte
# F0 - flags
# SL SH - packet size (lower 13 bits) and delta timestamp size (3 bits)
# T0 T1 T2 T3 T4 T5 T6 T7 - delta timestamp from previous packet
# d0....dN - captured USB packet data
#
# Delta timestamp size is 1 + value encoded in delta timestamp bits.
# Captured USB packet data is at most MAX_PACKET_SIZE bytes long. Packet
# size can be greater than MAX_PACKET_SIZE, however such size indicate
# errors either on the bus itself (babble) or capture path.
#
# Flags format
#
# F0.0 - ERR  - Line level error (ULPI.RXERR asserted during packet)
# F0.1 - OVF  - RX Path Overflow (can happen on high-speed traffic)
# F0.2 - CLIP - Filter clipped (we do not set yet)
# F0.3 - PERR - Protocol level err (but ULPI was fine, ???)
MAX_HEADER_SIZE = 12
class Producer(Module):

    def __init__(self, wrport, depth, consume_watermark, ena, la_filters=[]):
        self.ulpi_sink = Endpoint(ULPI_DATA_TAG)

        self.out_addr = Endpoint(dmatpl(depth))


        # Produce side
        self.submodules.produce_write = Acc_inc(max=depth)
        self.submodules.produce_header = Acc(max=depth, reset=MAX_HEADER_SIZE)

        self.consume_point = Acc(max=depth)

        # 13 bits (8191) for packet size is more than enough
        # Use the upper 3 bits to encode delta timestamp size
        self.submodules.delta_ts_size = Acc(3)
        self.submodules.size = Acc_inc_sat(13)
        self.submodules.flags = Acc_or(8)

        self.submodules.to_start = Acc(1)

        self.submodules.fsm = FSM()

        has_space = Signal()

        self.comb += has_space.eq(((consume_watermark - self.produce_write.v - 1) & (depth - 1)) > MAX_HEADER_SIZE)

        # Grab packet timestamp at SOP
        pkt_timestamp = Signal(len(self.ulpi_sink.payload.ts))
        self.sync += If(self.ulpi_sink.payload.is_start & self.ulpi_sink.ack,
                pkt_timestamp.eq(self.ulpi_sink.payload.ts))
        # Output variable-length delta timestamp relative to previous packet
        rel_timestamp = Signal(len(self.ulpi_sink.payload.ts))
        self.submodules.delta_timestamp = Acc(len(self.ulpi_sink.payload.ts))
        self.submodules.previous_timestamp = Acc(len(self.ulpi_sink.payload.ts))
        self.sync += rel_timestamp.eq(pkt_timestamp - self.previous_timestamp.v)

        payload_is_rxcmd = Signal()
        self.comb += payload_is_rxcmd.eq(
            self.ulpi_sink.payload.is_start | 
            self.ulpi_sink.payload.is_end | 
            self.ulpi_sink.payload.is_err |
            self.ulpi_sink.payload.is_ovf)

        # Packet first/last bits
        clear_acc_flags = Signal()

        en_last = Signal()
        self.sync += en_last.eq(ena)
        self.submodules.packet_first = Acc(1)
        self.submodules.packet_last = Acc(1)

        # Stuff-packet bit
        # At start-of-capture or end-of-capture, we stuff a packet to
        # indicate the exact time of capture
        stuff_packet = Signal()
        self.comb += stuff_packet.eq(self.packet_first.v | self.packet_last.v)

        self.comb += If(ena & ~en_last, 
            self.packet_first.set(1)).Elif(clear_acc_flags,
            self.packet_first.set(0))

        self.comb += If(~ena & en_last, 
            self.packet_last.set(1)).Elif(clear_acc_flags,
            self.packet_last.set(0))

        flags_ini = Signal(8)
        self.comb += flags_ini.eq(
            Mux(self.packet_last.v, HF0_LAST, 0) |
            Mux(self.packet_first.v, HF0_FIRST, 0)
            )


        # Combine outputs of filters
        la_resets = [f.reset.eq(1) for f in la_filters]
        filter_done = 1
        filter_reject = 0
        for f in la_filters:
            filter_done = f.done & filter_done
            filter_reject = f.reject | filter_reject

        self.fsm.act("IDLE",
                If(
                    ((self.ulpi_sink.stb | self.to_start.v) & ena 
                     | stuff_packet) & has_space,

                    If(~self.to_start.v | (self.ulpi_sink.stb & stuff_packet), self.ulpi_sink.ack.eq(1)),

                    # Produce header points to last written header byte, thus
                    # in IDLE state it points to first captured USB data byte
                    self.produce_write.set(self.produce_header.v),
                    self.size.set(0),
                    self.flags.set(flags_ini),
                    self.to_start.set(0),

                    la_resets,
                    
                    If(self.ulpi_sink.payload.is_start | self.to_start.v,
                        NextState("DATA")

                    ).Elif(stuff_packet,
                        # Capture reference timestamp
                        self.delta_timestamp.set(0),
                        self.previous_timestamp.set(self.ulpi_sink.payload.ts),

                        NextState("WRT0"),
                        clear_acc_flags.eq(1),
                    )

                # If not enabled, we just dump RX'ed data
                ).Elif(~ena,
                    self.ulpi_sink.ack.eq(1)
                )
        )

        def write_hdr(statename, nextname, val):
            self.fsm.act(statename, 
                    NextState(nextname),
                    self.produce_header.set(self.produce_header.v - 1),
                    wrport.adr.eq(self.produce_header.v - 1),
                    wrport.dat_w.eq(val),
                    wrport.we.eq(1)
                    )
        

        do_filter_write = Signal()

        # Feed data to lookaside filters
        for f in la_filters:
            self.comb += [
                f.write.eq(do_filter_write),
                f.dat_w.eq(self.ulpi_sink.payload)
            ]
            
        packet_too_long = Signal()
        self.comb += packet_too_long.eq(self.size.v >= MAX_PACKET_SIZE)

        self.fsm.act("DATA",
                If(has_space & self.ulpi_sink.stb,
                    self.ulpi_sink.ack.eq(1),
                    If(payload_is_rxcmd,

                        # Got another start-of-packet
                        If(self.ulpi_sink.payload.is_start,
                            self.flags._or(HF0_OVF),

                            # If we got a SOP, we need to skip RXCMD det in IDLE
                            self.to_start.set(1)

                        # Mark error if we hit an error
                        ).Elif(self.ulpi_sink.payload.is_err,
                            self.flags._or(HF0_ERR),

                        # Mark overflow if we got a stuffed overflow
                        ).Elif(self.ulpi_sink.payload.is_ovf,
                            self.flags._or(HF0_OVF)
                        ),

                        # In any case (including END), we're done RXing
                        NextState("waitdone")
                    ).Else(
                        self.size.inc(),
                        If(packet_too_long,
                            self.flags._or(HF0_TRUNC)
                        ).Else(
                            self.produce_write.inc(),
                            wrport.adr.eq(self.produce_write.v),
                            wrport.dat_w.eq(self.ulpi_sink.payload.d),
                            wrport.we.eq(1),
                            do_filter_write.eq(1)
                        )
                    )
                )
            )
        

        self.fsm.act("waitdone",
                If(filter_done,
                    If(filter_reject,
                        NextState("IDLE")
                    ).Else(
                        self.previous_timestamp.set(pkt_timestamp),
                        self.delta_timestamp.set(rel_timestamp),
                        If(rel_timestamp[56:64],
                            self.delta_ts_size.set(7),
                            NextState("WRT7")
                        ).Elif(rel_timestamp[48:56],
                            self.delta_ts_size.set(6),
                            NextState("WRT6")
                        ).Elif(rel_timestamp[40:48],
                            self.delta_ts_size.set(5),
                            NextState("WRT5")
                        ).Elif(rel_timestamp[32:40],
                            self.delta_ts_size.set(4),
                            NextState("WRT4")
                        ).Elif(rel_timestamp[24:32],
                            self.delta_ts_size.set(3),
                            NextState("WRT3")
                        ).Elif(rel_timestamp[16:24],
                            self.delta_ts_size.set(2),
                            NextState("WRT2")
                        ).Elif(rel_timestamp[8:16],
                            self.delta_ts_size.set(1),
                            NextState("WRT1")
                        ).Else(
                            self.delta_ts_size.set(0),
                            NextState("WRT0")
                        ),
                        clear_acc_flags.eq(1))
                ))

        # Write header beginning with last header byte
        write_hdr("WRT7", "WRT6", self.delta_timestamp.v[56:64])
        write_hdr("WRT6", "WRT5", self.delta_timestamp.v[48:56])
        write_hdr("WRT5", "WRT4", self.delta_timestamp.v[40:48])
        write_hdr("WRT4", "WRT3", self.delta_timestamp.v[32:40])
        write_hdr("WRT3", "WRT2", self.delta_timestamp.v[24:32])
        write_hdr("WRT2", "WRT1", self.delta_timestamp.v[16:24])
        write_hdr("WRT1", "WRT0", self.delta_timestamp.v[8:16])
        write_hdr("WRT0", "WRSH", self.delta_timestamp.v[:8])

        # Write size field
        write_hdr("WRSH", "WRSL", self.delta_ts_size.v[:3] << 5 | self.size.v[8:13])
        write_hdr("WRSL", "WRF0", self.size.v[:8])

        # Write flags field
        write_hdr("WRF0", "WH0", self.flags.v[:8])

        # Write header magic byte
        write_hdr("WH0", "SEND", 0xA0)

        self.fsm.act("SEND",
            self.out_addr.stb.eq(1),
            self.out_addr.payload.start.eq(self.produce_header.v),
            self.out_addr.payload.count.eq(self.produce_write.v - self.produce_header.v),
            If(self.out_addr.ack,
                # Reserve space for (worst case) next packet header
                self.produce_header.set(self.produce_write.v + MAX_HEADER_SIZE),
                NextState("IDLE")
            )
        )


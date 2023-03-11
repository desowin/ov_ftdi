from migen import *
from misoc.interconnect.csr import AutoCSR, CSRStatus, CSRStorage
from misoc.interconnect.stream import SyncFIFO

from ovhw.whacker.consumer import Consumer
from ovhw.whacker.producer import Producer
from ovhw.whacker.util import dmatpl

from ovhw.constants import *

class Whacker(Module, AutoCSR):
    def __init__(self, depth):
        self._cfg = CSRStorage(1)

        debug_signals = 1

        storage = Memory(8, depth)
        self.specials += storage

        wrport = storage.get_port(write_capable=True)
        self.specials += wrport
        rdport = storage.get_port(async_read=False)
        self.specials += rdport

        self.submodules.consumer = Consumer(rdport, depth)
        self.submodules.producer = Producer(wrport, depth, self.consumer.pos, self._cfg.storage[0])

        self.submodules.pkt_fifo = SyncFIFO(dmatpl(depth), 8)

        self.sink = self.producer.ulpi_sink
        self.comb += [
            self.producer.out_addr.connect(self.pkt_fifo.sink),
            self.pkt_fifo.source.connect(self.consumer.sink),
        ]
        self.source = self.consumer.source

        # Debug signals for state tracing
        if debug_signals:
            self._cons_lo = CSRStatus(8)
            self._cons_hi = CSRStatus(8)
            self._prod_lo = CSRStatus(8)
            self._prod_hi = CSRStatus(8)
            self._prod_hd_lo = CSRStatus(8)
            self._prod_hd_hi = CSRStatus(8)
            self._size_lo = CSRStatus(8)
            self._size_hi = CSRStatus(8)

            self._prod_state = CSRStatus(8)
            self._cons_status = CSRStatus(8)

            self._last_start_lo = CSRStatus(8)
            self._last_start_hi = CSRStatus(8)
            self._last_count_lo = CSRStatus(8)
            self._last_count_hi = CSRStatus(8)
            self._last_pw_lo = CSRStatus(8)
            self._last_pw_hi = CSRStatus(8)

            self.sync += [
                    self._cons_lo.status.eq(self.consumer.pos[:8]),
                    self._cons_hi.status.eq(self.consumer.pos[8:]),
                    self._prod_lo.status.eq(self.producer.produce_write.v[:8]),
                    self._prod_hi.status.eq(self.producer.produce_write.v[8:]),
                    self._prod_hd_lo.status.eq(self.producer.produce_header.v[:8]),
                    self._prod_hd_hi.status.eq(self.producer.produce_header.v[8:]),

                    self._size_lo.status.eq(self.producer.size.v[:8]),
                    self._size_hi.status.eq(self.producer.size.v[8:]),
                    self._cons_status.status[0].eq(self.consumer.busy),
                    #self._prod_state.status.eq(self.producer.fsm.state),

                    If(self.producer.out_addr.stb & self.producer.out_addr.ack,
                        self._last_start_lo.status.eq(self.producer.out_addr.payload.start[:8]),
                        self._last_start_hi.status.eq(self.producer.out_addr.payload.start[8:]),
                        self._last_count_lo.status.eq(self.producer.out_addr.payload.count[:8]),
                        self._last_count_hi.status.eq(self.producer.out_addr.payload.count[8:]),
                        self._last_pw_lo.status.eq(self.producer.produce_write.v[:8]),
                        self._last_pw_hi.status.eq(self.producer.produce_write.v[8:]),
                        )
                    ]

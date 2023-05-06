from migen import *
from migen.genlib.cdc import MultiReg
from misoc.interconnect.csr import CSR, CSRStorage, CSRStatus, AutoCSR

class _ULPI_cmd_reg(Module, CSR):
    B_GO = 7

    def __init__(self, trig, ack, addr):
        CSR.__init__(self, size=8)
        self.trig = trig
        self.trig.reset = 0
        self.ack = ack
        self.addr = addr

        self.sync += [
                If(self.re & self.r[self.B_GO],
                    self.trig.eq(1)
                ).Elif(self.ack,
                    self.trig.eq(0)
                ),

                If(self.re,
                    self.addr.eq(self.r[0:6])
                )]

        self.comb += self.w.eq(Cat(self.addr, self.ack, self.trig))

class ULPICfg(Module, AutoCSR):
    def __init__(self, clk, cd_rst, ulpi_rst, ulpi_stp_ovr, ulpi_reg, fs_pre):

        # TESTING - UCFG_RST register
        #  - BRST: reseting of code in the ULPI cd
        #  - URST: reset of external phy
        #  - FSTP: Force assert of STP during reset
        #   (should be part of ULPI cd bringup code)
        #
        # Note: resetting ULPI clock domain without resetting PHY may result
        # in cached transceiver reset value mismatch. The mismatch will be
        # seen by incorrectly reported captured packets speed. It is expected
        # that user understands this limitation and takes appropriate action
        # (e.g. reconfigure transceiver speed after reset).
        #
        # Format:
        #   
        #    7    6    5    4    3    2    1    0
        #    ---------------------------------------
        #    0    0    0    0    0    FSTP BRST URST
        #

        self._rst = CSRStorage(3)
        self.comb += ulpi_rst.eq(self._rst.storage[0])
        self.comb += cd_rst.eq(self._rst.storage[1])
        self.comb += ulpi_stp_ovr.eq(self._rst.storage[2])


        # TESTING - UCFG_STAT register
        # 
        # CKACT: single status bit that indicates whether 
        # the ULPI phy is providing a 60mhz clock signal on clk
        #
        # Format:
        #   
        #    7    6    5    4    3    2    1    0
        #    ----------------------------------------
        #    0    0    0    0    0    0    0    CKACT
        #

        self._stat = CSRStatus(1)

        ulpi_clk_act_cd = Signal(8)

        # Resample ulpi_clk in sys
        sys_ulpi_clk = Signal(1)
        self.specials += MultiReg(clk, sys_ulpi_clk)

        # Edge detect the ULPI clk
        last_ulpi_clk = Signal(1)
        self.sync += [
                last_ulpi_clk.eq(sys_ulpi_clk),
                
                # On detected transistions, reset the countdown
                If(last_ulpi_clk != sys_ulpi_clk, 
                    ulpi_clk_act_cd.eq(0)
                ).Elif(
                    ulpi_clk_act_cd < 0xFF, ulpi_clk_act_cd.eq(ulpi_clk_act_cd + 1)
                )]

        self.comb += self._stat.status.eq(ulpi_clk_act_cd != 0xFF)

        # ULPI_xDATA and xCMD registers
        #
        # Used for reading/writing ULPI regs
        # 
        # ULPI_xDATA Format: just 8 bit data reg
        #
        # ULPI_xCMD Format:
        #   
        #    7    6    5    4    3    2    1    0
        #    ----------------------------------------
        #    GO   0    UA5  UA4  UA3  UA2  UA1  UA0
        #
        # GO:
        #    - write 1 to start ULPI reg transaction
        #    - stays 1 while transaction in progress
        #    - clears to 0 when transaction complete
        #
        # UA5..0 - ULPI register address
        
        # To do a write: write UCFG_WDATA with the value
        # and then write ULPI_WCMD with GO | addr
        # 
        # To read: write ULPI_RCMD with GO | addr, poll 
        # until GO clear, then read ULPI_RDATA

        self._wdata = CSRStorage(8)
        self._wcmd = _ULPI_cmd_reg(ulpi_reg.wreq, ulpi_reg.wack, ulpi_reg.waddr)
        self.submodules += self._wcmd
        self.comb += ulpi_reg.wdata.eq(self._wdata.storage)

        self._rdata = CSRStatus(8)
        self._rcmd = _ULPI_cmd_reg(ulpi_reg.rreq, ulpi_reg.rack, ulpi_reg.raddr)
        self.submodules += self._rcmd
        self.sync += If(ulpi_reg.rack,
                self._rdata.status.eq(ulpi_reg.rdata))

        # UCFG_CAPTURE register
        #
        # FSPRE: automatically switch transceiver to Low-Speed after PRE
        #
        # Format:
        #
        #    7    6    5    4    3    2    1    0
        #    ----------------------------------------
        #    0    0    0    0    0    0    0    FSPRE

        self._capture = CSRStorage(8)
        self.submodules += self._capture
        self.comb += fs_pre.eq(self._capture.storage[0])

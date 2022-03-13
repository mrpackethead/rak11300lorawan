import sx1262_parameters as sx_p
import helpers
from sx126x import SX126X

_SX126X_PA_CONFIG_SX1262 = const(0x00)

class SX1262(SX126X):
    TX_DONE = sx_p.IRQ_TX_DONE
    RX_DONE = sx_p.IRQ_RX_DONE
    ADDR_FILT_OFF = sx_p.GFSK_ADDRESS_FILT_OFF
    ADDR_FILT_NODE = sx_p.GFSK_ADDRESS_FILT_NODE
    ADDR_FILT_NODE_BROAD = sx_p.GFSK_ADDRESS_FILT_NODE_BROADCAST
    PREAMBLE_DETECT_OFF = sx_p.GFSK_PREAMBLE_DETECT_OFF
    PREAMBLE_DETECT_8 = sx_p.GFSK_PREAMBLE_DETECT_8
    PREAMBLE_DETECT_16 = sx_p.GFSK_PREAMBLE_DETECT_16
    PREAMBLE_DETECT_24 = sx_p.GFSK_PREAMBLE_DETECT_24
    PREAMBLE_DETECT_32 = sx_p.GFSK_PREAMBLE_DETECT_32
    STATUS = sx_p.ERROR

    def __init__(self, cs=13, irq=29, rst=14, gpio=15, clk=10, mosi=11, miso=12):
        super().__init__(cs, irq, rst, gpio, clk, mosi, miso)
        self._callbackFunction = self._dummyFunction

    def begin(self, freq=434.0, bw=125.0, sf=9, cr=7, syncWord=sx_p.SYNC_WORD_PRIVATE,
              power=14, currentLimit=60.0, preambleLength=8, implicit=False, implicitLen=0xFF,
              crcOn=True, txIq=False, rxIq=False, tcxoVoltage=1.6, useRegulatorLDO=False,
              blocking=True):
        state = super().begin(bw, sf, cr, syncWord, currentLimit, preambleLength, tcxoVoltage, useRegulatorLDO, txIq, rxIq)
        helpers.ASSERT(state)

        if not implicit:
            state = super().explicitHeader()
        else:
            state = super().implicitHeader(implicitLen)
        helpers.ASSERT(state)

        state = super().setCRC(crcOn)
        helpers.ASSERT(state)

        state = self.setFrequency(freq)
        helpers.ASSERT(state)

        state = self.setOutputPower(power)
        helpers.ASSERT(state)

        state = super().fixPaClamping()
        helpers.ASSERT(state)

        state = self.setBlockingCallback(blocking)

        return state

    def beginFSK(self, freq=434.0, br=48.0, freqDev=50.0, rxBw=156.2, power=14, currentLimit=60.0,
                 preambleLength=16, dataShaping=0.5, syncWord=[0x2D, 0x01], syncBitsLength=16,
                 addrFilter=sx_p.GFSK_ADDRESS_FILT_OFF, addr=0x00, crcLength=2, crcInitial=0x1D0F, crcPolynomial=0x1021,
                 crcInverted=True, whiteningOn=True, whiteningInitial=0x0100,
                 fixedPacketLength=False, packetLength=0xFF, preambleDetectorLength=sx_p.GFSK_PREAMBLE_DETECT_16,
                 tcxoVoltage=1.6, useRegulatorLDO=False,
                 blocking=True):
        state = super().beginFSK(br, freqDev, rxBw, currentLimit, preambleLength, dataShaping, preambleDetectorLength, tcxoVoltage, useRegulatorLDO)
        helpers.ASSERT(state)

        state = super().setSyncBits(syncWord, syncBitsLength)
        helpers.ASSERT(state)

        if addrFilter == sx_p.GFSK_ADDRESS_FILT_OFF:
            state = super().disableAddressFiltering()
        elif addrFilter == sx_p.GFSK_ADDRESS_FILT_NODE:
            state = super().setNodeAddress(addr)
        elif addrFilter == sx_p.GFSK_ADDRESS_FILT_NODE_BROADCAST:
            state = super().setBroadcastAddress(addr)
        else:
            state = sx_p.ERR_UNKNOWN
        helpers.ASSERT(state)

        state = super().setCRC(crcLength, crcInitial, crcPolynomial, crcInverted)
        helpers.ASSERT(state)

        state = super().setWhitening(whiteningOn, whiteningInitial)
        helpers.ASSERT(state)

        if fixedPacketLength:
            state = super().fixedPacketLengthMode(packetLength)
        else:
            state = super().variablePacketLengthMode(packetLength)
        helpers.ASSERT(state)

        state = self.setFrequency(freq)
        helpers.ASSERT(state)

        state = self.setOutputPower(power)
        helpers.ASSERT(state)

        state = super().fixPaClamping()
        helpers.ASSERT(state)

        state = self.setBlockingCallback(blocking)

        return state

    def setFrequency(self, freq, calibrate=True):
        if freq < 150.0 or freq > 960.0:
            return sx_p.ERR_INVALID_FREQUENCY

        state = sx_p.ERR_NONE

        if calibrate:
            data = bytearray(2)
            if freq > 900.0:
                data[0] = sx_p.CAL_IMG_902_MHZ_1
                data[1] = sx_p.CAL_IMG_902_MHZ_2
            elif freq > 850.0:
                data[0] = sx_p.CAL_IMG_863_MHZ_1
                data[1] = sx_p.CAL_IMG_863_MHZ_2
            elif freq > 770.0:
                data[0] = sx_p.CAL_IMG_779_MHZ_1
                data[1] = sx_p.CAL_IMG_779_MHZ_2
            elif freq > 460.0:
                data[0] = sx_p.CAL_IMG_470_MHZ_1
                data[1] = sx_p.CAL_IMG_470_MHZ_2
            else:
                data[0] = sx_p.CAL_IMG_430_MHZ_1
                data[1] = sx_p.CAL_IMG_430_MHZ_2
            state = super().calibrateImage(data)
            helpers.ASSERT(state)
sy
        return super().setFrequencyRaw(freq)

    def setOutputPower(self, power):
        if not ((power >= -9) and (power <= 22)):
            return sx_p.ERR_INVALID_OUTPUT_POWER

        ocp = bytearray(1)
        ocp_mv = memoryview(ocp)
        state = super().readRegister(sx_p.REG_OCP_CONFIGURATION, ocp_mv, 1)
        helpers.ASSERT(state)

        state = super().setPaConfig(0x04, _sx_p.PA_CONFIG_SX1262)
        helpers.ASSERT(state)

        state = super().setTxParams(power)
        helpers.ASSERT(state)

        return super().writeRegister(sx_p.REG_OCP_CONFIGURATION, ocp, 1)

    def setTxIq(self, txIq):
        self._txIq = txIq

    def setRxIq(self, rxIq):
        self._rxIq = rxIq
        if not self.blocking:
            helpers.ASSERT(super().startReceive())

    def setPreambleDetectorLength(self, preambleDetectorLength):
        self._preambleDetectorLength = preambleDetectorLength
        if not self.blocking:
            helpers.ASSERT(super().startReceive())

    def setBlockingCallback(self, blocking, callback=None):
        self.blocking = blocking
        if not self.blocking:
            state = super().startReceive()
            helpers.ASSERT(state)
            if callback != None:
                self._callbackFunction = callback
                super().setDio1Action(self._onIRQ)
            else:
                self._callbackFunction = self._dummyFunction
                super().clearDio1Action()
            return state
        else:
            state = super().standby()
            helpers.ASSERT(state)
            self._callbackFunction = self._dummyFunction
            super().clearDio1Action()
            return state

    def recv(self, len=0, timeout_en=False, timeout_ms=0):
        if not self.blocking:
            return self._readData(len)
        else:
            return self._receive(len, timeout_en, timeout_ms)

    def send(self, data):
        if not self.blocking:
            return self._startTransmit(data)
        else:
            return self._transmit(data)

    def _events(self):
        return super().getIrqStatus()

    def _receive(self, len_=0, timeout_en=False, timeout_ms=0):
        state = sx_p.ERR_NONE
        
        length = len_
        
        if len_ == 0:
            length = sx_p.MAX_PACKET_LENGTH

        data = bytearray(length)
        data_mv = memoryview(data)

        try:
            state = super().receive(data_mv, length, timeout_en, timeout_ms)
        except AssertionError as e:
            state = list(sx_p.ERROR.keys())[list(sx_p.ERROR.values()).index(str(e))]

        if state == sx_p.ERR_NONE or state == sx_p.ERR_CRC_MISMATCH:
            if len_ == 0:
                length = super().getPacketLength(False)
                data = data[:length]

        else:
            return b'', state

        return  bytes(data), state

    def _transmit(self, data):
        if isinstance(data, bytes) or isinstance(data, bytearray):
            pass
        else:
            return 0, sx_p.ERR_INVALID_PACKET_TYPE

        state = super().transmit(data, len(data))
        return len(data), state

    def _readData(self, len_=0):
        state = sx_p.ERR_NONE

        length = super().getPacketLength()

        if len_ < length and len_ != 0:
            length = len_

        data = bytearray(length)
        data_mv = memoryview(data)

        try:
            state = super().readData(data_mv, length)
        except AssertionError as e:
            state = list(sx_p.ERROR.keys())[list(sx_p.ERROR.values()).index(str(e))]

        helpers.ASSERT(super().startReceive())

        if state == sx_p.ERR_NONE or state == sx_p.ERR_CRC_MISMATCH:
            return bytes(data), state

        else:
            return b'', state

    def _startTransmit(self, data):
        if isinstance(data, bytes) or isinstance(data, bytearray):
            pass
        else:
            return 0, sx_p.ERR_INVALID_PACKET_TYPE

        state = super().startTransmit(data, len(data))
        return len(data), state

    def _dummyFunction(self, *args):
        pass

    def _onIRQ(self, callback):
        events = self._events()
        if events & sx_p.IRQ_TX_DONE:
            super().startReceive()
        self._callbackFunction(events)

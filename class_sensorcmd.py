import time
from class_ch341 import *
from enum import *


class ClassSensorCmd:
    def __init__(self, ch341):
        # Command definitions
        self.CMD_GET_CHANNEL_NUM = 0x01  # Get number of channels
        self.CMD_GET_SENSOR_CAP_DATA = 0x60  # Get sensor data
        self.CMD_SET_SENSOR_UART_SEND_TYPE = 0x61  # Set UART output mode
        self.CMD_GET_SENSOR_CHANNEL = 0x62  # Get number of channels
        self.CMD_SET_SENSOR_AUTO_DAC = 0x63  # Auto DAC
        self.CMD_GET_SENSOR_ERR_CODE = 0x64  # Get error code
        self.CMD_SET_SENSOR_CHANNEL_CALIBRATE = 0x65  # Set channel weight calibration
        self.CMD_GET_SENSOR_CHANNEL_CALIBRATE = 0x66  # Get channel weight calibration
        self.CMD_SET_SENSOR_AUTO_THRESHOLD = 0x67  # Set auto threshold
        self.CMD_GET_SENSOR_THRESHOLD = 0x68  # Get auto threshold
        self.CMD_SET_SENSOR_THRESHOLD = 0x69  # Set channel threshold
        self.CMD_SET_SENSOR_TOUCH_THRESHOLD = 0x6A  # Set pressure threshold
        self.CMD_SET_SENSOR_CHANNEL_CALIBRATE_QUICK = 0x6B  # Quick max force calibration
        self.CMD_GET_SENSOR_TEST_HZ = 0x6C  # Get sampling rate
        self.CMD_GET_SENSOR_CALI_N_LIST = 0x6D  # Get calibration list
        self.CMD_SET_SENSOR_CHANNEL_GROUP_CALIBRATE = 0x6E  # Set group channel calibration

        self.CMD_SET_SENSOR_IIC_ADDR = 0x70  # Set I2C address
        self.CMD_GET_SENSOR_IIC_ADDR = 0x71  # Get I2C address
        self.CMD_SET_SENSOR_CDC_SYNC = 0x72  # Stop acquisition for synchronization
        self.CMD_SET_SENSOR_CDC_START_OFFSET = 0x73  # Set capacitance start offset
        self.CMD_GET_SENSOR_TEMP = 0x74  # Get temperature
        self.CMD_SET_SENSOR_CDC_CFG_SAVE = 0x75  # Save CDC configuration
        self.CMD_SET_SENSOR_FACTOR = 0x76  # Reset to factory settings
        self.CMD_SET_SENSOR_RESTART = 0x77  # Software reset
        self.CMD_SET_SENSOR_WEIGHT_CALIBRATE = 0x78  # Set weight calibration
        self.CMD_SET_SENSOR_BASE_RESET = 0x79  # Reset baseline
        self.CMD_SET_SENSOR_UNIFORMIZATION = 0x7A  # Uniformization
        self.CMD_SET_SENSOR_CHANNEL_ENABLE = 0x7B  # Enable channel
        self.CMD_SET_SENSOR_CHANNEL_BASE_RESET = 0x7C  # Reset individual channel
        self.CMD_GET_SENSOR_WEIGHT_CALIBRATE = 0x7D  # Get weight calibration
        self.CMD_GET_SENSOR_CHANNEL_STATE = 0x7E  # Get channel state
        self.CMD_SET_SENSOR_SEND_TYPE = 0x7F  # Set data transfer type

        self.CMD_GET_VERSION = 0xA0  # Get software version
        self.CMD_SOFT_RESTART = 0xA1  # Software reset
        self.CMD_GET_TYPE = 0xA2  # Get device type
        self.CMD_SET_TYPE = 0xA3  # Set device type
        self.CMD_SET_INF = 0xA5  # Set output interface
        self.CMD_GET_PRG = 0xA6  # Get project type

        self._ch341 = ch341
        self.sendTimePre = [time.time(), time.time()]
        self.sendTimeNow = [time.time(), time.time()]
        self.sendCnt = [0, 0]

    # Calculate checksum
    def calcSum(self, pack):
        if len(pack) <= 5:
            return 0
        _sum = sum((b & 0xFF) for b in pack)
        pack.append(_sum & 0xFF)
        pack.append((_sum >> 8) & 0xFF)

    # Verify checksum
    # return: True if valid, False otherwise
    def checkSum(self, pack):
        if len(pack) <= 5:
            return False
        _sum = sum((b & 0xFF) for b in pack[:-2])
        chkL = _sum & 0xFF
        chkH = (_sum >> 8) & 0xFF
        rchkL = pack[-2] & 0xFF
        rchkH = pack[-1] & 0xFF
        return chkL == rchkL and chkH == rchkH

    # Set sensor I2C address
    def setAddr(self, addr, new_addr):
        _pack = [0xAA, 0x55, 0x03, self.CMD_SET_SENSOR_IIC_ADDR, 0x00, 0x00, 0x00, new_addr, 0x00]
        self.calcSum(_pack)
        self._ch341.write(addr, _pack)
        _pack = list(range(11))
        time.sleep(0.01)
        self._ch341.read(new_addr, _pack)
        if self.checkSum(_pack) and (self.CMD_SET_SENSOR_IIC_ADDR | 0x80) == c_uint8(_pack[3]).value:
            return _pack[7] & 0xFF
        return 0

    def getAddr(self, addr):
        _pack = [0xAA, 0x55, 0x03, self.CMD_GET_SENSOR_IIC_ADDR, 0x00, 0x00, 0x00, 0x00, 0x00]
        self.calcSum(_pack)
        self._ch341.write(addr, _pack)
        _pack = list(range(11))
        time.sleep(0.01)
        self._ch341.read(addr, _pack)
        if self.checkSum(_pack):
            return _pack[7] & 0xFF
        return 0

    def getSensorVersion(self, addr):
        pass

    # Get number of sensor channels
    def getSensorNum(self, addr):
        _pack = [0xAA, 0x55, 0x03, self.CMD_GET_CHANNEL_NUM, 0x00, 0x00, 0x00, 0x00, 0x00]
        self.calcSum(_pack)
        self._ch341.write(addr, _pack)
        _pack = list(range(15))
        time.sleep(0.01)
        self._ch341.read(addr, _pack)
        if self.checkSum(_pack):
            return (_pack[5] & 0xFF) + (_pack[6] & 0xFF) * 256
        return 0

    # Get project ID
    def getSensorProjectIdex(self, addr):
        _pack = [0xAA, 0x55, 0x03, self.CMD_GET_PRG, 0x00, 0x00, 0x00, 0x00, 0x00]
        self.calcSum(_pack)
        self._ch341.write(addr, _pack)
        _pack = list(range(11))
        time.sleep(0.01)
        self._ch341.read(addr, _pack)
        if self.checkSum(_pack):
            return (_pack[7] & 0xFF) + (_pack[8] & 0xFF) * 256
        return 0

    # Set sensor data send type
    def setSensorSendType(self, addr, sendType):
        _pack = [0xAA, 0x55, 0x03, self.CMD_SET_SENSOR_SEND_TYPE, 0x00, 0x00, 0x00, sendType, 0x00]
        self.calcSum(_pack)
        self._ch341.write(addr, _pack)
        _pack = list(range(11))
        time.sleep(0.01)
        self._ch341.read(addr, _pack)
        if self.checkSum(_pack) and (self.CMD_SET_SENSOR_SEND_TYPE | 0x80) == c_uint8(_pack[3]).value:
            return True
        return False

    # Set capacitance offset for acquisition
    def setSensorCapOffset(self, addr, offset):
        _pack = [0xAA, 0x55, 0x03, self.CMD_SET_SENSOR_CDC_START_OFFSET, 0x00, 0x00, 0x00, offset, 0x00]
        self.calcSum(_pack)
        self._ch341.write(addr, _pack)
        _pack = list(range(6))
        time.sleep(0.01)
        self._ch341.read(addr, _pack)
        if self.checkSum(_pack) and (self.CMD_SET_SENSOR_CDC_START_OFFSET | 0x80) == c_uint8(_pack[3]).value:
            return True
        return False

    # Get sensor capacitance data
    def getSensorCapData(self, addr, buf):
        tarLen = len(buf)
        err = self._ch341.read(addr, buf)
        if err == 0:
            print("get data err")
        valid = self.checkSum(buf)

        if len(buf) != tarLen:
            buf.clear()
            buf.extend(range(tarLen))

        if len(buf) == tarLen and (buf[0] & 0xFF) == 0x55 and (buf[1] & 0xFF) == 0xAA and valid:
            return True
        return False

    # Set acquisition sync
    def setSensorSync(self, addr):
        _pack = [0xAA, 0x55, 0x03, self.CMD_SET_SENSOR_CDC_SYNC, 0x00, 0x00, 0x00, 0x00, 0x00]
        self.calcSum(_pack)
        self._ch341.write(addr, _pack)
        return True

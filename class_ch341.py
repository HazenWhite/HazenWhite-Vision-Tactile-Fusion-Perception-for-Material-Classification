import time
import os
import sys
from ctypes import *
import ctypes
import glob

# ch341 class: I2C read/write, INT pin read/write, speed setting
class ClassCh341:
    # Fixed interface macros
    _mCH341A_CMD_I2C_STREAM = 0xAA       # I2C command packet, followed by I2C command stream
    _mCH341A_CMD_I2C_STM_STA = 0x74	    # I2C command stream: generate start condition
    _mCH341A_CMD_I2C_STM_STO = 0x75		# I2C command stream: generate stop condition
    _mCH341A_CMD_I2C_STM_OUT = 0x80		# I2C command stream: output data, bits 5-0 indicate length, followed by data bytes; if length is 0, send one byte and return ACK
    _mCH341A_CMD_I2C_STM_IN = 0xC0		# I2C command stream: input data, bits 5-0 indicate length, if 0, receive one byte and send NACK
    _mCH341A_CMD_I2C_STM_MAX = 63        # Max input/output data length per I2C command
    _mCH341A_CMD_I2C_STM_SET = 0x60		# I2C command stream: set parameters, bit 2=SPI I/O mode (0=single, 1=dual), bits 1-0=I2C speed (00=low, 01=standard, 10=fast, 11=high)
    _mCH341A_CMD_I2C_STM_US = 0x40		# I2C command stream: delay in microseconds, bits 3-0 for delay value
    _mCH341A_CMD_I2C_STM_MS = 0x50		# I2C command stream: delay in milliseconds, bits 3-0 for delay value
    _mCH341A_CMD_I2C_STM_DLY = 0x0F		# Max delay value for a single I2C command
    _mCH341A_CMD_I2C_STM_END = 0x00		# I2C command stream: end command packet early

    _mStateBitINT = 0x00000400

    IIC_SPEED_20 = 0
    IIC_SPEED_100 = 1
    IIC_SPEED_400 = 2
    IIC_SPEED_750 = 3

    def __init__(self):
        self.deviceID = ctypes.c_uint32()
        pass

    def init(self):
        if os.name == 'nt':  # Windows environment
            libPath = os.path.dirname(sys.argv[0]) + r'/lib/ch341/CH341DLLA64.DLL'
        elif os.name == 'posix':
            libPath = './lib/ch341/libch347.so'
        dllExist = os.path.exists(libPath)
        if not dllExist:
            print('Library file not found')
            return False
        else:
            try:
                if os.name == 'nt':
                    self.ic = windll.LoadLibrary(libPath)  # Load ch341 interface

                    self.ch341GetInput = self.ic.CH341GetInput
                    self.ch341CloseDevice = self.ic.CH341CloseDevice
                    self.ch341WriteData = self.ic.CH341WriteData
                    self.ch341WriteRead = self.ic.CH341WriteRead
                    self.ch341SetOutput = self.ic.CH341SetOutput
                    self.ch341SetStream = self.ic.CH341SetStream

                elif os.name == 'posix':
                    self.ic = cdll.LoadLibrary(libPath)

                    self.ch341GetInput = self.ic.CH34xGetInput
                    self.ch341CloseDevice = self.ic.CH34xCloseDevice
                    self.ch341WriteData = self.ic.CH34xWriteData
                    self.ch341WriteRead = self.ic.CH34xWriteRead
                    self.ch341SetOutput = self.ic.CH34xSetOutput
                    self.ch341SetStream = self.ic.CH34xSetStream

                print("ch341 loaded successfully")
                return True
            except Exception as e:
                print("Failed to load ch341")
                return False

    # Check if ch341 is connected
    # return: False = not connected, True = connected
    def open(self):
        if os.name == 'nt':
            try:
                self.fd = self.ic.CH341OpenDevice(0)
                if self.fd == -1:
                    print("CH341 device open failed on Windows.")
                    return False
                else:
                    self.fd = 0  # TODO: implement port scanning
                    print("CH341 device opened successfully on Windows.")
                    return True
            except Exception as e:
                print(f"Error occurred while opening CH341 device on Windows: {e}")
                return False

        elif os.name == 'posix':
            try:
                devices = glob.glob('/dev/ch34x_pis*')  # Dynamically search for device
                if devices:
                    device_path = devices[0].encode()
                    self.fd = self.ic.CH34xOpenDevice(device_path)
                    if self.fd == -1:
                        print("CH341 device open failed on Linux.")
                        return False
                    else:
                        print("CH341 device opened successfully on Linux.")
                        return True
                else:
                    print("No CH341 device found on Linux.")
                    return False
            except Exception as e:
                print(f"Error occurred while opening CH341 device on Linux: {e}")
                return False

        else:
            print("Unsupported operating system.")
            return False

    def disconnect(self):
        self.ch341CloseDevice(self.fd)

    def connectCheck(self):
        if True == self.ch341GetInput(self.fd, ctypes.byref(self.deviceID)):
            return True
        else:
            return False

    # I2C write
    # addr: I2C slave address
    # data: list of bytes to write
    # return: number of bytes written (may be inaccurate)
    def write(self, addr, data):
        sLen = len(data)
        tmpData = []
        tmpLen = sLen

        pack = []
        cnt = 20
        packNum = sLen // cnt
        sLen %= cnt

        tmpData.extend(data)

        pack.append(self._mCH341A_CMD_I2C_STREAM)
        pack.append(self._mCH341A_CMD_I2C_STM_STA)
        pack.append(self._mCH341A_CMD_I2C_STM_OUT | 1)
        pack.append(addr << 1)
        for i in range(0, packNum):
            pack.append(self._mCH341A_CMD_I2C_STM_OUT | cnt)
            pack.extend(tmpData[0:20])
            del tmpData[0:20]
            pack.append(self._mCH341A_CMD_I2C_STM_END)
            sendBuf = (c_byte * len(pack))()
            for j in range(0, len(pack)):
                sendBuf[j] = pack[j]
            sendLen = (c_byte * 1)()
            sendLen[0] = len(pack)
            if not self.ch341WriteData(self.fd, sendBuf, sendLen):
                return 0
            if sendLen == 0:
                return 0
            pack.clear()
            pack.append(self._mCH341A_CMD_I2C_STREAM)
        if sLen >= 1:
            pack.append(self._mCH341A_CMD_I2C_STM_OUT | sLen)
            pack.extend(tmpData[0:sLen])
        pack.append(self._mCH341A_CMD_I2C_STM_STO)
        pack.append(self._mCH341A_CMD_I2C_STM_END)
        sendBuf = (c_byte * len(pack))()
        for j in range(0, len(pack)):
            sendBuf[j] = pack[j]
        sendLen = (c_byte * 1)()
        sendLen[0] = len(pack)
        if not self.ch341WriteData(self.fd, sendBuf, sendLen):
            return 0
        if sendLen == 0:
            return 0
        return tmpLen

    # I2C read
    # addr: I2C slave address
    # data: buffer to store read bytes, length determines how many bytes to read
    # return: number of bytes read (may be inaccurate)
    def read(self, addr, data):
        if id(data) == 0 or len(data) == 0:
            return 0
        rLen = len(data)
        pack = []
        readBuf = []
        readLen = 0

        packNum = rLen // 30
        rLen %= 30
        if rLen == 0:
            rLen = 30
            packNum -= 1
        pack.append(self._mCH341A_CMD_I2C_STREAM)
        pack.append(self._mCH341A_CMD_I2C_STM_STA)
        pack.append(self._mCH341A_CMD_I2C_STM_OUT | 1)
        pack.append((addr << 1) | 0x01)
        pack.append(self._mCH341A_CMD_I2C_STM_MS | 1)
        for i in range(0, packNum):
            pack.append(self._mCH341A_CMD_I2C_STM_IN | 30)
            pack.append(self._mCH341A_CMD_I2C_STM_END)
            sendBuf = (c_byte * len(pack))()
            for j in range(0, len(pack)):
                sendBuf[j] = pack[j]
            recLen = (c_byte * 1)()
            recBuf = (c_byte * self._mCH341A_CMD_I2C_STM_MAX)()
            if not self.ch341WriteRead(self.fd, len(pack), sendBuf, self._mCH341A_CMD_I2C_STM_MAX, 1, recLen, recBuf):
                return 0
            if recLen == 0:
                return 0
            for j in range(0, recLen[0]):
                readBuf.append(recBuf[j])
            readLen += 30
            pack.clear()
            pack.append(self._mCH341A_CMD_I2C_STREAM)
        if rLen > 1:
            pack.append(self._mCH341A_CMD_I2C_STM_IN | (rLen - 1))
        pack.append(self._mCH341A_CMD_I2C_STM_IN | 0)
        pack.append(self._mCH341A_CMD_I2C_STM_STO)
        pack.append(self._mCH341A_CMD_I2C_STM_END)
        sendBuf = (c_byte * len(pack))()
        for j in range(0, len(pack)):
            sendBuf[j] = pack[j]
        recLen = (c_byte * 1)()
        recBuf = (c_byte * self._mCH341A_CMD_I2C_STM_MAX)()
        if not self.ch341WriteRead(self.fd, len(pack), sendBuf, self._mCH341A_CMD_I2C_STM_MAX, 1, recLen, recBuf):
            return 0
        if recLen[0] == 0:
            return 0
        for j in range(0, recLen[0]):
            readBuf.append(recBuf[j])
        data.clear()
        data.extend(readBuf)
        readLen = len(pack)
        return readLen

    # Set INT pin state
    # lvl: 1 for high level, 0 for low level
    def set_int(self, lvl):
        status = (c_long * 1)()
        self.ic.CH341GetInput(0, status)
        time.sleep(0.01)
        if lvl:
            self.ch341SetOutput(self.fd, 0x03, 0xFF00, status[0] | self._mStateBitINT)
        else:
            self.ch341SetOutput(self.fd, 0x03, 0xFF00, status[0] & (~self._mStateBitINT))

    # Read INT pin state
    # return: 1 for high level, 0 for low level
    def get_int(self):
        status = (c_long * 1)()
        self.ic.CH341GetInput(0, status)
        return (status[0] & self._mStateBitINT) >> 10

    # Set I2C speed
    # return: False if error, True if success
    def set_speed(self, speed):
        if speed not in [self.IIC_SPEED_20, self.IIC_SPEED_100, self.IIC_SPEED_400, self.IIC_SPEED_750]:
            return False
        if not self.ch341SetStream(self.fd, speed | 0):
            print("speed err")
            return False
        else:
            return True

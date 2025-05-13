from class_sensorcmd import *
from sensorPara import *
import struct

# Capacitance data structure
class capData:
    def __init__(self):
        self.sensorIndex = 0            # Sensor index, same as I2C address
        self.channelCapData = list()    # Raw channel values
        self.tf = list()                # Tangential force array
        self.tfDir = list()             # Tangential force direction array
        self.nf = list()                # Normal force array
        self.sProxCapData = list()      # Self-capacitance proximity data
        self.mProxCapData = list()      # Mutual-capacitance proximity data
        self.cnt = 0                    # Counter for testing

    def init(self, addr, yddsNum, sProxNum, mProxNum, capChannelNum):
        self.sensorIndex = addr                         # Sensor index, same as I2C address
        self.channelCapData = list(range(capChannelNum))# Raw channel values
        self.tf = list(range(yddsNum))                  # Tangential force array
        self.tfDir = list(range(yddsNum))               # Tangential force direction array
        self.nf = list(range(yddsNum))                  # Normal force array
        self.sProxCapData = list(range(sProxNum))       # Self-capacitance proximity data
        self.mProxCapData = list(range(mProxNum))       # Mutual-capacitance proximity data
        self.cnt = 0                                     # Counter for testing

    def deinit(self):
        self.channelCapData = None
        self.tf = None
        self.tfDir = None
        self.nf = None
        self.sProxCapData = None
        self.mProxCapData = None

# Sensor class: encapsulates sensor-related parameters
class ClassFinger:
    def __init__(self, pca_idx, ch341):
        self.snsCmd = ClassSensorCmd(ch341)
        self.pcaIdx = pca_idx   # I2C enable chip index, starts from 2
        self.readData = capData()
        self.disconnected()

    # Check whether the sensor is connected; if the read/write address is correct, assume connected
    def checkSensor(self):
        # Broadcast to read current sensor address and default the I2C address to match PCA index
        addrRead = self.snsCmd.getAddr(0)
        if addrRead != self.pcaIdx:
            if self.pcaIdx != self.snsCmd.setAddr(addrRead, self.pcaIdx):
                return False
            else:
                addrRead = self.pcaIdx

        # Set the data transmission type to raw values
        if True != self.snsCmd.setSensorSendType(addrRead, 0):
            pass

        # Set the capacitance acquisition sequence; allocate sequence based on address
        if self.snsCmd.setSensorCapOffset(addrRead, addrRead) != True:
            pass

        # In actual use, only define parameters based on the sensor; no need to read project ID
        projectRead = self.snsCmd.getSensorProjectIdex(addrRead)
        findProjectFlg = False
        if projectRead > 0:
            for pro in finger_params:
                if pro.prg == projectRead:
                    self.projectPara = pro
                    findProjectFlg = True

        if findProjectFlg == False:
            pass

        self.connected(addrRead)
        return True

    # Sensor connected: initialize parameters
    def connected(self, addr):
        self.addr = addr
        self.connect = True
        self.connectTimer = time.time()
        self.packIdx = 0
        self.data = list()
        self.data.extend(range(self.projectPara.pack_len))

        self.readData.init(addr, self.projectPara.ydds_num, self.projectPara.s_prox_num, self.projectPara.m_prox_num, self.projectPara.sensor_num)

    # Sensor disconnected: reset parameters
    def disconnected(self):
        self.addr = 0xFF              # I2C address
        self.connect = False          # Connection flag
        self.packIdx = 0              # Sample index
        self.connectTimer = 0         # Connection timeout timer

        self.projectPara = finger_params[0]
        self.readData.deinit()

    def capRead(self):
        for retry in range(0, 3):
            if self.snsCmd.getSensorCapData(self.addr, self.data) == True:
                if self.data[5] != self.projectPara.sensor_num:
                    pass

                if self.data[4] != self.packIdx:
                    self.packIdx = self.data[4]
                    self.connectTimer = time.time()

                    # Extract channel data based on byte size
                    if self.projectPara.cap_byte == 4:
                        for j in range(0, self.projectPara.sensor_num):
                            self.readData.channelCapData[j] = ((self.data[6 + j * 4] & 0xFF) +
                                                               ((self.data[6 + j * 4 + 1] & 0xFF) << 8) +
                                                               ((self.data[6 + j * 4 + 2] & 0xFF) << 16) +
                                                               ((self.data[6 + j * 4 + 3] & 0xFF) << 24))
                    else:
                        for j in range(0, self.projectPara.sensor_num):
                            self.readData.channelCapData[j] = ((self.data[6 + j * 3] & 0xFF) +
                                                               ((self.data[6 + j * 3 + 1] & 0xFF) << 8) +
                                                               ((self.data[6 + j * 3 + 2] & 0xFF) << 16))

                    yddsOffset = 6 + self.projectPara.sensor_num * self.projectPara.cap_byte

                    if self.projectPara.ydds_type == 2:
                        struct_size = sizeof(DynamicYddsComTs)
                        for i in range(self.projectPara.ydds_num):
                            offset = yddsOffset + i * struct_size
                            struct_data = self.data[offset: offset + struct_size]
                            struct_data = [value & 0xFF for value in struct_data]
                            struct_data = bytes(struct_data)
                            instance = DynamicYddsComTs.from_buffer_copy(struct_data)
                            self.readData.nf[i] = instance.nf
                            self.readData.tf[i] = instance.tf
                            self.readData.tfDir[i] = instance.tfDir
                            self.readData.sProxCapData[i] = instance.prox
                    elif self.projectPara.ydds_type == 4:
                        struct_size = sizeof(DynamicYddsU16Ts)
                        for i in range(self.projectPara.ydds_num):
                            offset = yddsOffset + i * struct_size
                            struct_data = self.data[offset: offset + struct_size]
                            struct_data = [value & 0xFF for value in struct_data]
                            struct_data = bytes(struct_data)
                            instance = DynamicYddsU16Ts.from_buffer_copy(struct_data)
                            self.readData.nf[i] = instance.nf / 1000.0
                            self.readData.tf[i] = instance.tf / 1000.0
                            self.readData.tfDir[i] = instance.tfDir

                        sProxOffset = yddsOffset + self.projectPara.ydds_num * struct_size
                        for i in range(self.projectPara.s_prox_num):
                            self.readData.sProxCapData[i] = ((self.data[sProxOffset + i * 3] & 0xFF) +
                                                             ((self.data[sProxOffset + i * 3 + 1] & 0xFF) << 8) +
                                                             ((self.data[sProxOffset + i * 3 + 2] & 0xFF) << 16))
                        mProxOffset = yddsOffset + self.projectPara.ydds_num * struct_size
                        for i in range(self.projectPara.m_prox_num):
                            self.readData.mProxCapData[i] = ((self.data[mProxOffset + i * 3] & 0xFF) +
                                                             ((self.data[mProxOffset + i * 3 + 1] & 0xFF) << 8) +
                                                             ((self.data[mProxOffset + i * 3 + 2] & 0xFF) << 16))

                break
            else:
                pass

        # Timeout: no data received for 2 seconds
        if (time.time() - self.connectTimer) > 2:
            self.disconnected()

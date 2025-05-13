from enum import *
import threading
import queue  # Import queue module
from class_ch341 import *
from class_sensorcmd import *
from class_finger import *
from collections import namedtuple
import socket
import time
import threading
import matplotlib.pyplot as plt
from collections import deque

# Create a queue to transfer data
data_queue = queue.Queue()

# Count number of samples
update_interval = 1  # Update plot every 100 samples
sample_count = 0     # Count the number of samples
update_flag = False  # Flag to indicate when to update plot

# Store data of the most recent 100 time points
max_len = 100
time_series = deque(maxlen=max_len)
avg_series = deque(maxlen=max_len)
max_series = deque(maxlen=max_len)
min_series = deque(maxlen=max_len)

DEF_CDC_SYNC_MS = 1000  # Capacitance sync interval
DEF_GET_CAP_MS  = 30    # Capacitance read interval
DEF_PRO_CYC = 100

DEF_MAX_FINGER_NUM = 2  # Number of fingers to connect, up to 5

# Define a global queue for inter-thread communication
capReadQueue = queue.Queue()

# CH341 communication
class EnumCh341ConnectStatus(Enum):
    CH341_CONNECT_INIT = 0
    CH341_CONNECT_OPEN = 1
    CH341_CONNECT_SET_SPEED = 2
    CH341_CONNECT_SAMPLE_START = 3
    CH341_CONNECT_CHECK = 4
    CH341_CONNECT_SAMPLE_STOP = 5

class ClassCapRead:
    def __init__(self):
        self.ch341 = ClassCh341()

        # Connect up to 5 fingers
        self.fingers = list()  # List of sensor objects
        for i in range(DEF_MAX_FINGER_NUM):
            self.fingers.append(ClassFinger(2 + i, self.ch341))

        self.currCh341State = 0  # Current CH341 state
        self.prevCh341State = 0  # Previous CH341 state

        self.ch341CheckTimer = 0
        self.mcuInit = 0
        self.pcaAddr = 0x70  # I2C control chip address

        self.ch341Init = 0  # CH341 initialization flag

        self.syncTimer = 0

        self.connectStatus = EnumCh341ConnectStatus.CH341_CONNECT_INIT

        self.connectDebug()

    def __del__(self):
        self.disConnectDebug()
        self.ch341.disconnect()
        print("CH341 released")
        pass

    def connectDebug(self):
        # Connect to debug server
        self.vofaClient = socket.socket()
        addr = ('127.0.0.1', 1347)
        try:
            self.vofaClient.connect(addr)
            self.socketConnected = True
            print('Connected to server successfully')
        except Exception as e:
            self.socketConnected = False
            print('Failed to connect to server')

    def disConnectDebug(self):
        if self.vofaClient:
            self.vofaClient.close()

    def debugPrint(self):
        if self.socketConnected == True:
            fingerIndex = 0
            _log1 = ""
            # Output raw channel values
            # for index in range(0, self.fingers[fingerIndex].projectPara.sensor_num):
            #     _log1 += str(self.fingers[fingerIndex].readData.channelCapData[index])
            #     _log1 += ','

            for index in range(0, self.fingers[fingerIndex].projectPara.ydds_num):
                _log1 += str(int(self.fingers[fingerIndex].readData.nf[index]*1000))
                _log1 += ','
                _log1 += str(int(self.fingers[fingerIndex].readData.tf[index]*1000))
                _log1 += ','
                _log1 += str(self.fingers[fingerIndex].readData.tfDir[index])
                _log1 += ','
            for index in range(0, self.fingers[fingerIndex].projectPara.s_prox_num):
                _log1 += str(self.fingers[fingerIndex].readData.sProxCapData[index])
                _log1 += ','
            for index in range(0, self.fingers[fingerIndex].projectPara.m_prox_num):
                _log1 += str(self.fingers[fingerIndex].readData.mProxCapData[index])
                _log1 += ','

            _log1 += str(0)
            _log1 += '\r\n'
            if self.socketConnected == 1:
                self.vofaClient.send(_log1.encode())

    def set_sensor_enable(self, idx):
        _pack = list()
        _pack.append(idx)
        self.ch341.write(self.pcaAddr, _pack)

    def ch341Connect(self):
        if self.connectStatus == EnumCh341ConnectStatus.CH341_CONNECT_INIT:
            if(True == self.ch341.init()):
                self.connectStatus = EnumCh341ConnectStatus.CH341_CONNECT_OPEN
        elif self.connectStatus == EnumCh341ConnectStatus.CH341_CONNECT_OPEN:
            if(True == self.ch341.open()):
                self.connectStatus = EnumCh341ConnectStatus.CH341_CONNECT_SET_SPEED
            else:
                self.connectStatus = EnumCh341ConnectStatus.CH341_CONNECT_INIT
        elif self.connectStatus == EnumCh341ConnectStatus.CH341_CONNECT_SET_SPEED:
            if(True == self.ch341.set_speed(self.ch341.IIC_SPEED_400)):
                self.connectStatus = EnumCh341ConnectStatus.CH341_CONNECT_SAMPLE_START
            else:
                print("set speed err")
                self.connectStatus = EnumCh341ConnectStatus.CH341_CONNECT_SAMPLE_START

        elif self.connectStatus == EnumCh341ConnectStatus.CH341_CONNECT_SAMPLE_START:
            self.connectStatus = EnumCh341ConnectStatus.CH341_CONNECT_CHECK
            self.timer = threading.Timer(DEF_GET_CAP_MS / 1000, self.capRead)
            self.timer.start()
        elif self.connectStatus == EnumCh341ConnectStatus.CH341_CONNECT_CHECK:
            self.ch341CheckTimer += DEF_PRO_CYC
            if(self.ch341CheckTimer >= 1000):
                self.ch341CheckTimer = 0
                if(False == self.ch341.connectCheck()):
                    print("CH341 disconnected")
                    self.connectStatus = EnumCh341ConnectStatus.CH341_CONNECT_SAMPLE_STOP
        elif self.connectStatus == EnumCh341ConnectStatus.CH341_CONNECT_SAMPLE_STOP:
            self.syncTimer = 0
            for i in range(0, len(self.fingers)):
                self.fingers[i].disconnected()
            self.connectStatus = EnumCh341ConnectStatus.CH341_CONNECT_INIT
        else:
            self.connectStatus = EnumCh341ConnectStatus.CH341_CONNECT_INIT

    def capRead(self):
        global sample_count, update_flag
        capReadTime = time.time()
        ms_capReadTime = capReadTime

        connectedSensorChan = 0
        connectedSensorCnt = 0
        for fingerIndex in range(0, len(self.fingers)):
            self.set_sensor_enable(1 << (self.fingers[fingerIndex].pcaIdx))
            connectedSensorChan |= 1 << (self.fingers[fingerIndex].pcaIdx)

            if self.fingers[fingerIndex].connect == False:
                if True == self.fingers[fingerIndex].checkSensor():
                    pass  # Sensor reconnected
            else:
                self.fingers[fingerIndex].capRead()
                connectedSensorCnt += 1

                # Extract channel data
                channel_data = self.fingers[fingerIndex].readData.channelCapData
                normal_force_data = self.fingers[fingerIndex].readData.nf
                t_force_data = self.fingers[fingerIndex].readData.tf
                self_cap_data = self.fingers[fingerIndex].readData.sProxCapData
                m_cap_data = self.fingers[fingerIndex].readData.mProxCapData
                t_force_dire_data = self.fingers[fingerIndex].readData.tfDir

                # ✅ Store in `queue` for `main.py` to read
                data_queue.put({
                    "channel_data": channel_data,
                    "normal_force_data": normal_force_data,
                    "t_force_data": t_force_data,
                    "self_cap_data": self_cap_data,
                    "m_cap_data": m_cap_data,
                    "t_force_dire_data": t_force_dire_data
                })

        self.debugPrint()

        # If more than one sensor is connected, configure proximity sync
        if connectedSensorCnt > 1 and (time.time() - self.syncTimer) > DEF_CDC_SYNC_MS:
            self.syncTimer = time.time()
            self.set_sensor_enable(connectedSensorChan)
            for fingerIndex in range(0, len(self.fingers)):
                if self.fingers[fingerIndex].connect is True:
                    self.fingers[fingerIndex].snsCmd.setSensorSync(0)
                    break

        if self.connectStatus == EnumCh341ConnectStatus.CH341_CONNECT_CHECK:
            capReadTime = time.time()
            difftime = int(capReadTime * 1000 - ms_capReadTime * 1000)
            # Restart timer after task completion
            if difftime > DEF_GET_CAP_MS:
                timer = threading.Timer(DEF_GET_CAP_MS / 1000, self.capRead)
            else:
                timer = threading.Timer((DEF_GET_CAP_MS - difftime) / 1000, self.capRead)
            timer.start()


def capReadThread():
    # Main logic of the thread
    cap = ClassCapRead()

    while True:
        cap.ch341Connect()
        time.sleep(DEF_PRO_CYC / 1000)


def main():
    capReadThread()
    # Uncomment the below to use Matplotlib in main thread
    # fig, ax = plt.subplots()
    #
    # cap_read_thread = threading.Thread(target=capReadThread, daemon=True)
    # cap_read_thread.start()
    #
    # while True:
    #     if update_flag:
    #         ax.clear()
    #         ax.plot(time_series, avg_series, label="Avg", linestyle="-")
    #         ax.plot(time_series, max_series, label="Max", linestyle="--")
    #         ax.plot(time_series, min_series, label="Min", linestyle=":")
    #
    #         ax.set_title("Capacitance Data (Updated Every 1 Samples)")
    #         ax.set_xlabel("Time (s)")
    #         ax.set_ylabel("Capacitance Value")
    #         ax.legend()
    #         plt.draw()
    #         plt.pause(0.1)


if __name__ == "__main__":
    main()

from typing import List, Optional
from ctypes import Structure, sizeof, c_float, c_uint32, c_uint16


class DynamicYddsComTs(Structure):
    _pack_ = 1  # Align to 1 byte
    _fields_ = [
        ("nf", c_float),       # Normal force
        ("nfCap", c_uint32),   # Capacitance corresponding to normal force
        ("tf", c_float),       # Tangential force
        ("tfCap", c_uint32),   # Capacitance corresponding to tangential force
        ("tfDir", c_uint16),   # Direction of tangential force
        ("prox", c_uint32),    # Proximity (self-capacitance)
    ]


class DynamicYddsU16Ts(Structure):
    _pack_ = 1  # Align to 1 byte
    _fields_ = [
        ("nf", c_uint16),      # Normal force (16-bit)
        ("tf", c_uint16),      # Tangential force (16-bit)
        ("tfDir", c_uint16),   # Tangential force direction (16-bit)
    ]


# TODO: Add support for other 3D force types


class FingerHeatMap:
    """
    Represents the heatmap configuration of a capacitive sensor finger.
    """
    def __init__(self, rows: int, cols: int, file_path: str, cap_count: int, cap_indices: List[int]):
        self.rows = rows
        self.cols = cols
        self.file_path = file_path
        self.cap_count = cap_count
        self.cap_indices = cap_indices


class FingerParamTS:
    """
    Represents a parameter set for a sensor finger, including configuration for
    force sensing, proximity channels, data size, calibration, etc.
    """
    def __init__(self, prg: int, pack_len: int, sensor_num: int, touch_num: int, ydds_num: int,
                 s_prox_num: int, m_prox_num: int, cap_byte: int, ydds_type: int, had_err: int,
                 cali_num: int, name: str, display_type_para: str, p_heat_map: Optional[List[FingerHeatMap]]):
        self.prg = prg                              # Project ID
        self.pack_len = pack_len                    # Length of data packet
        self.sensor_num = sensor_num                # Number of sensor channels
        self.touch_num = touch_num                  # Number of touch points
        self.ydds_num = ydds_num                    # Number of force channels
        self.s_prox_num = s_prox_num                # Number of self-capacitive proximity channels
        self.m_prox_num = m_prox_num                # Number of mutual-capacitive proximity channels
        self.cap_byte = cap_byte                    # Bytes per capacitance reading
        self.ydds_type = ydds_type                  # Type of 3D force data structure
        self.had_err = had_err                      # Whether the sensor had any error
        self.cali_num = cali_num                    # Number of calibration parameters
        self.name = name                            # Finger name
        self.display_type_para = display_type_para  # Visualization style (e.g. TypeA, TypeB)
        self.p_heat_map = p_heat_map                # Optional heatmap visualization config


# === Finger Heatmap Configuration ===
finger2_power_cap_index = [
    FingerHeatMap(
        16, 8, "TS-F-A/heatMapPara16_8.dat", 7,
        [0, 1, 2, 3, 4, 5, 6, 255, 255, 255, 255, 255, 255, 255, 255, 255]
    )
]

finger17_power_cap_index = [
    FingerHeatMap(
        6, 7, "TS-T-A/weight6X7X6.dat", 6,
        [0, 1, 4, 5, 6, 7, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255]
    ),
    FingerHeatMap(
        6, 7, "TS-T-A/weight6X7X6.dat", 6,
        [2, 3, 8, 9, 10, 11, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255]
    )
]

# === Finger Parameter Table ===
finger_params = [
    FingerParamTS(
        prg=2, pack_len=62, sensor_num=8, touch_num=7, ydds_num=1,
        s_prox_num=1, m_prox_num=0, cap_byte=4, ydds_type=2, had_err=0,
        cali_num=22, name="Universal Finger", display_type_para="TypeA",
        p_heat_map=finger2_power_cap_index
    ),
    FingerParamTS(
        prg=17, pack_len=78, sensor_num=16, touch_num=13, ydds_num=2,
        s_prox_num=2, m_prox_num=1, cap_byte=3, ydds_type=4, had_err=1,
        cali_num=22, name="Two-Finger Large", display_type_para="TypeB",
        p_heat_map=finger17_power_cap_index
    )
]

from pickle import FALSE
import serial
import time
import matplotlib.pyplot as plt
from typing import List
import numpy as np

# oscillo = serial.Serial('/dev/tty.usbserial-14430', 9600, rtscts=True)
# if oscillo.is_open:
#     oscillo.close()
# oscillo.open()
MEMORYNUMBER = {1: "CH1 acquisition memory", 
                   2: "CH2 acquisition memory", 
                   3: "SAVE memory A", 
                   4: "SAVE memory B", 
                   5: "CH1 display memory",
                   6: "CH2 display memory"}

class Conditions:
    channel:int
    vertmode:str
    horimode:str
    A_time_div:float
    A_unit_div:str
    B_time_div:float
    B_unit_div:str
    is_calib:bool
    prob_factor:int
    volt_div:float
    unit_div:str
    delay:float
    n_sweeps:int
    
    def __str__(self) -> str:
        out:str = ''
        out += 'Channel : \t' + MEMORYNUMBER[self.channel] + '\n'
        out += 'Vert. mode : \t' + self.vertmode + '\n'
        out += 'Horiz. mode : \t' + ('single' if self.horimode == 'A' else 'X-Y') + '\n'
        out += 'Time/div : \t' + f'{self.A_time_div} {self.A_unit_div}' + '\n'
        out += 'Volt/div : \t' + f'{self.volt_div * self.prob_factor} {self.unit_div}'
        return  out

class RawMeasurement:
    channel:int
    addr:int
    size:int
    data:List[int]

class Measurments:
    cond:Conditions
    rawdata:RawMeasurement

class Oscilloscope:

    ser_port:serial.Serial
    DEL:str
    baudrate:int
    MEMOYNUMBER = {1: "CH1 acquisition memory", 
                   2: "CH2 acquisition memory", 
                   3: "SAVE memory A", 
                   4: "SAVE memory B", 
                   5: "CH1 display memory",
                   6: "CH2 display memory"}
    UNITS_LUT = {'s':'   S', 'm':'  MS', 'u':'MICS', 'n':'  NS'}

    def __init__(self, port:str, control:int) -> None:
        self.baudrate = min(38400 // (2**(control & int('0b11100', 2) // 4)), 9600)
        STOPBIT_LUT = [serial.STOPBITS_TWO, serial.STOPBITS_ONE]
        DELIMITER_LUT = ['\r', '\r\n']
        stopbit = STOPBIT_LUT[int(control & int('0b10', 2)) // 2]
        self.DEL = DELIMITER_LUT[int(control & 1)]
        self.DELstr = 'CR' if self.DEL == '\r' else 'LFCR'
        self.port = port
        self.stopbit = stopbit
        self.ser_port = serial.Serial(port=port, baudrate=self.baudrate, stopbits=stopbit, rtscts=True)

    def __str__(self) -> str:
        return f"On port {self.port} with baud of {self.baudrate} 1-8-{self.stopbit}, DEL : {self.DELstr}"

    def sample(self):
        cmd = self.sample_cmd()
        self.send_cmd(cmd)
    

    def get_measuresing_condition(self, mem_number:int) -> Conditions:
        cmd = self.get_meas_cond_cmd(mem_number=mem_number)
        self.send_cmd(cmd)
        data = self.recv_data(est_size=68)
        return self.decode_mc(data)

    def get_raw_measurement(self, mem_number:int, addr:int, size:int) -> RawMeasurement:
        cmd = self.get_wf_cmd(mem_number=mem_number, addr=addr, size=size)
        self.send_cmd(cmd)
        data = self.recv_data(14+size)
        return self.decode_wf(data)


    def send_cmd(self, cmd:bytes):
        self.ser_port.setRTS(False)
        time.sleep(0.02)
        self.ser_port.write(cmd)
        time.sleep(0.02)

    def recv_data(self, est_size:int) -> bytes:
        time_required = float(est_size) * 8.0 / float(self.baudrate) * 1.2
        self.ser_port.setRTS(True)
        data = self.ser_port.read_until(self.DEL)
        time.sleep(time_required)
        self.ser_port.setRTS(False)
        return data


    def sample_cmd(self) -> bytes:
        return bytes(f"S1{self.DEL}", encoding='ASCII')
    
    def sweep_cmd(self, time_value:float, units:str) -> bytes:
        return bytes(f"TM({time_value:2.1f},{self.UNITS_LUT[units]}){self.DEL}", encoding='ASCII')

    def get_wf_cmd(self, mem_number:int, addr:int, size:int) -> bytes:
        return bytes(f"R{mem_number}({addr:04d},{size:04d},B){self.DEL}", encoding='ASCII')

    def get_meas_cond_cmd(self, mem_number) -> bytes:
        return bytes(f"R0({mem_number}){self.DEL}", encoding='ASCII')

    def decode_wf(self, data: List[bytes]) -> RawMeasurement:
        if len(data) < 14:
            raise ValueError(1, 'Too short')
        
        rawMeas = RawMeasurement()
        try:
            rawMeas.channel = int(data[1])
            rawMeas.addr = int(data[4:8])
            rawMeas.size = int(data[9:13])
        except:
            raise ValueError(2, 'Conversion error', data)
        
        rawMeas.data = [int(v) for v in data[14:len(data)-2]]

        return rawMeas

    def decode_mc(self, data: List[bytes]) -> Conditions:
        # 0         1         2         3         4         5         6         
        # 0123456789012345678901234567890123456789012345678901234567890123456789
        # #1@, CH1,A, 0.5   MS,         ,UNCAL,P10X,10.0  V,0.000 DIV,  1,6F\r
        if len(data) < 64:
            raise ValueError(1, 'Too short')
        
        try:
            cond = Conditions()
            cond.channel = int(data[1])
            cond.vertmode = data[4:8]
            cond.horimode = data[9]
            cond.A_time_div = float(data[11:15])
            cond.A_unit_div = data[16, 20]
            if cond.horimode == 'B':
                cond.B_time_div = float(data[11:15])
                cond.B_unit_div = data[16, 20]
            else:
                cond.B_time_div = None
                cond.B_unit_div = None
            cond.is_calib = False if data[31:36] == 'UNCAL' else True
            cond.prob_factor = 10 if data[37:41] == 'P10X' else 1
            cond.volt_div = float(data[42:46])
            cond.unit_div = data[47:49]
            cond.delay = float(data[50:55])
            cond.n_sweeps = int(data[60:63])
        except:
            raise ValueError(2, 'Conversion error', data)
        
        return cond

if __name__ == "__main__":
    print("coucou")
    oscillo = Oscilloscope('/dev/tty.usbserial-14430', int('0b11001010', 2))
    print(oscillo)
    print(oscillo.get_measuresing_condition(1))

# Import Elveflow library
import sys
import os
from email.header import UTF8
# SDK_HOME = 'C:/Users/Admin/ELVEFLOW/SDK_V3_04_04'
SDK_HOME = 'C:/Users/Admin/ELVEFLOW/ESI_V3_04_01/SDK V3_04_01'
sys.path.append(SDK_HOME+'/DLL64/DLL64')#add the path of the library here
sys.path.append(SDK_HOME+'/Python_64')#add the path of the LoadElveflow.py

from ctypes import *
from array import array

from Elveflow64 import *

# Serial import to talk with Arduino for spectrometer trigger
import serial

# Thorlabs Power meter
from TLPM import TLPM

from time import sleep, time
import datetime
import numpy as np

Z_regulator_type = {
    'none': 0,
    '0_200_mbar': 1,
    '0_2000_mbar': 2,    
    '0_8000_mbar': 3,
    'm1000_1000_mbar': 4,
    'm1000_6000_mbar': 5
}
Z_sensor_type = {
    'none': 0,
    'Flow_1_5_uL_min': 1,
    'Flow_7_uL_min': 2,
    'Flow_50_uL_min': 3,
    'Flow_80_uL_min': 4,
    'Flow_1000_uL_min': 5,
    'Flow_5000_uL_min': 6,
    'Press_340_mbar': 7,
    'Press_1_bar': 8,
    'Press_2_bar': 9,
    'Press_7_bar': 10,
    'Press_16_bar': 11,
    'Level': 12
}

def _check_error(task, error):
    err_dict = {
        -8000: 'No Digital Sensor found',
        -8001: 'No pressure sensor compatible with OB1 MK3',
        -8002: 'No Digital pressure sensor compatible with OB1 MK3+',
        -8003: 'No Digital Flow sensor compatible with OB1 MK3',
        -8004: 'No IPA config for this sensor',
        -8005: 'Sensor not compatible with AF1',
        -8006: 'No Instrument with selected ID'
    }
    if error != 0:
        if error in err_dict:
            raise RuntimeError('{} failed with error code {}\n{}'.format(task, error, err_dict[error]))
        else:
            raise RuntimeError('{} failed with error code {}\n{}'.format(task, error, 'Unknown error, see http://www.ni.com/pdf/manuals/321551a.pdf'))
            
class ob1():
    def __init__(self, address='01EF64C3', calibrate=False):
        self.Instr_ID = c_int32()
        print('Instrument name and regulator types hardcoded in the python script'.encode('utf-8'))
        # see User guide to determine regulator type NI MAX to determine the instrument name
        # channel 1: -1000~1000 mBar, channel 2: none, channel 3: none, channel 4:none
        error = OB1_Initialization(address.encode('ascii'), 
                                   Z_regulator_type['m1000_1000_mbar'], 
                                   Z_regulator_type['none'], 
                                   Z_regulator_type['none'], 
                                   Z_regulator_type['none'], 
                                   byref(self.Instr_ID))
        # all functions will return error code to help you to debug your code, for further information see user guide
        _check_error('OB1 Initialization', error)

        # add one digital flow sensor to OB1 channel 1, 1mL/min, digital, water calibration, 16bit resolution, 
#         error=OB1_Add_Sens(self.Instr_ID, 1, Z_sensor_type['Flow_1000_uL_min'], 1, 0, 7, 0)
        # add one digital flow sensor to OB1 channel 1, 5mL/min, digital, water calibration, 9bit resolution, 
        error=OB1_Add_Sens(self.Instr_ID, 1, Z_sensor_type['Flow_1000_uL_min'], 1, 0, 7, 0)
        _check_error('Adding digital flow sensor', error)

        self.calib_path = os.path.abspath('ob1_calibration.txt')
        print(self.calib_path)
        self.Calib = (c_double * 1000)()
        if calibrate:
            print ('Starting calibration')
            OB1_Calib(self.Instr_ID.value, self.Calib, 1000)
            error = Elveflow_Calibration_Save(self.calib_path.encode('ascii'), byref(self.Calib), 1000)
            print ('Calibration finished')
            print ('Calibration saved in file %s' % self.calib_path.encode('ascii'))
        else:
            if not os.path.isfile(self.calib_path):
                raise IOError('Calibration file "{}" does not exist'.format(self.calib_path))               
            error = Elveflow_Calibration_Load(self.calib_path.encode('ascii'), byref(self.Calib), 1000)
            _check_error('Loading calibration file', error)            
            
        self.set_pressure(0)

    def meas_flowrate(self, channel=1):
        """
        Measures the instantaneous flow rate, on designated port.
        Parameters:
            channel: ob1 channel to set (1-4), defaults to 1
        return: (flowrate, error code)
            flowrate in uL/min
            0 : successful
            -1 : parameter error
            otherwise: NI error
        """
        if channel <1 or channel > 4:
            print('ob1.meas_flowrate: channel must be within 1~4')
            return -1
        
        set_channel = c_int32(channel)  # convert to c_int32
        get_flowrate = c_double(0.0)
        error = OB1_Get_Sens_Data(self.Instr_ID.value, set_channel, Acquire_Data1True0False=0, Sens_Data=byref(get_flowrate))  # Acquire_data =1 -> Read all the analog value
#         print('Press or Flow ch', set_channel.value,': ',get_flowrate.value)
#         _check_error('Getting data from flow sensor', error)
        
        return get_flowrate.value, error

    def set_pressure(self, pressure, channel=1):
        """
        Sets the pressure, on designated port.
        Parameters:
            pressure: target pressure in mBar (-1000 to 8000)
            channel: ob1 channel to set (1-4), defaults to 1
        return: error code
            0 : successful
            -1 : parameter error
            otherwise: NI error
        """      
        
        # check parameters
        if pressure >8000.0 or pressure < -1000.0:
            print('ob1.set_pressure: Error pressure must be within -1000~8000')
            return -1
        if channel <1 or channel > 4:
            print('set_pressure: channel must be within 1~4')
            return -1
        
        set_channel=c_int32(channel)#convert to c_int32        
        set_pressure=c_double(pressure) #convert to c_double        
        error=OB1_Set_Press(self.Instr_ID.value, set_channel, set_pressure, byref(self.Calib),1000) 
        _check_error('Setting pressure', error)
        
        return error
    
    def meas_pressure(self, channel=1):
        """
        Measures the instantaneous pressure, on designated channel.
        Parameters:
            channel: ob1 channel to set (1-4), defaults to 1
        return: (pressure, error code)
            flowrate in mBar
            0 : successful
            -1 : parameter error
            otherwise: NI error
        """
        if channel <1 or channel > 4:
            print('ob1.meas_pressure: channel must be within 1~4')
            return -1
        
        set_channel = c_int32(channel)  # convert to c_int32
        get_pressure = c_double(0.0)
        error = OB1_Get_Press(self.Instr_ID.value, set_channel, Acquire_Data1True0False=0, Calib_array_in=self.Calib, Pressure=byref(get_pressure), Calib_Array_len=1000)  # Acquire_data =1 -> Read all the analog value
        print('Press ch 1: ',get_pressure.value)
#         _check_error('Getting pressure from OB1', error)
        
        return get_pressure.value, error
    
    def __del__(self):
        self.set_pressure(0)
        error=OB1_Destructor(self.Instr_ID.value)
        
class mux_wire():
    def __init__(self, address='Dev2'):
        self.Instr_ID = c_int32()
        # see User guide to determine regulator type NI MAX to determine the instrument name
        error = MUX_Initialization(address.encode('ascii'), byref(self.Instr_ID))
        # all functions will return error code to help you to debug your code, for further information see user guide
        _check_error('MUX Initialization', error)
        print(error)
        
        self.state = [0]*8
        
    def set_all_valves(self, state):
        """
        Sets state of all valves
        Parameters:
            state: 8 integer (0: close, 1: open)
        return: error code
            0 : successful
            -1 : parameter error
            otherwise: NI error
        """
        if not (isinstance(state, list) and len(state) == 8):
            print('Error mux_wire.set_all_valves: state has to be a list of 8 integers')
            return -1
        
        self.state = state
        valve_state=(c_int32*16)(0)
        for i in range(8):
            valve_state[i]=c_int32(state[i]) 
            
        error=MUX_Wire_Set_all_valves(self.Instr_ID.value, valve_state, 16)
        
        return error     
    
    def set_valve(self, channel, state):
        """
        Sets state of a certain valve
        Parameters:
            channel: integer (1-8)
            state: integer (0: close to N.O., 1: open to N.C.)
        return: error code
            0 : successful
            -1 : parameter error
            otherwise: NI error
        """
        if not isinstance(state, int):
            print('Error mux_wire.set_valve: state has to be an integer')
            return -1
        if not (isinstance(channel, int) and channel>0 and channel<9) :
            print('Error mux_wire.set_valve: channel has to be integer (1-8)')
            return -1
                
        self.state[channel-1] = state
        valve_state=(c_int32*16)(0)
        for i in range(8):
            valve_state[i]=c_int32(self.state[i]) 
            
        error=MUX_Wire_Set_all_valves(self.Instr_ID.value, valve_state, 16)
        
    def get_state(self):
        """
        Returns state of current valves: list of 8 integers
        """
        return self.state
            
    def __del__(self):
        error=MUX_Destructor(self.Instr_ID.value)
        
class msr():
    def __init__(self, address='01EE3A2D'):
        self.Instr_ID = c_int32()
        # see User guide to determine regulator type NI MAX to determine theerror=M_S_R_D_Initialization('01DAA568'.encode('ascii'),5,0,0,0,0,0,byref(Instr_ID)) 
        
        error = M_S_R_D_Initialization(address.encode('ascii'),
                                       Sens_Ch_1=Z_sensor_type['Flow_5000_uL_min'],
                                       Sens_Ch_2=Z_sensor_type['none'],
                                       Sens_Ch_3=Z_sensor_type['Flow_1000_uL_min'],
                                       Sens_Ch_4=Z_sensor_type['none'],
                                       CustomSens_Voltage_Ch12=5,  # 5-25V unused for digital sensors
                                       CustomSens_Voltage_Ch34=5,  # 5-25V unused for digital sensors
                                       MSRD_ID_out=byref(self.Instr_ID))
        # all functions will return error code to help you to debug your code, for further information see user guide
        _check_error('MSR Initialization', error)
        print(error)
        
        # Add sensors
        #   on channel 1
        error =  M_S_R_D_Add_Sens(self.Instr_ID.value, 
                                  Channel_1_to_4         = c_int32(1),
                                  SensorType             = Z_sensor_type['Flow_5000_uL_min'],
                                  DigitalAnalog          = c_uint16(1), # Digital
                                  FSens_Digit_Calib      = c_uint16(0), # water calibration
                                  FSens_Digit_Resolution = c_uint16(0)) # 9 bit
        if error != 0:
            print('error add digital flow sensor on channel 1:%d' % error)
        
        #   on channel 2
        error =  M_S_R_D_Add_Sens(self.Instr_ID.value, 
                                  Channel_1_to_4         = c_int32(2),
                                  SensorType             = Z_sensor_type['Flow_5000_uL_min'],
                                  DigitalAnalog          = c_uint16(1), # Digital
                                  FSens_Digit_Calib      = c_uint16(0), # water calibration
                                  FSens_Digit_Resolution = c_uint16(0)) # 9 bit
        if error != 0:
            print('error add digital flow sensor on channel 2:%d' % error)
            
#         #   on channel 3
#         error =  M_S_R_D_Add_Sens(self.Instr_ID.value, 
#                                   Channel_1_to_4         = c_int32(3),
#                                   SensorType             = Z_sensor_type['Flow_1000_uL_min'],
#                                   DigitalAnalog          = c_uint16(1), # Digital
#                                   FSens_Digit_Calib      = c_uint16(0), # water calibration
#                                   FSens_Digit_Resolution = c_uint16(0)) # 9 bit
        if error != 0:
            print('error add digital flow sensor on channel 3:%d' % error)
        
        
    def meas_flowrate(self, channel=1):
        """
        Measures the instantaneous flow rate, on designated port.
        Parameters:
            channel: MSR channel to set (1-4), defaults to 1
        return: (flowrate, error code)
            flowrate in uL/min
            0 : successful
            -1 : parameter error
            otherwise: NI error
        """
        if channel <1 or channel > 4:
            print('msr.meas_flowrate: channel must be within 1~4')
            return -1
        
        set_channel = c_int32(channel)  # convert to c_int32
        get_flowrate = c_double(0.0)
        error = M_S_R_D_Get_Sens_Data(Channel_1_to_4=set_channel,
                                      M_S_R_D_ID=self.Instr_ID.value, 
                                      Sens_Data=byref(get_flowrate))
        
        return get_flowrate.value, error
    
    def __del__(self):
        error=M_S_R_D_Destructor(self.Instr_ID.value)
        
class acton_trigger():
    def __init__(self, address='COM3', timeout=30.0):
        self.uno = serial.Serial()
        self.uno.port = address
#         self.uno.timeout = 20
        # trigger.baudrate = 9600
        self.uno.baudrate = 115200
        self.uno.timeout=timeout
        self.uno.open()

        self.timeout = timeout
        self.dt = 0.005
        self.N_max=20
        
    def __del__(self):
        self.uno.close()
    
    def get_state(self):
        self.uno.write(b'S')
        state = self.uno.read()
        print('Acton state at {}'.format(state))
        
        return state
    
    def send_single(self):
        self.uno.write(b'T')
        state = self.uno.read()
        
        return state
    
    def send_trigger(self, N=10):
        t=0
        t_start=time()
        dt = self.dt

        for i in range(0, N):
            self.uno.write(b'S')
            state = self.uno.read()
            print('Acton state at {}'.format(state))

            if state == b'1':
                self.uno.write(b'T')
                t_0=time()
                state = self.uno.read()
        #         print(state)
                t=0
                while state ==b'2' and t-t_0 < self.timeout:
                    self.uno.write(b'S')
                    state = self.uno.read()
        #             print(state)

                    t=time()-t_0
                    if state == b'1' or state == b'0' :
                        print('{:.4g} Trigger acquired'.format(i+1))
                        break
                    sleep(dt)
            elif state==b'0':
                if i==0:
                    print('Acton state at 0, please check WinSpec')
                else:
                    print('Acquisition is done, state at 0')
                break

        t_end=time()

        if t>= self.timeout:
            print('Timed out')

        print('Experiment took {:.4g} secs'.format(t_end-t_start))

class sampler():
    def __init__(self, verbose=False):
        """
        Initialization
        """
        try:
            self.ob1 = ob1(calibrate=False)
            self.msr = msr()
            self.mux_wire = mux_wire()
        except:
            print('Error opening elveflow instruments')
        
        try:
            self.acton = acton_trigger(address='COM3', timeout=20.0)
        except:
            print('Error opening connection to Acton trigger board')
        
        
        try: 
            self.pm = TLPM()
            deviceCount = c_uint32()
            self.pm.findRsrc(byref(deviceCount))

            print("devices found: " + str(deviceCount.value))

            resourceName = create_string_buffer(1024)

            for i in range(0, deviceCount.value):
                tlPM.getRsrcName(c_int(i), resourceName)
                print(c_char_p(resourceName.raw).value)
                break

            self.pm.open(resourceName, c_bool(True), c_bool(True))

            message = create_string_buffer(1024)
            self.pm.getCalibrationMsg(message)
            print(c_char_p(message.raw).value)
        except:
            print('Could not open Thorlabs Power meter : {}'.format(sys.exc_info()))
        else:
            print('Thorlabs Powermeter connected')    
        
        self.verbose = verbose
        
    def set_valves(self, reservoir, target):
        """
        Sets valve configuration of sampling system
        """
        state = self.mux_wire.get_state()
        if reservoir=='air':
#             print('sampler.set_valves: Set air reservoir')
            state[1] = 0
        elif reservoir == 'water':
            state[1] = 1
            
        if target=='flow-cell':
            state[0] = 1
        elif target == 'tflask1':
            state[0] = 0
            state[2] = 0
            state[3] = 0
        elif target == 'tflask2':
            state[0] = 0
            state[2] = 0
            state[3] = 1
        elif target == 'tflask3':
            state[0] = 0
            state[2] = 1
            state[4] = 0
        elif target == 'tflask4':
            state[0] = 0
            state[2] = 1
            state[4] = 1
            
        error = self.mux_wire.set_all_valves(state)
        
        return error
    
    def sample(self, sensor=('msr', 1), setpoint=-100, trigger=0.1, overtime=10, timeout=10, press_range=(-900,1000)):
        """
        Samples fluid plug using OB1 and given flow sensor until flowrate error hits trigger threshold.
        This assumes we are fixing volume with tubing between valves, and there is air in sensor before sample plug.    
        Parameters:
            sensor: tuple (string "msr" or "ob1", integer of channel)
            setpoint: float (target flow rate in uL/min)
            trigger: float (error threshold to stop sampling relative to flowrate)
            timeout: float (timeout in seconds)
        return: timestamp
        """        
        Kp = 0.25
        Ki = 0.2
        Kd = 0.001

        start_time = time()
        cur_time = time()-start_time
        flag_time = -1.0
        err_i = 0.0
        err_d = 0.0
        err_prev = 0.0
        dt = 0.1 # sec

        # traces
        t_arr = []
        x_arr = []
        y_arr = []
        
        # setup sensor
        ch = sensor[1]
        if sensor[0]=='msr':
            device = self.msr
            if ch < 1 or ch > 4:
                print('wrong sensor channel for msr, defaulting to 1')
                ch = 1
        elif sensor[0]=='ob1':
            device = self.ob1
            ch = 1
        
#         self.set_valves('air', 'tflask')
        drop_test = 1
        drop_test_target = 5 # need at least drop_test_target consecutive above trigger to determine sample at flow cell 
        while cur_time < timeout:
            y,flag = device.meas_flowrate(channel=ch)
            if flag == 0:
                err = setpoint - y
                err_i = err_i + err*dt
                err_d = (err - err_prev)/dt

                x = Kp*err + Ki*err_i + Kd*err_d

                # Clip pressure to pressure range
                if x > press_range[1]:
                    if self.verbose:
                        print('Pressure over range {}'.format(x))
                    x = press_range[1]
                elif x < press_range[0]:
                    if self.verbose:
                        print('Pressure under range {}'.format(x))
                    x = press_range[0]
                        
                self.ob1.set_pressure(x)

                x_arr.append(x)
                y_arr.append(y)
                t_arr.append(cur_time)


                err_prev = err
                cur_time = time()-start_time
                if self.verbose:
                    print('Time {:10.3g}: err={:10.3g} err_i={:10.3g} y={:10.3g} x={:10.3g}'.format(cur_time, err, err_i, y, x))
                
                # increment drop_test, to make sure it is not a rogue water drop
                if (setpoint < 0.0 and y < setpoint*(1.0+trigger)) or (setpoint > 0.0 and y > setpoint*(1.0+trigger)):
                    drop_test = drop_test + 1
                else:
                    drop_test = 1
                    
                if flag_time < 0.0:
                    if drop_test > drop_test_target:
                        flag_time = cur_time
                        if self.verbose:
                            print('Sample at flow sensor. flag_time {:.5g} sec'.format(flag_time))
                        if overtime < dt: # overtime==0 case
                            if self.verbose:
                                print('Over sampling done')
                            break                        
                elif (cur_time - flag_time) > overtime:
                    if self.verbose:
                        print('Over sampling done')
                    break
            else:
                if self.verbose:
                    print('Flow rate sensor error {}, value {}'.format(flag, y))

            sleep(dt)
            
        if self.verbose:
            print('Zeroing pressure')
        self.ob1.set_pressure(0)
        sleep(3)
        if cur_time >= timeout: 
            print('Timed out during sampling.\n')
            raise ValueError('Timed out during sampling to {}'.format(sensor))
        else:
            print('Sampling Done.\n')
        
    
    def pushpull(self, pressure=100, timeout=30, sensor=('msr', 1), turnoff=True):
        """
        Push or pulls a certain volume or for some time using OB1 and connected flow sensor        
        Parameters:            
            pressure: float (set pressure in mbar)
            timeout: float (timeout in seconds)
            sensor: tuple (string "msr" or "ob1", integer of channel)
        return: 
        """
        if self.verbose:
            print('pushpull: {:.3g} mbar for {:.3g} secs'.format(pressure, timeout))
        # setup sensor
        ch = sensor[1]
        if sensor[0]=='msr':
            device = self.msr
            if ch < 1 or ch > 4:
                print('wrong sensor channel {} for msr, defaulting to 1'.format(ch))
                ch = 1
        elif sensor[0]=='ob1':
            device = self.ob1
            ch = 1
            
        time_step = 1
        self.ob1.set_pressure(pressure)
        start_time = time()
        cur_time = time()-start_time   
        while cur_time < timeout:                
            flowrate,error = device.meas_flowrate(channel=ch)
            if self.verbose:
                print('pushpull Time {:10.3g}: current flow rate {:10.3g}'.format(cur_time, flowrate))
            cur_time = time()-start_time

            sleep(time_step)
        
        if turnoff:
            if self.verbose:
                print('Zeroing pressure')
            self.ob1.set_pressure(0)
            sleep(3)
            
        print('Push pull: done\n')
        
    def mix(self, source, sink, Nplugs=1):
        """
        Sends designated number of plugs of fluid from source to sink reservoirs and bubbles for mix       
        Parameters:            
            source: string (reservoir to pull fluid plugs from)
            sink: string (reservoir to push fluid plugs to)
            Nplugs: integer (number of plugs)            
        return: 
        """
        if source not in ['tflask1', 'tflask2']:
            print('Mixing source must be from either tflask1 or 2')
            return
        if sink not in ['tflask3', 'tflask4']:
            print('Mixing sink must be to either tflask3 or 4')
            return
        
        for i in range(Nplugs):
            self.set_valves('water', source)
            self.sample(sensor=("msr", 1), setpoint=-500, trigger=0.1, overtime=8.0, timeout=40)

            # Push sample to sink and bubble mix
            self.set_valves('air', sink)
            self.pushpull(pressure=400, timeout=20, sensor=('msr', 1))
            
            print('{} out of {} plugs done'.format(i+1, Nplugs))
    
    def to_flowcell(self, source):
        """
        Sends designated number of plugs of fluid from source to sink reservoirs and bubbles for mix       
        Parameters:            
            source: string (reservoir to pull fluid plugs from)
        return: 
        """
        max_press = 60.0
        max_press_clean=100.0
        max_pull = 70.0
        
        print('Sending sample from {} to flowcell'.format(source))
        if source=='water':
            # push cleaning buffer (ethanol)
            self.set_valves('water', 'flow-cell')
            self.sample(sensor=("msr", 1), setpoint=500, press_range=(0,max_press), trigger=0.0, overtime=15.0, timeout=30)

            # send plug to flow-cell
            self.set_valves('air', 'flow-cell')
            self.sample(sensor=("msr", 2), setpoint=400, press_range=(-max_press,max_press), trigger=0.0, overtime=16.0, timeout=40)
        elif source=='tflask4': # Wash buffer
            # pull back on water line
            self.clear(target='flow-cell', timeout=5, pressure=max_press_clean)            
            self.set_valves('water', 'flow-cell')
            self.pushpull(pressure=-max_press, timeout=3)
            
            # pull water plug
            self.set_valves('water', source)
            self.sample(sensor=("msr", 1), setpoint=-1000, press_range=(-max_pull,max_pull/2.0), trigger=0.1, overtime=5.0, timeout=20)
            
            # push full plug to flow-cell
            self.set_valves('water', 'flow-cell')
            self.sample(sensor=("msr", 2), setpoint=1000, press_range=(-max_press_clean/2.0,max_press_clean), trigger=0.0, overtime=10.0, timeout=30)
            
            # push back fluid into reservoirs
            self.set_valves('air', source)
            self.pushpull(pressure=150, timeout=10)
            
        elif source in ['tflask1', 'tflask2', 'tflask3']:
            # pull back on water line
            self.clear(target='flow-cell', timeout=5, pressure=max_press_clean)            
            self.set_valves('water', 'flow-cell')
            self.pushpull(pressure=-max_press, timeout=3)
            
            # pull sample
            self.set_valves('water', source)
            self.sample(sensor=("msr", 1), setpoint=-100, press_range=(-max_pull,max_pull/2.0), trigger=0.0, overtime=0.5, timeout=20)
            
            # send plug to flow-cell
            self.set_valves('air', 'flow-cell')
            self.sample(sensor=("msr", 2), setpoint=900, press_range=(-max_press/2.0,max_press), trigger=0.0, overtime=6.0, timeout=30)

            # push back fluid into reservoirs
            self.set_valves('air', source)
            self.pushpull(pressure=150, timeout=10)
        else:
            print('{} source is not supported'.format(source))
            
        self.set_valves('air', 'tflask4')
        print('Sending Sample from {} at flowcell done\n'.format(source))
            
    def clear(self, target, timeout=10, pressure=100):
        """
        Sends air through lines to target to remove any liquids       
        Parameters:            
            target: string (reservoir to clear path to)  
        return: 
        """
        if target == 'all':
            targets = ['tflask2', 'tflask3', 'tflask4', 'flow-cell', 'tflask1']
        elif target == 'allflasks':            
            targets = ['tflask2', 'tflask3', 'tflask4', 'tflask1']
        else:
            targets = [target]
            
        for reservoir in targets:
            if self.verbose:
                print('Clearing lines to {}'.format(reservoir))
                
            self.set_valves('air', reservoir)
            if reservoir == 'flow-cell':
                self.pushpull(pressure=pressure, timeout=timeout, sensor=('msr', 2))
            else:
                self.pushpull(pressure=pressure, timeout=timeout, sensor=('msr', 1))
                
    def clean(self, target):
        """
        Sends water then air through lines to target to remove any liquids       
        Parameters:            
            target: string (reservoir to clear path to)  
        return: 
        """
        if target == 'all':
            targets = ['tflask1', 'tflask2', 'tflask3', 'tflask4', 'flow-cell']
        elif target == 'allflasks':            
            targets = ['tflask1', 'tflask2', 'tflask3', 'tflask4']
        else:
            targets = [target]
            
        for reservoir in targets:
#             if self.verbose:
            print('Cleaning lines to {}'.format(reservoir))
                
            self.set_valves('water', reservoir)
            self.pushpull(pressure=90, timeout=10, sensor=('msr', 1))
            
            self.set_valves('air', reservoir)
            if reservoir == 'flow-cell':
                self.pushpull(pressure=80, timeout=30, sensor=('msr', 2))
                self.pushpull(pressure=120, timeout=30, sensor=('msr', 2))
                self.pushpull(pressure=50, timeout=30, sensor=('msr', 2))
            else:
                self.pushpull(pressure=250, timeout=20)
        print('Cleaning done\n')
                
    def clean_cell(self, cycles=1):     
        """
        Sends water then air through lines to target to remove any liquids       
        Parameters:            
            target: string (reservoir to clear path to)  
        return: 
        """
        
        print('Cleaning flow cell')
        for i in range(cycles):
            # get water plug
            self.to_flowcell(source='tflask4')
#             sleep(20)
            # send water plug out 
#             self.clear(target='flow-cell', timeout=15, pressure=70)
            # blow dry
            self.clear(target='flow-cell', timeout=10, pressure=70)
        
#         # Pull water line
#         self.set_valves('water', 'flow-cell')
#         self.pushpull(pressure=-40, timeout=5)
        
        print('Cleaning done\n')
            
    def take_measurement(self, samples, N=10, Nclean=2):
        """
        Runs consecutive measurements
        Parameters:            
            sources: list (reservoirs to sample from) 
            N: integer (number of spectra per sample)
            Nclean: integer (number of cleaning cycles between sample)
        return: 
        """
        t_start = time()
        t_acquire = 0.0
#         print('\nclean cell')
#         self.clean_cell(cycles=Nclean)
        for sample in samples:
            print('\nsend sample from {} to cell'.format(sample))
            self.to_flowcell(source=sample)
            
            print('\nacquire spectra')
            t_acquire_start = time()
            self.acton.send_trigger(N=N)
            t_acquire = t_acquire+time()-t_acquire_start
            
            print('\nclear cell')
            self.clear(target='flow-cell', timeout=5, pressure=70)
            
            print('\nclean cell')
            self.clean_cell(cycles=Nclean)
        
        self.clear('allflasks', timeout=10, pressure=150)
        
        t_end = time()
        print('Measurement Done in total: {}, acquiring: {}'.format(datetime.timedelta(seconds=t_end-t_start), datetime.timedelta(seconds=t_acquire)))
        
    def __del__(self):        
        self.set_valves('air', 'flow-cell')
        self.ob1.set_pressure(0)
        
        self.acton.uno.close()
        
        self.pm.close()
        
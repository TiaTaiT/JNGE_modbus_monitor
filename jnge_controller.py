"""
JNGE Wind and Solar Hybrid Controller RS485 Modbus Interface
Core library for communication with JNGE MPPT controllers
"""

import serial
import struct
import time
from typing import Optional, Dict, List
from enum import IntEnum


class BatteryType(IntEnum):
    LEAD_ACID = 1
    LIFEPO4 = 2
    TERNARY_LITHIUM = 3
    CUSTOMIZE = 4


class ChargeState(IntEnum):
    UNCHARGED = 0
    CONSTANT_CURRENT = 1
    BOOST_CHARGING = 2
    FLOAT_CHARGING = 3


class LoadStatus(IntEnum):
    SHUT_DOWN = 0
    BOOT_UP = 1


class OperatingMode(IntEnum):
    HOUSEHOLD = 0
    STREET_LAMP = 1


class JNGEController:
    """Interface for JNGE Wind and Solar Hybrid Controller via RS485 Modbus"""
    
    # Register addresses
    REG_BATTERY_VOLTAGE = 0x1000
    REG_PV_VOLTAGE = 0x1001
    REG_FAN_VOLTAGE = 0x1002
    REG_PV_CURRENT = 0x1003
    REG_FAN_CURRENT = 0x1004
    REG_PV_POWER = 0x1005
    REG_FAN_POWER = 0x1006
    REG_PV_TOTAL_ENERGY = 0x1007
    REG_FAN_TOTAL_ENERGY = 0x1008
    REG_CHARGE_STATE = 0x1009
    REG_LOAD_STATUS = 0x100A
    REG_VERSION = 0x100B
    REG_PV_RATING = 0x100C
    REG_FAN_RATING = 0x100D
    REG_BATTERY_STRINGS = 0x100E
    REG_BATTERY_TYPE = 0x100F
    REG_BATTERY_VOLTAGE_LEVEL = 0x1010
    REG_ERROR_CODE = 0x1011
    REG_PV_SWITCH = 0x1012
    REG_LOAD_SWITCH = 0x1013
    
    # Setting parameters
    REG_OVERVOLTAGE = 0x1024
    REG_OVERVOLTAGE_RECOVERY = 0x1025
    REG_BOOST_VOLTAGE = 0x1026
    REG_BOOST_RETURN_VOLTAGE = 0x1027
    REG_FLOAT_VOLTAGE = 0x1028
    REG_FLOAT_RETURN_VOLTAGE = 0x1029
    REG_UNDERVOLTAGE = 0x102A
    REG_UNDERVOLTAGE_RECOVERY = 0x102B
    REG_MODBUS_ADDRESS = 0x1030
    REG_LIGHT_CTRL_ON_VOLTAGE = 0x1031
    REG_LIGHT_CTRL_OFF_VOLTAGE = 0x1032
    REG_OPERATING_MODE = 0x1033
    
    # Fault code definitions
    FAULT_CODES = {
        0: "PV charging overcurrent",
        1: "Short circuit fault",
        3: "PV charging battery overvoltage",
        4: "PV array overvoltage (reverse)",
        5: "Fan input overvoltage",
        6: "Fan charging overcurrent",
        12: "Battery undervoltage",
        14: "PV array undervoltage (under 6V)"
    }
    
    def __init__(self, port: str, address: int = 0x06, baudrate: int = 9600, 
                 timeout: float = 1.0, inter_byte_timeout: float = 0.1):
        """
        Initialize JNGE Controller interface
        
        Args:
            port: Serial port name (e.g., 'COM3' or '/dev/ttyUSB0')
            address: Modbus device address (default: 0x06)
            baudrate: Communication baudrate (default: 9600)
            timeout: Serial timeout in seconds
            inter_byte_timeout: Timeout between bytes
        """
        self.port = port
        self.address = address
        self.baudrate = baudrate
        self.timeout = timeout
        self.inter_byte_timeout = inter_byte_timeout
        self.serial = None
        self.debug = False
        
    def connect(self):
        """Open serial connection"""
        try:
            self.serial = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=self.timeout,
                inter_byte_timeout=self.inter_byte_timeout
            )
            time.sleep(0.2)
            self.serial.reset_input_buffer()
            self.serial.reset_output_buffer()
            return True
        except serial.SerialException as e:
            print(f"Failed to open serial port: {e}")
            return False
    
    def disconnect(self):
        """Close serial connection"""
        if self.serial and self.serial.is_open:
            self.serial.close()
    
    def _calculate_crc16(self, data: bytes) -> int:
        """Calculate CRC16 for Modbus"""
        crc = 0xFFFF
        for byte in data:
            crc ^= byte
            for _ in range(8):
                if crc & 0x0001:
                    crc = (crc >> 1) ^ 0xA001
                else:
                    crc >>= 1
        return crc
    
    def _build_request(self, function_code: int, start_reg: int, 
                      num_regs: int) -> bytes:
        """Build Modbus request frame"""
        frame = struct.pack('>BBHH', self.address, function_code, 
                           start_reg, num_regs)
        crc = self._calculate_crc16(frame)
        frame += struct.pack('<H', crc)
        return frame
    
    def _build_write_request(self, function_code: int, reg_addr: int, 
                            value: int) -> bytes:
        """Build Modbus write request frame"""
        frame = struct.pack('>BBHH', self.address, function_code, 
                           reg_addr, value)
        crc = self._calculate_crc16(frame)
        frame += struct.pack('<H', crc)
        return frame
    
    def _send_request(self, request: bytes) -> Optional[bytes]:
        """Send request and receive response"""
        if not self.serial or not self.serial.is_open:
            print("Serial port not open")
            return None
        
        try:
            self.serial.reset_input_buffer()
            self.serial.reset_output_buffer()
            time.sleep(0.01)
            
            self.serial.write(request)
            if self.debug:
                print(f"TX: {request.hex(' ')}")
            
            time.sleep(0.1)
            
            retries = 0
            max_retries = 10
            while self.serial.in_waiting < 3 and retries < max_retries:
                time.sleep(0.02)
                retries += 1
            
            if self.serial.in_waiting < 3:
                print("No response from device")
                return None
            
            response = self.serial.read(3)
            if len(response) < 3:
                print(f"Incomplete response header: got {len(response)} bytes")
                return None
            
            addr, func, byte_count = struct.unpack('BBB', response)
            
            if self.debug:
                print(f"RX Header: addr={addr:02X} func={func:02X} count={byte_count}")
            
            remaining = byte_count + 2
            data_received = b''
            bytes_to_read = remaining
            timeout_counter = 0
            max_timeout = 50
            
            while len(data_received) < bytes_to_read and timeout_counter < max_timeout:
                if self.serial.in_waiting > 0:
                    chunk = self.serial.read(min(self.serial.in_waiting, bytes_to_read - len(data_received)))
                    data_received += chunk
                else:
                    time.sleep(0.02)
                    timeout_counter += 1
            
            response += data_received
            
            if len(data_received) < remaining:
                print(f"Incomplete response: expected {remaining} bytes, got {len(data_received)}")
                if self.debug:
                    print(f"RX (partial): {response.hex(' ')}")
                return None
            
            if self.debug:
                print(f"RX: {response.hex(' ')}")
            
            crc_received = struct.unpack('<H', response[-2:])[0]
            crc_calculated = self._calculate_crc16(response[:-2])
            
            if crc_received != crc_calculated:
                print(f"CRC mismatch: received={crc_received:04X}, calculated={crc_calculated:04X}")
                return None
            
            return response
            
        except serial.SerialException as e:
            print(f"Serial communication error: {e}")
            return None
        except Exception as e:
            print(f"Unexpected error: {e}")
            return None
    
    def discover_device(self) -> Optional[int]:
        """Broadcast to discover device address (FF function code)"""
        broadcast_frame = struct.pack('>BBHH', 0xFF, 0x03, 0x1030, 0x0001)
        crc = self._calculate_crc16(broadcast_frame)
        broadcast_frame += struct.pack('<H', crc)
        
        if not self.serial or not self.serial.is_open:
            print("Serial port not open")
            return None
        
        try:
            self.serial.reset_input_buffer()
            self.serial.write(broadcast_frame)
            time.sleep(0.1)
            
            response = self.serial.read(7)
            if len(response) >= 7:
                addr = response[0]
                print(f"Device found at address: 0x{addr:02X}")
                return addr
        except Exception as e:
            print(f"Discovery error: {e}")
        
        return None
    
    def read_registers(self, start_reg: int, num_regs: int) -> Optional[List[int]]:
        """Read holding registers (function code 0x03)"""
        request = self._build_request(0x03, start_reg, num_regs)
        time.sleep(0.05)
        response = self._send_request(request)
        
        if not response:
            return None
        
        byte_count = response[2]
        data = response[3:3+byte_count]
        
        values = []
        for i in range(0, len(data), 2):
            if i+1 < len(data):
                value = struct.unpack('>H', data[i:i+2])[0]
                values.append(value)
        
        return values
    
    def write_register(self, reg_addr: int, value: int) -> bool:
        """Write single register (function code 0x06)"""
        request = self._build_write_request(0x06, reg_addr, value)
        time.sleep(0.05)
        response = self._send_request(request)
        
        if not response:
            return False
        
        return response[:-2] == request[:-2]
    
    def set_debug(self, debug: bool):
        """Enable/disable debug output"""
        self.debug = debug
    
    def read_operating_parameters(self) -> Optional[Dict]:
        """Read current operating parameters (20 registers from 0x1000)"""
        values = self.read_registers(0x1000, 0x14)
        
        if not values:
            return None
        
        params = {
            'battery_voltage': values[0] * 0.1,
            'pv_voltage': values[1] * 0.1,
            'fan_voltage': values[2] * 0.1,
            'pv_current': values[3] * 0.1,
            'fan_current': values[4] * 0.1,
            'pv_power': values[5],
            'fan_power': values[6],
            'pv_total_energy': values[7] * 0.1,
            'fan_total_energy': values[8] * 0.1,
            'charge_state': ChargeState(values[9]),
            'load_status': LoadStatus(values[10]),
            'version': values[11],
            'pv_rating': values[12] * 100,
            'fan_rating': values[13] * 100,
            'battery_strings': values[14],
            'battery_type': BatteryType(values[15]),
            'battery_voltage_level': values[16] * 0.1,
            'error_code': values[17],
            'pv_switch': values[18],
            'load_switch': values[19]
        }
        
        return params
    
    def read_system_settings(self) -> Optional[Dict]:
        """Read system setup parameters (25 registers from 0x1024)"""
        values = self.read_registers(0x1024, 0x19)
        
        if not values:
            return None
        
        settings = {
            'overvoltage': values[0] * 0.1,
            'overvoltage_recovery': values[1] * 0.1,
            'boost_voltage': values[2] * 0.1,
            'boost_return_voltage': values[3] * 0.1,
            'float_voltage': values[4] * 0.1,
            'float_return_voltage': values[5] * 0.1,
            'undervoltage': values[6] * 0.1,
            'undervoltage_recovery': values[7] * 0.1,
            'boost_charging_time': values[8],
            'battery_voltage_level': values[9] * 0.1,
            'battery_type': BatteryType(values[10]),
            'battery_strings': values[11],
            'modbus_address': values[12],
            'light_ctrl_on_voltage': values[13] * 0.1,
            'light_ctrl_off_voltage': values[14] * 0.1,
            'operating_mode': OperatingMode(values[15]),
            'light_period_1_hours': values[16],
            'light_period_1_intensity': values[17] * 10,
            'light_period_2_hours': values[18],
            'light_period_2_intensity': values[19] * 10,
            'light_period_3_hours': values[20],
            'light_period_3_intensity': values[21] * 10,
            'fan_unloading_voltage': values[22] * 0.1,
            'charging_switch': values[23],
            'load_switch': values[24]
        }
        
        return settings
    
    def read_device_info(self) -> Optional[Dict]:
        """Read device type and number (2 registers from 0x1060)"""
        values = self.read_registers(0x1060, 0x02)
        
        if not values:
            return None
        
        return {
            'device_type': values[0],
            'device_number': values[1]
        }
    
    def decode_error_code(self, error_code: int) -> List[str]:
        """Decode error code bitmap into list of active faults"""
        faults = []
        for bit, description in self.FAULT_CODES.items():
            if error_code & (1 << bit):
                faults.append(description)
        return faults if faults else ["No faults"]
    
    def set_battery_type(self, battery_type: BatteryType) -> bool:
        """Set battery type"""
        return self.write_register(0x102E, battery_type.value)
    
    def set_modbus_address(self, new_address: int) -> bool:
        """Set device Modbus address (1-255)"""
        if not 1 <= new_address <= 255:
            print("Address must be between 1 and 255")
            return False
        return self.write_register(0x1030, new_address)
    
    def set_overvoltage(self, voltage: float) -> bool:
        """Set overvoltage protection (V)"""
        value = int(voltage * 10)
        return self.write_register(0x1024, value)
    
    def set_undervoltage(self, voltage: float) -> bool:
        """Set undervoltage protection (V)"""
        value = int(voltage * 10)
        return self.write_register(0x102A, value)
    
    def enable_pv_charging(self, enable: bool) -> bool:
        """Enable/disable PV charging"""
        return self.write_register(0x103B, 1 if enable else 0)
    
    def enable_load(self, enable: bool) -> bool:
        """Enable/disable load output"""
        return self.write_register(0x103C, 1 if enable else 0)
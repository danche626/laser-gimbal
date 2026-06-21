"""
motor.py - YL42M步进电机 Modbus RTU 控制
"""
import serial
import struct
import glob


def find_serial_port():
    candidates = glob.glob("/dev/ttyUSB*") + glob.glob("/dev/ttyACM*")
    for c in sorted(candidates):
        try:
            s = serial.Serial(c, 115200, timeout=0.1)
            s.close()
            return c
        except (serial.SerialException, OSError):
            continue
    return None


def _calc_crc(data):
    crc = 0xFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 0x0001:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
    return crc & 0xFFFF


class MotorController:
    def __init__(self, cfg):
        port = find_serial_port()
        if port is None:
            raise RuntimeError("No serial port found for motors")
        self.ser = serial.Serial(port, cfg.MOTOR_BAUDRATE, timeout=0.01,
                                 bytesize=8, stopbits=1, parity="N")
        self.ser.reset_input_buffer()
        self._last_spd = {cfg.MOTOR_ADDR_X: 0, cfg.MOTOR_ADDR_Y: 0}
        self._threshold = cfg.MOTOR_SPEED_THRESHOLD
        self._accel = cfg.MOTOR_ACCEL
        print(f"[MOTOR] Serial: {port}")

    def set_velocity(self, addr, speed):
        speed = int(speed)
        if abs(speed - self._last_spd[addr]) < self._threshold:
            return
        self._last_spd[addr] = speed
        if speed == 0:
            self._stop(addr)
            return
        val = speed & 0xFFFF
        payload = bytes([addr, 0x10, 0x00, 0x60, 0x00, 0x04, 0x08])
        payload += struct.pack(">HHHH", 0x0000, val, self._accel, 100)
        crc = _calc_crc(payload)
        self.ser.write(payload + struct.pack("<H", crc))
        self.ser.reset_input_buffer()

    def _stop(self, addr):
        payload = bytes([addr, 0x10, 0x00, 0x01, 0x00, 0x02, 0x04])
        payload += struct.pack(">HH", 0x0000, 0x0000)
        crc = _calc_crc(payload)
        self.ser.write(payload + struct.pack("<H", crc))
        self.ser.reset_input_buffer()

    def stop_all(self):
        for addr in list(self._last_spd.keys()):
            self._stop(addr)
        self._last_spd = {k: 0 for k in self._last_spd}

    def get_last_speed(self, addr):
        return self._last_spd.get(addr, 0)

    def close(self):
        self.stop_all()
        self.ser.close()

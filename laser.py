"""
laser.py - 激光控制模块
"""
try:
    import Jetson.GPIO as GPIO
    GPIO_AVAILABLE = True
except ImportError:
    GPIO_AVAILABLE = False


class LaserController:
    def __init__(self, cfg):
        self.pin = cfg.PIN_LASER
        self.on = False
        if GPIO_AVAILABLE:
            GPIO.setup(self.pin, GPIO.OUT, initial=GPIO.LOW)

    def fire(self):
        if self.on:
            return
        self.on = True
        if GPIO_AVAILABLE:
            GPIO.output(self.pin, GPIO.HIGH)

    def off(self):
        if not self.on:
            return
        self.on = False
        if GPIO_AVAILABLE:
            GPIO.output(self.pin, GPIO.LOW)

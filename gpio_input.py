"""
gpio_input.py - 按键输入处理（防抖 + 边沿检测）
"""
import time

try:
    import Jetson.GPIO as GPIO
    GPIO_AVAILABLE = True
except ImportError:
    GPIO_AVAILABLE = False


class GPIOInput:
    def __init__(self, cfg):
        self.pin_key1 = cfg.PIN_KEY1
        self.pin_key2 = cfg.PIN_KEY2
        self.debounce_s = cfg.DEBOUNCE_MS / 1000.0
        self.boot_ignore_s = cfg.BOOT_IGNORE_MS / 1000.0
        self._boot_time = time.time()

        if GPIO_AVAILABLE:
            GPIO.setup(self.pin_key1, GPIO.IN, pull_up_down=GPIO.PUD_UP)
            GPIO.setup(self.pin_key2, GPIO.IN, pull_up_down=GPIO.PUD_UP)
            self._key1_last = GPIO.input(self.pin_key1)
            self._key2_last = GPIO.input(self.pin_key2)
        else:
            self._key1_last = 1
            self._key2_last = 1

        self.key1_pressed = False
        self.key2_pressed = False

    def update(self):
        """每帧调用，检测按键下降沿（按下=LOW）"""
        self.key1_pressed = False
        self.key2_pressed = False

        if not GPIO_AVAILABLE:
            return
        if (time.time() - self._boot_time) < self.boot_ignore_s:
            return

        key1_now = GPIO.input(self.pin_key1)
        key2_now = GPIO.input(self.pin_key2)

        # 下降沿检测: 1→0 表示按下
        if self._key1_last == 1 and key1_now == 0:
            time.sleep(self.debounce_s)
            if GPIO.input(self.pin_key1) == 0:
                self.key1_pressed = True
        self._key1_last = key1_now

        if self._key2_last == 1 and key2_now == 0:
            time.sleep(self.debounce_s)
            if GPIO.input(self.pin_key2) == 0:
                self.key2_pressed = True
        self._key2_last = key2_now

import RPi.GPIO as GPIO
import time

# 设置 GPIO 模式为 BCM
GPIO.setmode(GPIO.BCM)

# 设置 GPIO 引脚为输出模式
led_pin = 17  # 以 GPIO18 为例
GPIO.setup(led_pin, GPIO.OUT)

# 循环闪烁 LED
try:
    while True:
        #GPIO.output(led_pin, GPIO.HIGH)  # LED ON
        #time.sleep(1)                    # 持续 1 秒
        GPIO.output(led_pin, GPIO.LOW)   # LED OFF
        #time.sleep(1)                    # 持续 1 秒
except KeyboardInterrupt:
    GPIO.cleanup()  # 清理配置并退出


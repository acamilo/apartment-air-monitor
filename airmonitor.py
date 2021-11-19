import time
import supervisor
import ssl
import socketpool
import wifi
import adafruit_minimqtt.adafruit_minimqtt as MQTT
import board
import neopixel
import busio
from digitalio import DigitalInOut, Direction, Pull
from adafruit_pm25.i2c import PM25_I2C
from adafruit_led_animation.animation.blink import Blink
from adafruit_led_animation.animation.comet import Comet
from adafruit_led_animation.animation.chase import Chase
from adafruit_led_animation.color import PURPLE, AMBER, JADE, RED, GREEN
from adafruit_led_animation.sequence import AnimationSequence

import adafruit_scd4x


class AirMonitor:
    secrets = None
    def __init__(self,secrets,name="ESP32"):
        pass

    def init_particulate_sensor(self):
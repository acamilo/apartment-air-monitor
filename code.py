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
    name = None
    i2c = None
    pm25 = None
    connected = False
    def __init__(self,secrets,name="ESP32"):
        self.name = name
        self.secrets = secrets
        self.init_status_leds()
        self.set_animation_wakeup()
        try:
            
            self.init_iic_bus()
            self.init_co2_sensor()
            self.init_particulate_sensor()
            self.connected = self.connect_network()
            self.set_animation_wakeup()
            self.short_animation(5000)
        except Exception as e:
            self.log("Unable to initialise! %s"%(e))
            self.set_animation_wakeup_error()
            self.short_animation(5000)
            time.sleep(10)
            self.log(" Rebooting!")
            #supervisor.reload()

        return

    def connect_network(self):
        self.set_animation_wifi_connect()
        self.short_animation(5000)
        if self.init_wifi():
                self.set_animation_yeswifi()
                self.short_animation(5000)
                self.init_mqtt()
                self.connect_mqtt()
                return True
        else:
                self.set_animation_nowifi()
                self.short_animation(5000)
                return False

    def connect_mqtt(self):
            self.mqtt_client.connect()

    def loop(self):
        try:
            self.mqtt_client.loop()
            self.mqtt_client.ping()
            self.log("Polling Sensors..")
            self.get_particulate_data()
            self.get_atmosphere_data()
            self.calculate_air_quality()
            self.upload_sensor_data()
            self.led_quality()
            time.sleep(30)
        except Exception as e:
            self.set_animation_nowifi()
            self.short_animation(2000)
            self.log("Unhandled Exception %s\n Rebooting"%(e))
            supervisor.reload()

    def calculate_air_quality(self):
        pass

    def upload_sensor_data(self):
        if self.new_pm or self.new_atmo:
            self.set_animation_upload()
            self.short_animation(3000)
        try:
            if self.new_pm:
                self.publish_particulate_data()
                self.new_pm=False
            if self.new_atmo:
                self.publish_atmosphere_data()
                self.new_atmo=False
        except OSError:
            self.set_animation_nowifi()
            self.short_animation(2000)
            self.log("Error Sending Data")
            self.mqtt_client.reconnect()
        except MQTT.MMQTTException:
            self.set_animation_nowifi()
            self.short_animation(2000)
            self.log("MQTT Error")
            self.mqtt_client.reconnect()

    def log(self,message):
        print("[AirMonitor %s]\t%s"%(self.name,message))

    def init_iic_bus(self):
        self.log("Initializing i2c Bus")
        self.i2c = busio.I2C(board.SCL, board.SDA, frequency=100000)

    def init_particulate_sensor(self):
        self.log("Initializing PM25 Particle Sensor")
        self.pm25 = PM25_I2C(self.i2c)

    def init_co2_sensor(self):
        self.log("Initializing SCD4x CO2 Sensor")
        self.scd4x = adafruit_scd4x.SCD4X(self.i2c)
        self.log("Starting Periodic Measurment")
        self.scd4x.start_periodic_measurement()

    def init_wifi(self):
        try:
            self.log("Initializing WiFi")
            self.log("\tConnecting to %s..." % self.secrets["ssid"])
            wifi.radio.connect(secrets["ssid"], self.secrets["password"])
            self.log("\tSuccess!")
            self.pool = socketpool.SocketPool(wifi.radio)
            return True
        except ConnectionError:
            self.log("\tFailure!")
            return False


    def init_mqtt(self):
        self.mqtt_client = MQTT.MQTT(
            broker=self.secrets["broker"],
            port=self.secrets["port"],
            socket_pool=self.pool,
            is_ssl=False,
        )


    def get_particulate_data(self):
        try:
            self.log("Getting Particulate Data..")
            self.aqdata = self.pm25.read()
            self.log("\tSuccess!")
            self.new_pm=True
            return True
        except RuntimeError:
            self.log("\tUnable to read from PM25")
        self.new_pm=False
        return False

    def get_atmosphere_data(self):
        try:
            self.log("Getting Atmospheric Data..")
            if self.scd4x.data_ready:
                self.new_atmo=True
                return True
            else:
                self.log("\tSensor not ready..")
        except RuntimeError:
            self.log("Unable to read from SCX4x")
        self.new_atmo=False
        return False

    def publish_particulate_data(self):
        self.log("Publising Particulate Data")
        mqtt_client = self.mqtt_client
        aqdata = self.aqdata
        devicename = self.name
        mqtt_client.publish("%s/feeds/pm/standard/10"%(devicename),
            aqdata["pm10 standard"])
        mqtt_client.publish("%s/feeds/pm/standard/25"%(devicename),
            aqdata["pm25 standard"])
        mqtt_client.publish("%s/feeds/pm/standard/100"%(devicename),
            aqdata["pm100 standard"])

        mqtt_client.publish("%s/feeds/pm/env/10"%(devicename),
            aqdata["pm10 env"])
        mqtt_client.publish("%s/feeds/pm/env/25"%(devicename),
            aqdata["pm25 env"])
        mqtt_client.publish("%s/feeds/pm/env/100"%(devicename),
            aqdata["pm100 env"])

        mqtt_client.publish("%s/feeds/particles/03um"%(devicename),
            aqdata["particles 03um"])
        mqtt_client.publish("%s/feeds/particles/05um"%(devicename),
            aqdata["particles 05um"])
        mqtt_client.publish("%s/feeds/particles/10um"%(devicename),
            aqdata["particles 10um"])
        mqtt_client.publish("%s/feeds/particles/25um"%(devicename),
            aqdata["particles 25um"])
        mqtt_client.publish("%s/feeds/particles/50um"%(devicename),
            aqdata["particles 50um"])
        mqtt_client.publish("%s/feeds/particles/100um"%(devicename),
            aqdata["particles 100um"])

    def publish_atmosphere_data(self):
        devicename = self.name
        mqtt_client = self.mqtt_client
        mqtt_client.publish("%s/feeds/env/co2"%(devicename),
            self.scd4x.CO2)
        mqtt_client.publish("%s/feeds/env/temp"%(devicename),
            self.scd4x.temperature)
        mqtt_client.publish("%s/feeds/env/humid"%(devicename),
            self.scd4x.relative_humidity)

    def init_status_leds(self):
        self.pixels = neopixel.NeoPixel(board.IO6, 8, brightness=0.1, auto_write=False)
        self.led_animation_nowifi = Comet(self.pixels, speed=0.05, color=RED, tail_length=4, bounce=True)
        self.led_animation_wakeup = Blink(self.pixels, speed=0.5, color=JADE)
        self.led_animation_wakeup_error = Blink(self.pixels, speed=0.2, color=RED)
        self.led_animation_yeswifi = Comet(self.pixels, speed=0.05, color=GREEN, tail_length=4, bounce=True)
        self.led_animation_upload = Comet(self.pixels, speed=0.05, color=AMBER, tail_length=4, bounce=False)
        self.led_animation_comms_error = Comet(self.pixels, speed=0.01, color=RED, tail_length=4, bounce=False)

    def set_animation_nowifi(self):
        self.animations = AnimationSequence(self.led_animation_nowifi, advance_interval=10, auto_clear=True)

    def set_animation_yeswifi(self):
        self.animations = AnimationSequence(self.led_animation_yeswifi, advance_interval=10, auto_clear=True)

    def set_animation_wakeup(self):
        self.animations = AnimationSequence(self.led_animation_wakeup, advance_interval=10, auto_clear=True)

    def set_animation_upload(self):
        self.animations = AnimationSequence(self.led_animation_upload, advance_interval=10, auto_clear=True)
        
    def set_animation_comms_error(self):
        self.animations = AnimationSequence(self.led_animation_comms_error, advance_interval=10, auto_clear=True)

    def set_animation_wakeup_error(self):
        pass

    def set_animation_wifi_connect(self):
        pass

    def led_quality(self):
        self.pixels.fill((0, 100, 0))
        self.pixels.show()

    def short_animation(self,cycles):
        for i in range(1,cycles):
            self.animations.animate()
        self.pixels.fill((0, 0, 0))
        self.pixels.show()
try:
    from secrets import secrets
except ImportError:
    print("WiFi secrets are kept in secrets.py, please add them there!")
    raise

monitor = AirMonitor(secrets)

while True:
    monitor.loop()
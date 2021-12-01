import time
import supervisor
import ssl
import socketpool
import wifi
import adafruit_minimqtt.adafruit_minimqtt as MQTT
import board
import neopixel
import busio
import math
from digitalio import DigitalInOut, Direction, Pull
from adafruit_pm25.i2c import PM25_I2C
import adafruit_sgp30
from adafruit_led_animation.animation.blink import Blink
from adafruit_led_animation.animation.comet import Comet
from adafruit_led_animation.animation.chase import Chase
from adafruit_led_animation.animation.pulse import Pulse
from adafruit_led_animation.color import PURPLE, AMBER, JADE, RED, GREEN,WHITE,MAGENTA
from adafruit_led_animation.sequence import AnimationSequence, AnimateOnce

import adafruit_scd4x

DATABASE = "floor3apt"
ROOM = "livingroom"
PARTICULATE_TOPIC_PM_STANDARD_10 = f"{DATABASE}/{ROOM}/pm_standard_10"
PARTICULATE_TOPIC_PM_STANDARD_25 = f"{DATABASE}/{ROOM}/pm_standard_25"
PARTICULATE_TOPIC_PM_STANDARD_100 = f"{DATABASE}/{ROOM}/pm_standard_100"
PARTICULATE_TOPIC_PM_ENVIRONMENT_10 = f"{DATABASE}/{ROOM}/pm_environment_10"
PARTICULATE_TOPIC_PM_ENVIRONMENT_25 = f"{DATABASE}/{ROOM}/pm_environment_25"
PARTICULATE_TOPIC_PM_ENVIRONMENT_100 = f"{DATABASE}/{ROOM}/pm_environment_100"
PARTICULATE_TOPIC_PARTICLES_03UM = f"{DATABASE}/{ROOM}/pm_particles_0_3um"
PARTICULATE_TOPIC_PARTICLES_05UM = f"{DATABASE}/{ROOM}/pm_particles_0_5um" 
PARTICULATE_TOPIC_PARTICLES_10UM = f"{DATABASE}/{ROOM}/pm_particles_1_0um"
PARTICULATE_TOPIC_PARTICLES_25UM = f"{DATABASE}/{ROOM}/pm_particles_2_5um"
PARTICULATE_TOPIC_PARTICLES_50UM = f"{DATABASE}/{ROOM}/pm_particles_5_0um"
PARTICULATE_TOPIC_PARTICLES_100UM = f"{DATABASE}/{ROOM}/pm_particles_10_0um"

ATMOSPHERE_TOPIC_CO2 = f"{DATABASE}/{ROOM}/am_co2_ppm"
ATMOSPHERE_TOPIC_TEMPERATURE = f"{DATABASE}/{ROOM}/am_temp_degc"
ATMOSPHERE_TOPIC_HUMIDITY = f"{DATABASE}/{ROOM}/am_humid_rel"


ATMOSPHERE_TOPIC_ECO2 = f"{DATABASE}/{ROOM}/am_eco2_ppm"
ATMOSPHERE_TOPIC_TVOC = f"{DATABASE}/{ROOM}/am_tvoc_ppm"

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

        #while True:
        #    self.short_animation(8000,self.led_animation_nowifi)
        #    time.sleep(3)

        self.short_animation(8000,self.led_animation_wakeup)
        try:

            self.init_iic_bus()
            self.init_co2_sensor()
            self.init_particulate_sensor()
            self.init_voc_sensor()
            self.connected = self.connect_network()
            self.short_animation(5000,self.led_animation_wakeup)
            self.calibrate_voc_sensor()
        except Exception as e:
            self.log("Unable to initialise! %s"%(e))
            self.short_animation(5000,self.led_animation_wakeup_error)
            time.sleep(10)
            self.log(" Rebooting!")
            #supervisor.reload()

        return

    def calibrate_voc_sensor(self):
        rh = self.scd4x.relative_humidity
        T = self.scd4x.temperature
        self.log("Calibrating VOC with RH: %s and T: %s"%(rh,T))
        self.sgp30.set_iaq_humidity(self.rel_to_abs_humidity(rh,T))


    def connect_network(self):
        self.short_animation(5000,self.led_animation_wifi_connect)
        if self.init_wifi():
                self.short_animation(5000,self.led_animation_yeswifi)
                self.init_mqtt()
                self.connect_mqtt()
                return True
        else:
                self.short_animation(5000,self.led_animation_nowifi)
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
            self.get_voc_Data()
            self.calculate_air_quality()
            self.upload_sensor_data()
            self.led_quality()
            time.sleep(10)
        except Exception as e:
            self.short_animation(2000,self.led_animation_nowifi)
            self.log("Unhandled Exception %s\n Rebooting"%(e))
            supervisor.reload()

    def calculate_air_quality(self):
        pass

    def upload_sensor_data(self):
        if self.new_pm or self.new_atmo or self.new_voc:
            self.short_animation(3000,self.led_animation_upload)
        try:
            if self.new_pm:
                self.publish_particulate_data()
                self.new_pm=False
            if self.new_atmo:
                self.publish_atmosphere_data()
                self.new_atmo=False
            if self.new_voc:
                self.publish_voc_data()
                self.new_voc = False
        except OSError:
            self.short_animation(2000,self.led_animation_comms_error)
            self.log("Error Sending Data")
            self.mqtt_client.reconnect()
        except MQTT.MMQTTException:
            self.short_animation(2000,self.led_animation_comms_error)
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
        #self.log("Starting Periodic Measurment")
        self.scd4x.start_periodic_measurement()

    def init_voc_sensor(self):
        self.log("Initializing SGP30 VOC Sensor")
        self.sgp30 = adafruit_sgp30.Adafruit_SGP30(self.i2c)


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
            keep_alive=300,
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

    def get_voc_Data(self):
        try:
            self.sgp_eCO2, self.sgp_TVOC = self.sgp30.iaq_measure()
            self.new_voc = True
        except RuntimeError:
            self.log("Unable to read from SGP30")

    def publish_particulate_data(self):
        self.log("Publising Particulate Data")
        mqtt_client = self.mqtt_client
        aqdata = self.aqdata
        devicename = self.name
        mqtt_client.publish(PARTICULATE_TOPIC_PM_STANDARD_10,
            aqdata["pm10 standard"])
        mqtt_client.publish(PARTICULATE_TOPIC_PM_STANDARD_25,
            aqdata["pm25 standard"])
        mqtt_client.publish(PARTICULATE_TOPIC_PM_STANDARD_100,
            aqdata["pm100 standard"])

        mqtt_client.publish(PARTICULATE_TOPIC_PM_ENVIRONMENT_10,
            aqdata["pm10 env"])
        mqtt_client.publish(PARTICULATE_TOPIC_PM_ENVIRONMENT_25,
            aqdata["pm25 env"])
        mqtt_client.publish(PARTICULATE_TOPIC_PM_ENVIRONMENT_100,
            aqdata["pm100 env"])

        mqtt_client.publish(PARTICULATE_TOPIC_PARTICLES_03UM,
            aqdata["particles 03um"])
        mqtt_client.publish(PARTICULATE_TOPIC_PARTICLES_05UM,
            aqdata["particles 05um"])
        mqtt_client.publish(PARTICULATE_TOPIC_PARTICLES_10UM,
            aqdata["particles 10um"])
        mqtt_client.publish(PARTICULATE_TOPIC_PARTICLES_25UM,
            aqdata["particles 25um"])
        mqtt_client.publish(PARTICULATE_TOPIC_PARTICLES_50UM,
            aqdata["particles 50um"])
        mqtt_client.publish(PARTICULATE_TOPIC_PARTICLES_100UM,
            aqdata["particles 100um"])
            
            
    def publish_atmosphere_data(self):
        self.log("Publising Atmospheric Data")
        devicename = self.name
        mqtt_client = self.mqtt_client
        mqtt_client.publish(ATMOSPHERE_TOPIC_CO2,
            self.scd4x.CO2)
        mqtt_client.publish(ATMOSPHERE_TOPIC_TEMPERATURE,
            self.scd4x.temperature)
        mqtt_client.publish(ATMOSPHERE_TOPIC_HUMIDITY,
            self.scd4x.relative_humidity)

    def publish_voc_data(self):
        self.log("Publising Volatile Organic Compound Data")
        devicename = self.name
        mqtt_client = self.mqtt_client
        mqtt_client.publish(ATMOSPHERE_TOPIC_ECO2,
            self.sgp_eCO2)
        mqtt_client.publish(ATMOSPHERE_TOPIC_TVOC,
            self.sgp_TVOC)

    def init_status_leds(self):
        self.pixels = neopixel.NeoPixel(board.IO6, 8, brightness=1, auto_write=False)
        self.led_animation_nowifi = Pulse(self.pixels,speed=0.0001,color=RED)
        self.led_animation_wakeup = Pulse(self.pixels, speed=0.005, color=WHITE)
        self.led_animation_wakeup_error = Pulse(self.pixels,speed=0.0001,color=RED)
        self.led_animation_yeswifi = Pulse(self.pixels,speed=0.0001,color=GREEN)
        self.led_animation_wifi_connect = Comet(self.pixels, speed=0.05, color=WHITE, tail_length=4, bounce=True)
        self.led_animation_upload = Comet(self.pixels, speed=0.07, color=AMBER, tail_length=4, bounce=False)
        self.led_animation_comms_error = Pulse(self.pixels,0.0001,color=MAGENTA)

    def rel_to_abs_humidity(self,rh,T):
        return ( 6.112*math.exp((17.67*T)/(T+243.5) )*rh*2.1674 )/(273.15+T)

    def led_quality(self):
        self.pixels.fill((0, 100, 0))
        self.pixels.show()

    def short_animation(self,cycles,animaton):
        self.animations = AnimateOnce(animaton,auto_clear=True)
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
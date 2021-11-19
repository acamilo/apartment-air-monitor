print("Sensor is awake!\nLoading Libraries...")
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

pixels = neopixel.NeoPixel(board.IO6, 8, brightness=0.5, auto_write=False)
blink = Blink(pixels, speed=0.5, color=JADE)
chase = Chase(pixels, speed=0.1, size=6, spacing=2, color=GREEN)
comet = Comet(pixels, speed=0.05, color=PURPLE, tail_length=4, bounce=True)


wifi_error = Comet(pixels, speed=0.05, color=RED, tail_length=4, bounce=True)

wifi_conn = Chase(pixels, speed=0.1, size=6, spacing=2, color=GREEN)
    
try:
    from secrets import secrets
except ImportError:
    print("WiFi secrets are kept in secrets.py, please add them there!")
    raise

try:    
    print("Connecting to %s" % secrets["ssid"])
    wifi.radio.connect(secrets["ssid"], secrets["password"])
    print("Connected to %s!" % secrets["ssid"])
except ConnectionError:
    animations = AnimationSequence(wifi_error, advance_interval=10, auto_clear=True)
    for i in range(1,50000):
        animations.animate()
        
    supervisor.reload()

# Define callback methods which are called when events occur
# pylint: disable=unused-argument, redefined-outer-name
def connected(client, userdata, flags, rc):
    # This function will be called when the client is connected
    # successfully to the broker.
    print("Connected to MQTT Broker!\n\tClient:\t%s\n\tuserdata:\t%s\n\tflags:\t%s\n\trc:\t%s" % (client,userdata,flags,rc))



def disconnected(client, userdata, rc):
    # This method is called when the client is disconnected
    print("Disconnected from MQTT Broker!")


def message(client, topic, message):
    # This method is called when a topic the client is subscribed to
    # has a new message.
    print("New message on topic {0}: {1}".format(topic, message))
    
# Create a socket pool
pool = socketpool.SocketPool(wifi.radio)

mqtt_client = MQTT.MQTT(
    broker=secrets["broker"],
    port=secrets["port"],
    socket_pool=pool,
    is_ssl=False,
    
)

# Setup the callback methods above
mqtt_client.on_connect = connected
mqtt_client.on_disconnect = disconnected
mqtt_client.on_message = message

# Connect the client to the MQTT broker.
print("Connecting to MQTT Broker...")
mqtt_client.connect()

print("PM25 Init")
i2c = busio.I2C(board.SCL, board.SDA, frequency=100000)
pm25 = PM25_I2C(i2c, reset_pin)

print("SCD4x Init")
scd4x = adafruit_scd4x.SCD4X(i2c)
print("Serial number:", [hex(i) for i in scd4x.serial_number])
scd4x.start_periodic_measurement()

device_name="esp32"

time.sleep(1)
i=0
while True:
    animations.animate()
    mqtt_client.loop()
    # Collect C02
    
    # Collect particles
    try:
        aqdata = pm25.read()
        mqtt_client.publish("%s/feeds/pm/stadard/10"%(devicename),
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
    except RuntimeError:
        print("Unable to read from PM25")
        
    try:
        if scd4x.data_ready:
            mqtt_client.publish("%s/feeds/env/co2"%(devicename),
                scd4x.CO2)
            mqtt_client.publish("%s/feeds/env/temp"%(devicename),
                scd4x.temperature)
            mqtt_client.publish("%s/feeds/env/humid"%(devicename),
                scd4x.relative_humidity)
    except RuntimeError:
        print("Unable to read from SCX4x")
    
    mqtt_client.publish("esp32/feeds/test",i)
    i=i+1
    time.sleep(5)
"""
'publish.py'
=========================================
Publishes an incrementing value to a feed
Author(s): Brent Rubell, Todd Treece for Adafruit Industries
"""
# Import standard python modules
import time
import serial
import sys

# Import Adafruit IO REST client.
from Adafruit_IO import Client, Feed

#
from Adafruit_IO import MQTTClient

# holds the count for the feed
run_count = 0
servo = ['A','B','C','D','E','F','G','H','I','J','K','L']
# Set to your Adafruit IO key.
# Remember, your key is a secret,
# so make sure not to publish it when you publish this code!
ADAFRUIT_IO_KEY = 'aio_oTAy85WbEpuhrK6Y1ow2qVFKUyQF'

# Set to your Adafruit IO username.
# (go to https://accounts.adafruit.com to find your username)
ADAFRUIT_IO_USERNAME = 'andresini58'


# Create an instance of the REST client.
aio = Client(ADAFRUIT_IO_USERNAME, ADAFRUIT_IO_KEY)
feeds = aio.feeds()
print(feeds)

# Create a new feed named 'counter'
feed = Feed(name="control")
FEED_ID = 'modos'
FEED_ID1 = 'servo1'
FEED_ID2 = 'servo2'
FEED_ID3 = 'servo3'
FEED_ID4 = 'servo4'
FEED_ID5 = 'servo1b'
FEED_ID6 = 'servo2b'
FEED_ID7 = 'servo3b'
FEED_ID8 = 'servo4b'
#response = aio.create_feed(feed)

ser = serial.Serial('COM5')
print(ser.name)

# Define callback functions which will be called when certain events happen.
def connected(client):
    """Connected function will be called when the client is connected to
    Adafruit IO.This is a good place to subscribe to feed changes.  The client
    parameter passed to this function is the Adafruit IO MQTT client so you
    can make calls against it easily.
    """
    # Subscribe to changes on a feed named Counter.
    print('Subscribing to Feed {0}'.format(FEED_ID))
    client.subscribe(FEED_ID)
    client.subscribe(FEED_ID1)
    client.subscribe(FEED_ID2)
    client.subscribe(FEED_ID3)
    client.subscribe(FEED_ID4)
    client.subscribe(FEED_ID5)
    client.subscribe(FEED_ID6)
    client.subscribe(FEED_ID7)
    client.subscribe(FEED_ID8)
    print('Waiting for feed data...')

def disconnected(client):
    """Disconnected function will be called when the client disconnects."""
    sys.exit(1)

def message(client, feed_id, payload):
    """Message function will be called when a subscribed feed has a new value.
    The feed_id parameter identifies the feed, and the payload parameter has
    the new value.
    """
    print('Feed {0} received new value: {1}'.format(feed_id, payload))
    if (feed_id == 'modos'):
        if (payload == '1'):
            ser.write(bytes('z','utf-8'))
        if (payload == '2'):
            ser.write(bytes('x','utf-8'))
        if (payload == '3'):
            ser.write(bytes('c','utf-8')) 
    if (feed_id == 'servo1'):
        if(payload == '1'):
            ser.write(bytes('d','utf-8'))
    if (feed_id == 'servo1b'):
        if(payload == '1'):
            ser.write(bytes('a','utf-8'))
    if (feed_id == 'servo2'):
        if(payload == '1'):
            ser.write(bytes('w','utf-8'))
    if (feed_id == 'servo2b'):
        if(payload == '1'):
            ser.write(bytes('s','utf-8'))
    if (feed_id == 'servo3'):
        if(payload == '1'):
            ser.write(bytes('l','utf-8'))
    if (feed_id == 'servo3b'):
        if(payload == '1'):
            ser.write(bytes('j','utf-8'))
    if (feed_id == 'servo4'):
        if(payload == '1'):
            ser.write(bytes('i','utf-8'))
    if (feed_id == 'servo4b'):
        if(payload == '1'):
            ser.write(bytes('k','utf-8')) 

# Create an MQTT client instance.
client = MQTTClient(ADAFRUIT_IO_USERNAME, ADAFRUIT_IO_KEY)

# Setup the callback functions defined above.
client.on_connect = connected
client.on_disconnect = disconnected
client.on_message = message

# Connect to the Adafruit IO server.
client.connect()

# The first option is to run a thread in the background so you can continue
# doing things in your program.
client.loop_blocking()


while True:
    run_count = ser.readline()
    print(run_count)

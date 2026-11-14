import paho.mqtt.client as mqtt
from fleetmanager.fleetmanager import FleetManager
import time
import json

mqtt_broker = "localhost"
client = mqtt.Client("FleetManager")
client.connect(mqtt_broker)

while True:
    client.loop()
    time.sleep(1)

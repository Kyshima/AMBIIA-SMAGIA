import sys
import paho.mqtt.client as paho

def message_handling(client, userdata, msg):
    print(f"{msg.topic}: {msg.payload.decode()}")

client = paho.Client()
client.on_message = message_handling

try:
    if client.connect("localhost", 1883, 60) != 0:
        print("Couldn't connect to the MQTT broker")
        sys.exit(1)

    result, mid = client.subscribe("test_topic")
    if result != paho.MQTT_ERR_SUCCESS:
        print(f"Failed to subscribe to topic: {result}")
        sys.exit(1)

    print("Press CTRL+C to exit...")
    client.loop_forever()
except KeyboardInterrupt:
    print("Caught KeyboardInterrupt, exiting...")
except Exception as e:
    print(f"Caught an Exception, something went wrong: {e}")
finally:
    print("Disconnecting from the MQTT broker")
    client.disconnect()

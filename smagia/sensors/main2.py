from sensors_mqtt import HumiditySensorAgent
import spade



async def main():
    robot_network = ['robot1@jabbers.one', 'robot2@jabbers.one', 'robot3@jabbers.one', 'robot4@jabbers.one']

    sensor_humidity2 = HumiditySensorAgent("sensor_humidity2@jabbers.one", "sensor_humidity2", 71, 0.5, "sensor1", 0, 10, robot_network, "1")
    sensor_humidity2.set("receiver_jid", "receiver@jabbers.one")
    await sensor_humidity2.start(auto_register=True)

    sensor_humidity2.web.start(hostname="127.0.0.1", port="10001")

    await spade.wait_until_finished(sensor_humidity2)
    await sensor_humidity2.stop()
    print("Receiver Agent finished")


if __name__ == "__main__":
    spade.run(main())

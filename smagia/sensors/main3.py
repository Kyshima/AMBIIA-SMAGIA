from sensors_network import HumiditySensorAgent
import spade



async def main():
    robot_network = ['robot1@jabbers.one', 'robot2@jabbers.one', 'robot3@jabbers.one', 'robot4@jabbers.one']

    sensor_humidity3 = HumiditySensorAgent("sensor_humidity3@jabbers.one", "sensor_humidity3", 71, 0.5, "sensor3", 4, -3.5, robot_network)
    sensor_humidity3.set("receiver_jid", "receiver@jabbers.one")
    await sensor_humidity3.start(auto_register=True)

    sensor_humidity3.web.start(hostname="127.0.0.1", port="10002")

    await spade.wait_until_finished(sensor_humidity3)
    await sensor_humidity3.stop()
    print("Receiver Agent finished")


if __name__ == "__main__":
    spade.run(main())

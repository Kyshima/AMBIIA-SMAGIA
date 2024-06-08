from receiver import ReceiverAgent
import spade

from robots.robots import RobotAgent
from sensors.sensors import HumiditySensorAgent


#receiver@jabbers.one receiver

async def main():
    receiver = ReceiverAgent("receiver@jabbers.one", "receiver")
    await receiver.start()
    print("Receiver Agent started!")

    sensor_humidity1 = HumiditySensorAgent("sensor_humidity1@jabbers.one", "sensor_humidity1", 100, 5, "sensor1", 3, 7)
    sensor_humidity1.set("receiver_jid", "receiver@jabbers.one")
    await sensor_humidity1.start(auto_register=True)

    sensor_humidity2 = HumiditySensorAgent("sensor_humidity2@jabbers.one", "sensor_humidity2", 100, 5, "sensor2", 4, 7)
    sensor_humidity2.set("receiver_jid", "receiver@jabbers.one")
    await sensor_humidity2.start(auto_register=True)

    robot1 = RobotAgent("robot1@jabbers.one", "robot1", 500, 500, "robot1", 1, 1, 10)
    robot1.set("receiver_jid", "receiver@jabbers.one")
    await robot1.start(auto_register=True)

    robot2 = RobotAgent("robot2@jabbers.one", "robot2", 300, 300, "robot2", 9, 9, 8)
    robot2.set("receiver_jid", "receiver@jabbers.one")
    await robot2.start(auto_register=True)

    sensor_humidity1.web.start(hostname="127.0.0.1", port="10000")
    sensor_humidity2.web.start(hostname="127.0.0.1", port="10001")
    robot1.web.start(hostname="127.0.0.1", port="10002")
    robot2.web.start(hostname="127.0.0.1", port="10003")
    receiver.web.start(hostname="127.0.0.1", port="10004")

    await spade.wait_until_finished(receiver)
    await sensor_humidity1.stop()
    await sensor_humidity2.stop()
    await robot1.stop()
    await robot2.stop()
    await receiver.stop()
    print("Receiver Agent finished")


if __name__ == "__main__":
    spade.run(main())

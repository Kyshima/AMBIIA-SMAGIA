from sensors_network import HumiditySensorAgent
import spade


#sensor_humidity1@jabbers.one sensor_humidity1
#sensor_humidity2@jabbers.one sensor_humidity2
#sensor_humidity3@jabbers.one sensor_humidity3
#sensor_humidity4@jabbers.one sensor_humidity4
#sensor_humidity5@jabbers.one sensor_humidity5
#sensor_humidity6@jabbers.one sensor_humidity6
#sensor_humidity7@jabbers.one sensor_humidity7
#sensor_humidity8@jabbers.one sensor_humidity8
#sensor_humidity9@jabbers.one sensor_humidity9
#sensor_humidity10@jabbers.one sensor_humidity10
#sensor_humidity11@jabbers.one sensor_humidity11
#sensor_humidity12@jabbers.one sensor_humidity12
#sensor_humidity13@jabbers.one sensor_humidity13
#sensor_humidity14@jabbers.one sensor_humidity14
#sensor_humidity15@jabbers.one sensor_humidity15
#sensor_humidity16@jabbers.one sensor_humidity16

async def main():
    robot_network = ['robot1@jabbers.one', 'robot2@jabbers.one', 'robot3@jabbers.one', 'robot4@jabbers.one']

    sensor_humidity1 = HumiditySensorAgent("sensor_humidity1@jabbers.one", "sensor_humidity1", 100, 5, "sensor1", 3, 7, robot_network)
    sensor_humidity1.set("receiver_jid", "receiver@jabbers.one")
    await sensor_humidity1.start(auto_register=True)

    sensor_humidity1.web.start(hostname="127.0.0.1", port="10000")

    await spade.wait_until_finished(sensor_humidity1)
    await sensor_humidity1.stop()
    print("Receiver Agent finished")


if __name__ == "__main__":
    spade.run(main())

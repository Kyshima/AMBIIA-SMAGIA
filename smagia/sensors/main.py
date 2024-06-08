from sensors import HumiditySensorAgent
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


    sensor = HumiditySensorAgent(jid="sensor_humidity1@jabbers.one", password="sensor_humidity1")
    sensor.set("id", "sensor")
    sensor.set("humidity", 100)
    sensor.set("decrease_amount", 0.1)
    await sensor.start(auto_register=True)
    print("Sender started")
    sensor.web.start(hostname="127.0.0.1", port="10000")


    await spade.wait_until_finished(sensor)
    await sensor.stop()
    print("Sensors finished")

if __name__ == "__main__":
    spade.run(main())


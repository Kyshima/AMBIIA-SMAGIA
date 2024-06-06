from sensors import PeriodicSensorAgent
import spade

async def main():

    sensors = []
    for i in range(16):
        sensors.append(PeriodicSensorAgent(jid="testesmagia@jabbers.one", password="password"))
        sensors[i].set("id", "sensor" + str(i))
        sensors[i].set("humidity", 100)
        sensors[i].set("decrease_amount", 0.1 * i)
        await sensors[i].start(auto_register=True)
        print("Sender started")
        sensors[i].web.start(hostname="127.0.0.1", port="10000")


    await spade.wait_until_finished(sensors[0])
    await sensors[0].stop()
    print("Agents finished")


if __name__ == "__main__":
    spade.run(main())


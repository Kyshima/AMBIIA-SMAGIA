from water_station import WaterStationAgent
import spade

async def main():
    jid = "water_station@jabbers.one"
    password = "water_station"
    water_station_agent = WaterStationAgent(jid, password)
    
    
    await water_station_agent.start(auto_register=True)
    print("Water Station Agent Started")

    water_station_agent.web.start(hostname="127.0.0.1", port="1007")

    await spade.wait_until_finished(water_station_agent)
    await water_station_agent.stop()
    print("Water Station Agent Finished")

if __name__ == "__main__":
    spade.run(main())


from water_station import WaterStationAgent
import spade

async def main():
    jid = "water_station@jabbers.one"
    password = "water_station"
    refill_water_quantity = 100
    water_station_posx = -7.5
    water_station_posy = 7.17
    water_station_agent = WaterStationAgent(jid, password, refill_water_quantity, water_station_posx, water_station_posy)
    
    await water_station_agent.start(auto_register=True)
    print("Water Station Agent Started")

    water_station_agent.web.start(hostname="127.0.0.1", port="1007")

    await spade.wait_until_finished(water_station_agent)
    await water_station_agent.stop()
    print("Water Station Agent Finished")

if __name__ == "__main__":
    spade.run(main())
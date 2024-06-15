from energy_station import EnergyStationAgent
import spade

async def main():
    jid = "energy_station@jabbers.one"
    password = "energy_station"
    energy_station_agent = EnergyStationAgent(jid, password, 100, 0, 0)
    
    await energy_station_agent.start(auto_register=True)
    print("Energy Station Agent Started")

    energy_station_agent.web.start(hostname="127.0.0.1", port="1009")

    await spade.wait_until_finished(energy_station_agent)
    await energy_station_agent.stop()
    print("Energy Station Agent Finished")

if __name__ == "__main__":
    spade.run(main())
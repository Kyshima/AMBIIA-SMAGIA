from spade import agent
from spade.behaviour import CyclicBehaviour
from spade.message import Message
import asyncio

class RobotAgent(agent.Agent):
    async def setup(self):

        print(f"Robot Agent {self.jid} is running")

        #Recharge Energy Request
        energy_recharge_request = Message(to="energy_station@jabbers.one")
        energy_recharge_request.body = "Requesting energy recharge"
        energy_recharge_request.set_metadata("performative", "request")
        energy_recharge_request.set_metadata("message_type", "Energy Recharge Request")
        await self.send(energy_recharge_request)

        #Recharge Water Request
        water_recharge_request = Message(to="water_station@jabbers.one")
        water_recharge_request.body = "Requesting water recharge"
        water_recharge_request.set_metadata("performative", "request")
        water_recharge_request.set_metadata("message_type", "Water Recharge Request")
        await self.send(water_recharge_request)


class SensorAgent(agent.Agent):
    async def setup(self):
        print(f"Sensor Agent {self.jid} is running")

    async def run(self):
        # Request water
        for i in range(1, 5):
            plant_watering_request = Message(to=f"robot{i}@jabbers.one")
            plant_watering_request.body = "Plant Watering Request: Low humidity level"
            plant_watering_request.set_metadata("performative", "request")
            plant_watering_request.set_metadata("message_type", "Plant Watering Request")
            await self.send(plant_watering_request)


class CoordinatorAgent(agent.Agent):
    
    async def setup(self):    
        print(f"Coordinator Agent {self.jid} is running")
        self.add_behaviour(self.RequestPositionBehaviour())

    class RequestPositionBehaviour(CyclicBehaviour):
        async def run(self):
        
            for i in range(1, 5):
                position_request = Message(to=f"robot{i}@jabbers.one")
                position_request.body = "Requesting position update"
                position_request.set_metadata("performative", "request")
                position_request.set_metadata("message_type", "Position Update Request")
                await self.send(position_request)
                await asyncio.sleep(10)

if __name__ == "__main__":
    coordinator_agent = CoordinatorAgent("coordinator@jabbers.one", "coordinator")
    
    for i in range(1, 5):
        robot_agent = RobotAgent(f"robot{i}@jabbers.one", f"robot{i}")
        robot_agent.start(auto_register=True)
    
    for i in range(1, 17):
        sensor_agent = SensorAgent(f"sensor_humidity{i}@jabbers.one", f"sensor_humidity{i}")
        sensor_agent.start(auto_register=True)
    
    coordinator_agent.start(auto_register=True)

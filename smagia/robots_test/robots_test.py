from spade.agent import Agent
from spade.message import Message
from spade.behaviour import CyclicBehaviour

class RobotAgent(Agent):
    async def setup(self):
        
        print(f"Robot Agent {self.jid} is running with water capacity at {self.water_capacity}%")
        self.add_behaviour(self.WaterRechargeBehaviour())

    class WaterRechargeBehaviour(CyclicBehaviour):
        async def run(self):
            if self.agent.water_capacity < 100:
                
                water_recharge_request = Message(to="water_station@jabbers.one")
                water_recharge_request.body = "Requesting water recharge"
                water_recharge_request.set_metadata("performative", "request")
                water_recharge_request.set_metadata("message_type", "Water Recharge Request")
                
                await self.send(water_recharge_request)
                
                response = await self.receive(timeout=10)
                
                if response:
                    if response.get_metadata("response_status") == "accepted":

                        self.agent.water_capacity += 10
                        print(f"Water capacity increased to {self.agent.water_capacity}%")
                    else:
                        print("Water recharge denied: The station is already refilling")
            else:
                print("Water capacity is already at 100%. Stopping recharge.")
                await self.agent.stop()

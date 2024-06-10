from spade.agent import Agent
from spade.message import Message
from spade.behaviour import CyclicBehaviour

class RobotAgent(Agent):

    async def setup(self):
        print(f"Robot Agent {self.jid} is running with water capacity at {self.water_capacity}%")
        print(f"Robot Agent {self.jid} is running with energy capacity at {self.energy_capacity}%")

        '''if self.energy_capacity<=30:
            self.add_behaviour(self.EnergyChargeBehaviour())

        elif self.energy_capacity > 30 and self.water_capacity<=30:
            self.add_behaviour(self.WaterRefillBehaviour())'''
        
        self.add_behaviour(self.WaterRefillBehaviour())


    class WaterRefillBehaviour(CyclicBehaviour):
        async def run(self):
            if self.agent.water_capacity < 100:
                # Send water refill request
                water_refill_request = Message(to="water_station@jabbers.one")
                water_refill_request.body = str(self.agent.water_capacity)
                water_refill_request.set_metadata("performative", "request")
                water_refill_request.set_metadata("message_type", "Water Refill Request")

                await self.send(water_refill_request)
                
                # Wait for response
                response = await self.receive(timeout=10)
                
                if response:
                    if response.get_metadata("response_status") == "accepted":
                        print(f"Message received: {response.body}")
                        print("Waiting for update")

                        while self.agent.water_capacity < 100:
                            
                            update = await self.receive(timeout=15)
                            
                            if update and update.get_metadata("message_type") == "Water Refill Update":
                                new_capacity = float(update.body)
                                print(f"Water capacity updated to {new_capacity}%")
                                self.agent.water_capacity = new_capacity
                    else:
                        print(f"Message received: {response.body}")
                else:
                    print("No response from the water station.")
                
            else:
                print("Water capacity is already at 100%. Stopping refill.")
                await self.agent.stop()

    class EnergyChargeBehaviour(CyclicBehaviour):
        async def run(self):
            if self.agent.energy_capacity < 100:
                # Send energy charge request
                energy_charge_request = Message(to="energy_station@jabbers.one")
                energy_charge_request.body = str(self.agent.energy_capacity)
                energy_charge_request.set_metadata("performative", "request")
                energy_charge_request.set_metadata("message_type", "Energy Recharge Request")

                await self.send(energy_charge_request)
                
                # Wait for response
                response = await self.receive(timeout=10)
                
                if response:
                    if response.get_metadata("response_status") == "accepted":
                        print(f"Message received: {response.body}")
                        print("Waiting for update")

                        while self.agent.energy_capacity < 100:
                            
                            update = await self.receive(timeout=15)
                            
                            if update and update.get_metadata("message_type") == "Energy Recharge Update":
                                new_capacity = float(update.body)
                                print(f"Energy capacity updated to {new_capacity}%")
                                self.agent.energy_capacity = new_capacity
                    else:
                        print(f"Message received: {response.body}")
                else:
                    print("No response from the energy station.")
                
            else:
                print("Energy capacity is already at 100%. Stopping recharge.")
                await self.agent.stop()


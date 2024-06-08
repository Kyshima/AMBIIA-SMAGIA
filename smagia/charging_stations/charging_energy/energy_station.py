from spade.agent import Agent
from spade.behaviour import CyclicBehaviour
from spade.message import Message

class EnergyStationAgent(Agent):
    async def setup(self):
        print(f"Energy Station Agent {self.jid} is running")
        self.isAlreadyRefilling = False
        self.add_behaviour(self.HandleEnergyRequests())

    class HandleEnergyRequests(CyclicBehaviour):
        async def run(self):
            try:
                msg = await self.receive(timeout=10)
                if msg:
                    if msg.get_metadata("message_type") == "Energy Recharge Request":
                        response = Message(to=msg.sender)
                        response.set_metadata("performative", "proposal")
                        response.set_metadata("message_type", "Energy Recharge Response")

                        if not self.agent.isAlreadyRefilling:
                            # Accept the refill request
                            self.agent.isAlreadyRefilling = True
                            response.body = "Energy recharge accepted"
                            response.set_metadata("response_status", "accept proposal")
                        else:
                            # Deny the refill request
                            response.body = "Energy recharge denied: The station is already refilling"
                            response.set_metadata("response_status", "deny proposal")
                        
                        await self.send(response)

            except Exception as e:
                # Error handling
                print(f"An error occurred: {str(e)}")
                error_response = Message(to=msg.sender)
                error_response.set_metadata("performative", "failure")
                error_response.set_metadata("message_type", "Energy Recharge Response")
                error_response.set_metadata("response_status", "error")
                error_response.body = f"An error occurred: {str(e)}"
                await self.send(error_response)
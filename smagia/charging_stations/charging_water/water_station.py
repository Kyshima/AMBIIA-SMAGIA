from spade.agent import Agent
from spade.behaviour import CyclicBehaviour
from spade.message import Message
from utils.utils import jid_to_string


class WaterStationAgent(Agent):
    async def setup(self):
        print(f"Water Station Agent {self.jid} is running")
        self.isAlreadyRefilling = False
        self.add_behaviour(self.HandleWaterRequests())

    class HandleWaterRequests(CyclicBehaviour):
        async def run(self):
            try:
                msg = await self.receive(timeout=10)
                if msg:
                    if msg.get_metadata("message_type") == "Water Recharge Request":
                        response = Message(to=jid_to_string(msg.sender))
                        response.set_metadata("performative", "proposal")
                        response.set_metadata("message_type", "Water Recharge Response")

                        if not self.agent.isAlreadyRefilling:
                            # Accept the refill request
                            self.agent.isAlreadyRefilling = True
                            response.body = "Water recharge accepted"
                            response.set_metadata("response_status", "accepted")
                        else:
                            # Deny the refill request
                            response.body = "Water recharge denied: The station is already refilling"
                            response.set_metadata("response_status", "denied")

                        await self.send(response)       

            except Exception as e:
                # Error handling
                print(f"An error occurred: {str(e)}")
                error_response = Message(to=jid_to_string(msg.sender))
                error_response.set_metadata("performative", "failure")
                error_response.set_metadata("message_type", "Water Recharge Response")
                error_response.set_metadata("response_status", "error")
                error_response.body = f"An error occurred: {str(e)}"
                await self.send(error_response)


def jid_to_string(jid):
    if jid.resource:
        return f"{jid.localpart}@{jid.domain}/{jid.resource}"
    else:
        return f"{jid.localpart}@{jid.domain}"

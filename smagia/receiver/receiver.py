import json
from spade.agent import Agent
from spade.behaviour import CyclicBehaviour
from spade.message import Message

class ReceiverAgent(Agent):
    class ReceivePositionBehaviour(CyclicBehaviour):

        async def run(self):
            msg = await self.receive(timeout=10)
            if msg:
                data = json.loads(msg.body)
                position = data.get("position", {})
                print(f"Received position: {position}")
            else:
                print("No message received within the timeout period")

    async def setup(self):
        print("ReceiverAgent started")
        b = self.ReceivePositionBehaviour()
        self.add_behaviour(b)
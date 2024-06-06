import datetime
import json
from spade.agent import Agent
from spade.behaviour import PeriodicBehaviour
from spade.message import Message

class RobotAgent(Agent):
    class SendPositionBehaviour(PeriodicBehaviour):

        async def run(self):
            self.position['x'] += self.increment['x']
            self.position['y'] += self.increment['y']

            data = {
                "position": self.position
            }

            msg = Message(to=self.get("receiver")) 
            msg.body = json.dumps(data)

            await self.send(msg)
            print(f"Robot {self.id} position: {self.position}")

        async def on_end(self):
            await self.agent.stop()

        async def on_start(self):
            self.position = self.get("position")
            self.increment = self.get("increment")
            self.id = self.get("id")

    async def setup(self):
        print(f"RobotAgent started at {datetime.datetime.now().time()}")
        start_at = datetime.datetime.now() + datetime.timedelta(seconds=1)
        b = self.SendPositionBehaviour(period=1, start_at=start_at)
        self.add_behaviour(b)
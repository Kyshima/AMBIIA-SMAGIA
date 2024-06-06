import datetime

import json
from typing import Optional

from spade.agent import Agent
from spade.behaviour import PeriodicBehaviour
from spade.message import Message


class PeriodicSensorAgent(Agent):
    class InformBehav(PeriodicBehaviour):

        async def run(self):
            self.humidity = self.humidity - self.decrease_amount

            data = {
                "humidity": self.humidity
            }

            msg = Message(to=self.get("receiver_jid"))  # Instantiate the message
            msg.body = json.dumps(data)

            await self.send(msg)
            print(f"Sensor {self.id} humidity: {self.humidity}")

        async def on_end(self):
            await self.agent.stop()

        async def on_start(self):
            self.humidity = self.get("humidity")
            self.decrease_amount = self.get("decrease_amount")
            self.id = self.get("id")

    async def setup(self):
        print(f"PeriodicSenderAgent started at {datetime.datetime.now().time()}")
        start_at = datetime.datetime.now() + datetime.timedelta(seconds=1)
        b = self.InformBehav(period=1, start_at=start_at)
        self.add_behaviour(b)

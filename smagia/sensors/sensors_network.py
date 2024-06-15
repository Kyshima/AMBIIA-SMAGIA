import datetime

import json
from typing import Optional

from spade.agent import Agent
from spade.behaviour import PeriodicBehaviour, CyclicBehaviour
from spade.message import Message
from utils.utils import jid_to_string

log = True


def log_sensor(msg):
    if log:
        print(f"Sensor: {msg}")
class HumiditySensorAgent(Agent):
    class InformBehaviour(PeriodicBehaviour):
        async def run(self):
            self.agent.humidity = self.agent.humidity - self.agent.decrease_amount
            log_sensor(f"Sensor {self.agent.sensor_id} humidity: {self.agent.humidity}")
            if self.agent.humidityThreshold >= self.agent.humidity > 0:
                if not self.agent.taskHanded:
                    data = {
                        "humidity": self.agent.humidity,
                        "x": self.agent.x,
                        "y": self.agent.y,
                    }

                    for robot in self.agent.robots_network:
                        msg = Message(to=robot)
                        msg.set_metadata("performative", "propose")
                        msg.set_metadata("agent", "sensor")
                        msg.set_metadata("type", "watering")
                        msg.body = json.dumps(data)
                        await self.send(msg)

                    log_sensor(f"Sensor {self.agent.sensor_id} HELP!!!")

            elif self.agent.humidity == 0:
                log_sensor(f"Sensor {self.agent.sensor_id}'s Plant has died")
                self.kill()

        async def on_end(self):
            await self.agent.stop()

    class ReceivePositionBehaviour(CyclicBehaviour):

        async def run(self):
            msg = await self.receive(timeout=1000)
            if msg:
                response = json.loads(msg.body)

                match msg.get_metadata("agent"):
                    case "robot":
                        await self.handle_robot_task(response, msg)

                    case _:
                        log_sensor("Message sent by unknown agent")
            else:
                log_sensor("Did not received any message after 1000 seconds")
                self.kill()

        async def on_end(self):
            await self.agent.stop()

        async def handle_robot_task(self, response, msg):
            match msg.get_metadata("type"):
                case "watering":
                    self.agent.humidity = self.agent.humidity + response["water"]

                    data = {
                        "stop": self.agent.humidity >= 100,
                    }

                    if self.agent.humidity >= 100:
                        reply = msg.make_reply()
                        reply.body = json.dumps(data)
                        reply.set_metadata("performative", "cancel")
                        reply.set_metadata("agent", "sensor")
                        reply.set_metadata("type", "stop_watering")
                        await self.send(reply)
                        self.agent.taskHanded = False
                case "give_task":
                    self.agent.taskHanded = True

                    data = {
                        "humidity": self.agent.humidity,
                        "x": self.agent.x,
                        "y": self.agent.y,
                    }

                    reply = msg.make_reply()
                    reply.body = json.dumps(data)
                    reply.set_metadata("performative", "subscribe")
                    reply.set_metadata("agent", "sensor")
                    reply.set_metadata("type", "give_task")
                    await self.send(reply)
                case _:
                    log_sensor("Unknown type from robot agent")

    def __init__(self, jid, password, humidity, decrease_amount, sensor_id, x, y, robots_network):
        super().__init__(jid, password)
        self.sensor_id = sensor_id
        self.decrease_amount = decrease_amount
        self.humidity = humidity
        self.humidityThreshold = 70
        self.x = x
        self.y = y
        self.taskHanded = False
        self.robots_network = robots_network

    async def setup(self):
        log_sensor(f"PeriodicSenderAgent started at {datetime.datetime.now().time()}")
        start_at = datetime.datetime.now() + datetime.timedelta(seconds=2)
        a = self.ReceivePositionBehaviour()
        self.add_behaviour(a)

        b = self.InformBehaviour(period=1, start_at=start_at)
        self.add_behaviour(b)

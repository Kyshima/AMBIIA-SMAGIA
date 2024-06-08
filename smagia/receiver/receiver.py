import json
from spade.agent import Agent
from spade.behaviour import CyclicBehaviour
from spade.message import Message

from utils.utils import get_max_potency_jid, jid_to_string

log = True


def log_controller(msg):
    if log:
        print(f"Controller: {msg}")


class ReceiverAgent(Agent):
    class ReceiverBehaviour(CyclicBehaviour):

        async def run(self):
            msg = await self.receive(timeout=1000)
            if msg:
                response = json.loads(msg.body)
                match msg.get_metadata("agent"):
                    case "robot":
                        await self.handle_robot_task(response, msg)
                    case "sensor":
                        await self.handle_sensor_task(response, msg)
                    case _:
                        log_controller("Message sent by unknown agent")
            else:
                log_controller("Did not received any message after 1000 seconds")
                self.kill()

        async def on_end(self):
            await self.agent.stop()

        async def handle_robot_task(self, response, msg):
            match msg.get_metadata("type"):
                case "subscribe":

                    self.agent.robotsAvailability[msg.sender] = {
                        "potency": response['potency'],
                        "availability": True,
                        "x": response['x'],
                        "y": response['y']
                    }
                    log_controller(self.agent.robotsAvailability)
                case "update":
                    self.agent.robotsAvailability[msg.sender]['availability'] = response['availability']
                    self.agent.robotsAvailability[msg.sender]['x'] = response['x']
                    self.agent.robotsAvailability[msg.sender]['y'] = response['y']

                    log_controller(self.agent.robotsAvailability)
                case _:
                    log_controller("Unknown type from robot agent")

        async def handle_sensor_task(self, response, msg):
            match msg.get_metadata("type"):
                case "watering":
                    robot = get_max_potency_jid(self.agent.robotsAvailability)

                    if robot:
                        robot = jid_to_string(robot)
                        data = {
                            "x": response['x'],
                            "y": response['y'],
                            "sensor_jid": jid_to_string(msg.sender),
                        }

                        msg1 = Message(to=robot)
                        msg1.set_metadata("agent", "controller")
                        msg1.set_metadata("type", "give_task")
                        msg1.body = json.dumps(data)
                        await self.send(msg1)

                        msg1 = Message(to=jid_to_string(msg.sender))
                        msg1.set_metadata("agent", "controller")
                        msg1.set_metadata("type", "task_allocated")
                        msg1.body = json.dumps(data)
                        await self.send(msg1)
                    else:
                        log_controller("No available robots found")
                case _:
                    log_controller("Unknown type from sensor agent")

    def __init__(self, jid, password):
        super().__init__(jid, password)
        self.robotsAvailability = {}

    async def setup(self):
        log_controller("ReceiverAgent started")
        b = self.ReceiverBehaviour()
        self.add_behaviour(b)

import datetime
import json
from spade.agent import Agent
from spade.behaviour import PeriodicBehaviour, CyclicBehaviour, OneShotBehaviour
from spade.message import Message

from utils.utils import jid_to_string

log = True


def log_robots(msg):
    if log:
        print(f"robot: {msg}")


class RobotAgent(Agent):
    class SubscribeBehaviour(OneShotBehaviour):
        async def run(self):
            data = {
                "potency": self.agent.water_potency,
                "availability": True,
                "x": self.agent.x,
                "y": self.agent.y,
            }

            msg = Message(to=self.get("receiver_jid"))
            msg.set_metadata("agent", "robot")
            msg.set_metadata("type", "subscribe")
            msg.body = json.dumps(data)
            await self.send(msg)
            log_robots(f"robot {self.agent.robot_id} subscribed")

    class UpdateInformationBehaviour(PeriodicBehaviour):
        async def run(self):
            data = {
                "availability": self.agent.task == 'resting',
                "x": self.agent.x,
                "y": self.agent.y,
            }

            msg = Message(to=self.get("receiver_jid"))
            msg.set_metadata("agent", "robot")
            msg.set_metadata("type", "update")
            msg.body = json.dumps(data)
            await self.send(msg)
            log_robots(f"robot {self.agent.robot_id} updated")

        async def on_end(self):
            await self.agent.stop()

    class ReceiverBehaviour(CyclicBehaviour):

        async def run(self):
            msg = await self.receive(timeout=1000)
            if msg:
                response = json.loads(msg.body)

                match msg.get_metadata("agent"):
                    case "controller":
                        await self.handle_controller_task(response, msg)

                    case "sensor":
                        await self.handle_sensor_task(response, msg)

                    case _:
                        log_robots("Message sent by unknown agent")
            else:
                log_robots("Did not received any message after 1000 seconds")
                self.kill()

        async def on_end(self):
            await self.agent.stop()

        async def handle_controller_task(self, response, msg):
            match msg.get_metadata("type"):
                case "give_task":
                    self.agent.task_x = response["x"]
                    self.agent.task_y = response["y"]
                    self.agent.task = "watering"
                    self.agent.taskSender = response["sensor_jid"]
                    log_robots(
                        f"Recieved task for x:{self.agent.task_x},y:{self.agent.task_y} from {self.agent.taskSender}")
                case _:
                    log_robots("Unknown type from controller agent")

        async def handle_sensor_task(self, response, msg):
            match msg.get_metadata("type"):
                case "stop_watering":
                    self.agent.task_x = self.agent.base_x
                    self.agent.task_y = self.agent.base_x
                    self.agent.task = "resting"
                    self.agent.taskSender = ""
                case _:
                    log_robots("Unknown type from controller agent")

    class MovementBehaviour(PeriodicBehaviour):
        async def run(self):
            if self.agent.task == "resting":
                if self.agent.x != self.agent.base_x or self.agent.y != self.agent.base_y:
                    self.move_towards_goal(self.agent.base_x, self.agent.base_y)
            else:
                if self.agent.x != self.agent.task_x or self.agent.y != self.agent.task_y:
                    self.move_towards_goal(self.agent.task_x, self.agent.task_y)

        async def on_end(self):
            await self.agent.stop()

        def move_towards_goal(self, x, y):
            if self.agent.x < x:
                self.agent.x += 1
            elif self.agent.x > x:
                self.agent.x -= 1

            if self.agent.y < y:
                self.agent.y += 1
            elif self.agent.y > y:
                self.agent.y -= 1

            log_robots(f"Moved to ({self.agent.x},{self.agent.y})")

    class WaterPlantsBehaviour(PeriodicBehaviour):
        async def run(self):
            if self.agent.task == "watering" and self.agent.x == self.agent.task_x and self.agent.y == self.agent.task_y:
                data = {
                    "water": self.agent.water_potency,
                }
                self.agent.water = self.agent.water - self.agent.water_potency
                msg = Message(to=self.agent.taskSender)
                msg.set_metadata("agent", "robot")
                msg.set_metadata("type", "watering")
                msg.body = json.dumps(data)
                await self.send(msg)
                log_robots(f"robot {self.agent.robot_id} watered plant with sensor {self.agent.taskSender}")

        async def on_end(self):
            await self.agent.stop()

    def __init__(self, jid, password, max_energy, max_water, robot_id, base_x, base_y, water_potency):
        super().__init__(jid, password)
        self.max_energy = max_energy
        self.energy = max_energy
        self.max_water = max_water
        self.water = max_water
        self.robot_id = robot_id
        self.base_x = base_x
        self.base_y = base_y
        self.x = base_x
        self.y = base_y
        self.water_potency = water_potency
        self.task = "resting"
        self.taskSender = ""
        self.task_x = base_x
        self.task_y = base_y

    async def setup(self):
        log_robots(f"RobotAgent started at {datetime.datetime.now().time()}")
        start_at = datetime.datetime.now() + datetime.timedelta(seconds=1)

        a = self.SubscribeBehaviour()
        self.add_behaviour(a)

        b = self.UpdateInformationBehaviour(period=1, start_at=start_at)
        self.add_behaviour(b)

        c = self.ReceiverBehaviour()
        self.add_behaviour(c)

        d = self.MovementBehaviour(period=0.5, start_at=datetime.datetime.now())
        self.add_behaviour(d)

        e = self.WaterPlantsBehaviour(period=0.5, start_at=datetime.datetime.now())
        self.add_behaviour(e)

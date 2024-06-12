import datetime
import json
from spade.agent import Agent
from spade.behaviour import PeriodicBehaviour, CyclicBehaviour, OneShotBehaviour
from spade.message import Message

from utils.utils import jid_to_string, get_max_potency_jid

log = True


def log_robots(msg):
    if log:
        print(f"robot: {msg}")


def get_max_potency_jid_network(robot):
    robots_dict = robot.robots_availability.copy()
    robots_dict[robot.jid] = {
        "potency": robot.water_potency,
        "availability": robot.task == "resting",
        "x": robot.x,
        "y": robot.y
    }
    available_robots = {jid: info for jid, info in robots_dict.items() if info['availability']}

    if not available_robots:
        return None

    max_potency_jid = max(available_robots, key=lambda jid: available_robots[jid]['potency'])
    return max_potency_jid


class RobotAgent(Agent):
    class UpdateNetworkBehaviour(PeriodicBehaviour):
        async def run(self):
            data = {
                "potency": self.agent.water_potency,
                "availability": self.agent.task == 'resting',
                "x": self.agent.x,
                "y": self.agent.y,
            }

            for robot in self.agent.robot_network:
                msg = Message(to=robot)
                msg.set_metadata("performative", "accept-proposal")
                msg.set_metadata("agent", "robot")
                msg.set_metadata("type", "update")
                msg.body = json.dumps(data)
                await self.send(msg)

            log_robots(f"robot network {self.agent.robot_id} updated")

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
                        log_robots("Message sent by unknown agent")
            else:
                log_robots("Did not received any message after 1000 seconds")
                self.kill()

        async def on_end(self):
            await self.agent.stop()

        async def handle_robot_task(self, response, msg):
            match msg.get_metadata("type"):
                case "update":
                    sender = jid_to_string(msg.sender)
                    self.agent.robots_availability[msg.sender] = {
                        "potency": response['potency'],
                        "availability": response['availability'],
                        "x": response['x'],
                        "y": response['y']
                    }

                    log_robots(self.agent.robots_availability)
                case _:
                    log_robots("Unknown type from robot agent")

        async def handle_sensor_task(self, response, msg):
            match msg.get_metadata("type"):
                case "stop_watering":
                    self.agent.task_x = self.agent.base_x
                    self.agent.task_y = self.agent.base_x
                    self.agent.task = "resting"
                    self.agent.taskSender = ""
                case "watering":
                    robot = jid_to_string(get_max_potency_jid_network(self.agent))
                    if robot == jid_to_string(self.agent.jid):
                        data = {
                            "x": response['x'],
                            "y": response['y'],
                            "sensor_jid": jid_to_string(msg.sender),
                        }

                        msg1 = Message(to=jid_to_string(msg.sender))
                        msg.set_metadata("performative", "accept-proposal")
                        msg1.set_metadata("agent", "robot")
                        msg1.set_metadata("type", "give_task")
                        msg1.body = json.dumps(data)
                        await self.send(msg1)
                    else:
                        log_robots("No available robots found")
                case "give_task":
                    self.agent.task_x = response["x"]
                    self.agent.task_y = response["y"]
                    self.agent.task = "watering"
                    self.agent.taskSender = jid_to_string(msg.sender)
                    log_robots(f"Received task for x:{self.agent.task_x},y:{self.agent.task_y} from {self.agent.taskSender}")
                case _:
                    log_robots("Unknown type from sensor agent")

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
                msg.set_metadata("performative", "update")
                msg.set_metadata("agent", "robot")
                msg.set_metadata("type", "watering")
                msg.body = json.dumps(data)
                await self.send(msg)
                log_robots(f"robot {self.agent.robot_id} watered plant with sensor {self.agent.taskSender}")

        async def on_end(self):
            await self.agent.stop()

    def __init__(self, jid, password, max_energy, max_water, robot_id, base_x, base_y, water_potency, robot_network):
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
        self.robot_network = [robot for robot in robot_network if robot != jid_to_string(self.jid)]
        self.robots_availability = {}

    async def setup(self):
        log_robots(f"RobotAgent started at {datetime.datetime.now().time()}")
        start_at = datetime.datetime.now() + datetime.timedelta(seconds=1)

        b = self.UpdateNetworkBehaviour(period=1, start_at=start_at)
        self.add_behaviour(b)

        c = self.ReceiverBehaviour()
        self.add_behaviour(c)

        d = self.MovementBehaviour(period=0.5, start_at=datetime.datetime.now())
        self.add_behaviour(d)

        e = self.WaterPlantsBehaviour(period=0.5, start_at=datetime.datetime.now())
        self.add_behaviour(e)


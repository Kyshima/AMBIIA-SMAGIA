import datetime
import json
from spade.agent import Agent
from spade.behaviour import PeriodicBehaviour, CyclicBehaviour, OneShotBehaviour
from spade.message import Message
import math


#from utils.utils import jid_to_string, get_max_potency_jid

def jid_to_string(jid):
    return f"{jid.localpart}@{jid.domain}"


log = True


def log_robots(msg):
    if log:
        print(f"robot: {msg}")


def get_max_potency_jid_network(robot, task_x, task_y):
    robots_dict = robot.robots_availability.copy()
    robots_dict[robot.jid] = {
        "potency": robot.water_potency,
        "availability": robot.task == "resting",
        "x": robot.x,
        "y": robot.y,
        "water": robot.water,
        "energy": robot.energy,
        "max_energy": robot.max_energy,
        "energy_waste": robot.energy_waste
    }
    available_robots = {jid: info for jid, info in robots_dict.items() if info['availability']}

    filtered_robots = {}
    for jid, info in available_robots.items():
        energy_estimate = get_energy_estimate(info, task_x, task_y)
        print(f"jid: {jid}, energy_estimate: {energy_estimate}")
        if info["max_energy"] - energy_estimate > 0.1 * info["max_energy"]:
            filtered_robots[jid] = info

    if not filtered_robots:
        return None

    max_water = max(info["water"] for info in filtered_robots.values())
    max_energy = max(info["energy"] for info in filtered_robots.values())

    # Find the robot with the highest combined normalized score of water and energy
    max_score_robot = max(
        filtered_robots.items(),
        key=lambda item: (item[1]['water'] / max_water) + (item[1]['energy'] / max_energy)
    )
    print(max_score_robot[0])
    return max_score_robot[0]


def get_energy_estimate(robot, task_x, task_y):
    distance = math.sqrt((task_x - robot["x"]) ** 2 + (task_y - robot["y"]) ** 2)
    energy = distance * robot["energy_waste"] * 2
    return energy


class RobotAgent(Agent):
    class UpdateNetworkBehaviour(PeriodicBehaviour):
        async def run(self):
            data = {
                "potency": self.agent.water_potency,
                "water": self.agent.water,
                "energy": self.agent.energy,
                "availability": self.agent.task == 'resting',
                "x": self.agent.x,
                "y": self.agent.y,
                "max_energy": self.agent.max_energy,
                "energy_waste": self.agent.energy_waste
            }

            for robot in self.agent.robot_network:
                msg = Message(to=robot)
                msg.set_metadata("performative", "inform")
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

                    case "Water Station":
                        await self.handle_water_refill_task(response, msg)

                    case "Energy Station":
                        await self.handle_energy_recharge_task(response, msg)

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
                        "y": response['y'],
                        "water": response['water'],
                        "energy": response['energy'],
                        "max_energy": response['max_energy'],
                        "energy_waste": response['energy_waste'],

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
                    self.agent.add_behaviour(self.agent.RechargeEnergyBehaviour())
                    self.agent.add_behaviour(self.agent.RefillWaterBehaviour())

                case "watering":
                    robot = get_max_potency_jid_network(self.agent, response['x'], response['y'])
                    if robot is None:
                        return
                    robot = jid_to_string(robot)
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
                    log_robots(
                        f"Received task for x:{self.agent.task_x},y:{self.agent.task_y} from {self.agent.taskSender}")
                case _:
                    log_robots("Unknown type from sensor agent")

        async def handle_water_refill_task(self, response, msg):

            match msg.get_metadata("type"):

                case "Water Refill Response":
                    self.agent.task_x = response["water_station_x"]
                    self.agent.task_y = response["water_station_y"]
                    self.agent.task = "going_refill"
                    self.agent.taskSender = jid_to_string(msg.sender)
                    print(f"Received task for x:{self.agent.task_x}, y:{self.agent.task_y} from {self.agent.taskSender}")

                case "Water Refill":
                    self.agent.task = "refilling_water"
                    new_water = response["new_water"]
                    self.agent.water += new_water

                    if self.agent.water < self.agent.max_water:
                        print(f"Refilling. Water at {self.agent.water}")

                    if self.agent.water >= self.agent.max_water:
                        self.agent.water = self.agent.max_water
                        self.agent.task = "resting"
                        self.agent.taskSender = ""
                        self.agent.task_x = self.agent.base_x
                        self.agent.task_y = self.agent.base_x

                        water_station = self.agent.water_station_jid
                        msg = Message(to=water_station)
                        msg.set_metadata("performative", "cancel")
                        msg.set_metadata("type", "Water Refill Finished")
                        msg.set_metadata("agent", "Water Station")

                        await self.send(msg)
                        print(f"Water Refill Complete! Now at max water capacity of {self.agent.max_water}")
                        self.agent.add_behaviour(self.agent.RechargeEnergyBehaviour())

                case "Water Refill Queue":
                    self.agent.task = "waiting_refill"

        async def handle_energy_recharge_task(self, response, msg):

            match msg.get_metadata("type"):

                case "Energy Recharge Response":
                    self.agent.task_x = response["energy_station_x"]
                    self.agent.task_y = response["energy_station_y"]
                    self.agent.task = "going_recharge"
                    self.agent.taskSender = jid_to_string(msg.sender)
                    print(
                        f"Received task for x:{self.agent.task_x}, y:{self.agent.task_y} from {self.agent.taskSender}")

                case "Energy Recharge":
                    self.agent.task = "recharging_energy"
                    new_energy = response["new_energy"]
                    self.agent.energy += new_energy

                    if self.agent.energy < self.agent.max_energy:
                        print(f"Recharging. Energy at {self.agent.energy}")

                    if self.agent.energy >= self.agent.max_energy:
                        self.agent.energy = self.agent.max_energy
                        self.agent.task = "resting"
                        self.agent.taskSender = ""
                        self.agent.task_x = self.agent.base_x
                        self.agent.task_y = self.agent.base_x

                        energy_station = self.agent.energy_station_jid
                        msg = Message(to=energy_station)
                        msg.set_metadata("performative", "cancel")
                        msg.set_metadata("type", "Energy Recharge Finished")
                        msg.set_metadata("agent", "Energy Station")

                        await self.send(msg)
                        print(f"Energy Recharge Complete! Now at max energy capacity of {self.agent.max_energy}")
                        self.agent.add_behaviour(self.agent.RefillWaterBehaviour())

                case "Energy Recharge Queue":
                    self.agent.task = "waiting_recharge"

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

            self.agent.energy -= self.agent.energy_waste
            log_robots(f"Moved to ({self.agent.x},{self.agent.y})")

            self.agent.add_behaviour(self.agent.RechargeEnergyBehaviour())

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

    class RefillWaterBehaviour(OneShotBehaviour):
        async def run(self):
            min_energy = self.agent.max_energy * 0.35
            min_water = self.agent.max_water * 0.35
            if self.agent.task == "resting" and self.agent.energy <= min_energy:
                self.agent.add_behaviour(self.agent.RechargeEnergyBehaviour())
            elif self.agent.task == "resting" and self.agent.water <= min_water:
                water_station = self.agent.water_station_jid
                msg = Message(to=water_station)
                msg.set_metadata("performative", "request")
                msg.set_metadata("type", "Water Refill Request")
                msg.body = json.dumps({
                    "water": self.agent.water
                })

                await self.send(msg)

    class RechargeEnergyBehaviour(OneShotBehaviour):
        async def run(self):
            min_energy = self.agent.max_energy * 0.35
            if self.agent.task == "resting" and self.agent.energy <= min_energy:
                energy_station = self.agent.energy_station_jid
                msg = Message(to=energy_station)
                msg.set_metadata("performative", "request")
                msg.set_metadata("type", "Energy Recharge Request")
                msg.body = json.dumps({
                    "energy": self.agent.energy
                })

                await self.send(msg)

    class UpdatePosBehaviour(PeriodicBehaviour):
        async def run(self):
            if self.agent.task == "going_refill":
                water_station = self.agent.water_station_jid
                msg = Message(to=water_station)
                msg.set_metadata("performative", "inform")
                msg.set_metadata("type", "Robot In Station")
                msg.set_metadata("agent", "Water Station")

                msg.body = json.dumps({

                    "robot_x": self.agent.x,
                    "robot_y": self.agent.y

                })

                await self.send(msg)

            elif self.agent.task == "going_recharge":

                energy_station = self.agent.energy_station_jid
                msg = Message(to=energy_station)
                msg.set_metadata("performative", "inform")
                msg.set_metadata("type", "Robot In Station")
                msg.set_metadata("agent", "Energy Station")

                msg.body = json.dumps({

                    "robot_x": self.agent.x,
                    "robot_y": self.agent.y

                })

                await self.send(msg)

    def __init__(self, jid, password, max_energy, max_water, robot_id, base_x, base_y, water_potency, robot_network,
                 water_station_jid, energy_station_jid, energy_waste):
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
        self.water_station_jid = water_station_jid
        self.energy_station_jid = energy_station_jid
        self.energy_waste = energy_waste

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

        f = self.UpdatePosBehaviour(period=0.5)
        self.add_behaviour(f)

        g = self.RechargeEnergyBehaviour()
        self.add_behaviour(g)

        h = self.RefillWaterBehaviour()
        self.add_behaviour(h)

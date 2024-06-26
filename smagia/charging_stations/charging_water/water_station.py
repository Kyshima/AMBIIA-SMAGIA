import datetime
import json
from spade.agent import Agent
from spade.behaviour import PeriodicBehaviour, CyclicBehaviour
from spade.message import Message
from datetime import datetime

def jid_to_string(jid):

    return f"{jid.localpart}@{jid.domain}"

def get_robot(robots_list):
    best_sender = None
    lowest_water = float('inf')
    longest_waiting_time = datetime.now()

    for sender, data in robots_list.items():
        water = data['water']
        waiting_time = data['waiting_time']

        if water < lowest_water or (water == lowest_water and waiting_time < longest_waiting_time):
            best_sender = sender
            lowest_water = water
            longest_waiting_time = waiting_time

    return best_sender

class WaterStationAgent(Agent):

    class ReceiverBehaviour(CyclicBehaviour):
        async def run(self):
            msg = await self.receive(timeout=1000)
            if msg:

                if msg.body is not None:
                    response = json.loads(msg.body)

                match msg.get_metadata("type"):
                    case "Water Refill Request":
                        sender = jid_to_string(msg.sender)
                        self.agent.robots_list[sender] = {
                            "water": response['water'],
                            "waiting_time" : datetime.now()
                        }

                        print(f"I got a water refill request. This is now my queue {self.agent.robots_list.keys()}")

                    case "Water Refill Finished":
                        sender = jid_to_string(msg.sender)
                        print("The robot " + self.agent.robot_jid + " completed it's water refill")
                        self.agent.robot_jid = None
                        self.agent.robots_list.pop(sender)
                        self.agent.in_pos = False

                    case "Robot In Station":
                        sender = jid_to_string(msg.sender)
                        pos_x = response['robot_x']
                        pos_y = response['robot_y']

                        if (-1 < pos_x - self.agent.station_x < 1
                                and -1 < pos_y - self.agent.station_y < 1):

                            self.agent.in_pos = True

                    case _:
                        print("Unknown ")

            else:
                print("Did not received any message after 1000 seconds")
                self.kill()

    class ResponseBehaviour(PeriodicBehaviour):
        async def run(self):
            if(self.agent.robot_jid == None):

                robot = get_robot(self.agent.robots_list)
                for sender in self.agent.robots_list.keys():
                    print(sender)
                    if robot == sender:
                        response = Message(to=robot)
                        response.set_metadata("performative", "inform")
                        response.set_metadata("type", "Water Refill Response")
                        response.set_metadata("agent", "Water Station")
                        response.body = json.dumps({
                            "water_station_x": self.agent.station_x,
                            "water_station_y": self.agent.station_y
                        })
                        self.agent.robot_jid = robot
                    else:
                        response = Message(to=sender)
                        response.set_metadata("performative", "accept-proposal")
                        response.set_metadata("type", "Water Refill Queue")
                        response.set_metadata("agent", "Water Station")
                        response.body = "You are now in the waiting list"

                    await self.send(response)

    class RefillBehaviour(PeriodicBehaviour):
        async def run(self):
            if(self.agent.robot_jid != None and self.agent.in_pos):

                msg = Message(to=self.agent.robot_jid)
                msg.set_metadata("performative", "update")
                msg.set_metadata("type", "Water Refill")
                msg.set_metadata("agent", "Water Station")
                msg.body = json.dumps({
                    "new_water": self.agent.water
                })
                await self.send(msg)


    def __init__(self, jid, password, water, station_x, station_y):
        super().__init__(jid, password)
        self.robot_jid = None
        self.water = water
        self.station_x = station_x
        self.station_y = station_y
        self.robots_list = {}
        self.in_pos = False

    async def setup(self):

        a = self.ReceiverBehaviour()
        self.add_behaviour(a)

        b = self.ResponseBehaviour(period = 2)
        self.add_behaviour(b)

        c = self.RefillBehaviour(period = 2)
        self.add_behaviour(c)
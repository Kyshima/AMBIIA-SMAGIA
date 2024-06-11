import heapq
from spade.agent import Agent
from spade.behaviour import CyclicBehaviour, OneShotBehaviour
from spade.message import Message
import time

class WaterStationAgent(Agent):
    async def setup(self):
        print(f"Water Station Agent {self.jid} is running")
        self.isAlreadyRefilling = False
        self.refill_queue = []
        self.add_behaviour(self.HandleWaterRequests())

    class HandleWaterRequests(CyclicBehaviour):
        async def run(self):
            msg = await self.receive(timeout=10)
            if msg:
                if msg.get_metadata("message_type") == "Water Refill Request":
                    print("sou eu")
                    print(self.agent.refill_queue)
                    water_capacity = float(msg.body)
                    robot_jid = jid_to_string(msg.sender)
                        
                    # Add the request to the priority queue
                    heapq.heappush(self.agent.refill_queue, (water_capacity, robot_jid))
                    
                    response = Message(to=robot_jid)
                    response.set_metadata("performative", "inform")
                    response.set_metadata("message_type", "Water Refill Response")
                    response.body = "Your request has been received and you are now in the queue."
                    await self.send(response)
                        
                    # Process the next refill request if not already refilling
                    if not self.agent.isAlreadyRefilling:
                        self.agent.add_behaviour(self.agent.ProcessNextRefillRequest())

    class ProcessNextRefillRequest(OneShotBehaviour):
        async def run(self):
            # If there are elements in the queue
            if self.agent.refill_queue:
                print("Olha eu antes")
                print(self.agent.refill_queue)
                water_capacity, robot_jid = heapq.heappop(self.agent.refill_queue)
                print("Olha eu depois")
                print(self.agent.refill_queue)
                response = Message(to=robot_jid)
                response.set_metadata("performative", "proposal")
                response.set_metadata("message_type", "Water Refill Response")

                if not self.agent.isAlreadyRefilling:
                    # Accept the refill request
                    self.agent.isAlreadyRefilling = True
                    response.body = "Water refill accepted"
                    response.set_metadata("response_status", "accepted")
                    await self.send(response)
                    self.agent.add_behaviour(self.agent.RefillRobotBehaviour(robot_jid, water_capacity))
                else:
                    # Deny the refill request
                    response.body = "Water refill denied: The station is already refilling. You are now in queue"
                    response.set_metadata("response_status", "denied")
                    await self.send(response)

    class RefillRobotBehaviour(OneShotBehaviour):
        def __init__(self, robot_jid, water_capacity):
            super().__init__()
            self.robot_jid = robot_jid
            self.water_capacity = water_capacity

        async def run(self):
            water_level = self.water_capacity
            while water_level < 100:
                time.sleep(3)
                water_level += 10
                update_message = Message(to=self.robot_jid)
                update_message.set_metadata("performative", "inform")
                update_message.set_metadata("message_type", "Water Refill Update")
                update_message.body = str(water_level)
                await self.send(update_message)
                print(f"Sent update to {self.robot_jid}: water level {water_level}%")
            
            self.agent.isAlreadyRefilling = False

            print("olha eu simplesmente")
            print(self.agent.refill_queue)
            if self.agent.refill_queue:
                self.agent.add_behaviour(self.agent.ProcessNextRefillRequest())

def jid_to_string(jid):
    if jid.resource:
        return f"{jid.localpart}@{jid.domain}/{jid.resource}"
    else:
        return f"{jid.localpart}@{jid.domain}"

import heapq
from spade.agent import Agent
from spade.behaviour import CyclicBehaviour, OneShotBehaviour
from spade.message import Message
import time

class EnergyStationAgent(Agent):
    async def setup(self):
        print(f"Energy Station Agent {self.jid} is running")
        self.isAlreadyCharging = False
        self.recharge_queue = []
        self.add_behaviour(self.HandleEnergyRequests())

    class HandleEnergyRequests(CyclicBehaviour):
        async def run(self):
            try:
                msg = await self.receive(timeout=10)
                if msg:
                    if msg.get_metadata("message_type") == "Energy Recharge Request":
                        energy_capacity = float(msg.body)
                        robot_jid = jid_to_string(msg.sender)
                        
                        # Add the request to the priority queue
                        heapq.heappush(self.agent.recharge_queue, (energy_capacity, robot_jid))
                        
                        # Process the next recharge request if not already charging
                        if not self.agent.isAlreadyCharging:
                            await self.agent.process_next_recharge_request()
            except Exception as e:
                # Error handling
                print(f"An error occurred: {str(e)}")
                if msg:
                    error_response = Message(to=jid_to_string(msg.sender))
                    error_response.set_metadata("performative", "failure")
                    error_response.set_metadata("message_type", "Energy Recharge Response")
                    error_response.set_metadata("response_status", "error")
                    error_response.body = f"An error occurred: {str(e)}"
                    await self.send(error_response)

    async def process_next_recharge_request(self):
        if self.recharge_queue:
            energy_capacity, robot_jid = heapq.heappop(self.recharge_queue)
            
            response = Message(to=robot_jid)
            response.set_metadata("performative", "proposal")
            response.set_metadata("message_type", "Energy Recharge Response")

            if not self.isAlreadyCharging:
                # Accept the recharge request
                self.isAlreadyCharging = True
                response.body = "Energy recharge accepted"
                response.set_metadata("response_status", "accepted")
                await self.send(response)

                self.add_behaviour(self.RechargeRobotBehaviour(robot_jid, energy_capacity))
            else:
                # Deny the recharge request
                response.body = "Energy recharge denied: The station is already charging. You're in queue"
                response.set_metadata("response_status", "denied")
                await self.send(response)

    class RechargeRobotBehaviour(OneShotBehaviour):
        def __init__(self, robot_jid, energy_capacity):
            super().__init__()
            self.robot_jid = robot_jid
            self.energy_capacity = energy_capacity

        async def run(self):
            energy_level = self.energy_capacity
            while energy_level < 100:
                time.sleep(3)
                energy_level += 10
                update_message = Message(to=self.robot_jid)
                update_message.set_metadata("performative", "inform")
                update_message.set_metadata("message_type", "Energy Recharge Update")
                update_message.body = str(energy_level)
                await self.send(update_message)
                print(f"Sent update to {self.robot_jid}: energy level {energy_level}%")
            
            self.agent.isAlreadyCharging = False
            await self.agent.process_next_recharge_request()

def jid_to_string(jid):
    if jid.resource:
        return f"{jid.localpart}@{jid.domain}/{jid.resource}"
    else:
        return f"{jid.localpart}@{jid.domain}"

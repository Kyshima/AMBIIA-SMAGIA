from robots import RobotAgent
import spade

async def main():
    jid="testesmagia@jabbers.one"
    password="password"
    agent = RobotAgent(jid, password)
    
    agent.set("receiver", "receiver@jabbers.one")
    agent.set("position", {"x": 0, "y": 0})
    agent.set("increment", {"x": 1, "y": 1})
    agent.set("id", "robot_1")

    future = agent.start()
    future.result()
    print("RobotAgent started!")

    import asyncio
    asyncio.run(asyncio.sleep(10))

    agent.stop()

from robots import RobotAgent
import spade

#robot1@jabbers.one robot1
#robot2@jabbers.one robot2
#robot3@jabbers.one robot3
#robot4@jabbers.one robot4

async def main():
    jid="robot1@jabbers.one"
    password="robot1"
    agent = RobotAgent(jid, password)
    
    agent.set("receiver", "receiver@jabbers.one")
    agent.set("position", {"x": 0, "y": 0})
    agent.set("increment", {"x": 1, "y": 1})
    agent.set("id", "robot_1")

    agent.start()
    print("Robot Agent started!")
    agent.web.start(hostname="127.0.0.1", port="1002")

    await spade.wait_until_finished(agent)
    await agent.stop()
    print("Robot Agent finished")

if __name__ == "__main__":
        spade.run(main())
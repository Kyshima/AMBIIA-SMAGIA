from robots_test import RobotAgent
import spade

async def main():
    jid = "robot1@jabbers.one"
    password = "robot1"
    robot_agent = RobotAgent(jid, password)
    
    robot_agent.water_capacity = 30
    
    await robot_agent.start(auto_register=True)
    print("Robot Agent Started")

    robot_agent.web.start(hostname="127.0.0.1", port="1004")

    await spade.wait_until_finished(robot_agent)
    await robot_agent.stop()
    print("Robot Agent Finished")

if __name__ == "__main__":
    spade.run(main())


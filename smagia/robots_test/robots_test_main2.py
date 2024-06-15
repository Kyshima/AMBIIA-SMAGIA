from robots_test import RobotAgent
import spade

async def main():
    jid = "robot2@jabbers.one"
    password = "robot2"
    robot_agent = RobotAgent(jid, password)
    
    robot_agent.water_capacity = 20
    robot_agent.energy_capacity = 20

    
    await robot_agent.start(auto_register=True)
    print("Robot Agent Started")

    robot_agent.web.start(hostname="127.0.0.1", port="1009")

    await spade.wait_until_finished(robot_agent)
    await robot_agent.stop()
    print("Robot Agent Finished")

if __name__ == "__main__":
    spade.run(main())
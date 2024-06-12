import spade

from robots_network import RobotAgent


#receiver@jabbers.one receiver

async def main():
    robot_network = ['robot1@jabbers.one', 'robot2@jabbers.one', 'robot3@jabbers.one', 'robot4@jabbers.one']

    robot4 = RobotAgent("robot4@jabbers.one", "robot4", 300, 300, "robot4", 5, 20, 12, robot_network)
    await robot4.start(auto_register=True)

    robot4.web.start(hostname="127.0.0.1", port="11003")

    await spade.wait_until_finished(robot4)
    await robot4.stop()
    print("Receiver Agent finished")


if __name__ == "__main__":
    spade.run(main())

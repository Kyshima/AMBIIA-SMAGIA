import spade

from robots_network import RobotAgent


#receiver@jabbers.one receiver

async def main():
    robot_network = ['robot1@jabbers.one', 'robot2@jabbers.one', 'robot3@jabbers.one', 'robot4@jabbers.one']

    robot3 = RobotAgent("robot3@jabbers.one", "robot3", 300, 300, "robot3", 10, 0, 2, robot_network
                        , 'water_station@jabbers.one','energy_station@jabbers.one', 5)
    await robot3.start(auto_register=True)

    robot3.web.start(hostname="127.0.0.1", port="11002")

    await spade.wait_until_finished(robot3)
    await robot3.stop()
    print("Receiver Agent finished")


if __name__ == "__main__":
    spade.run(main())

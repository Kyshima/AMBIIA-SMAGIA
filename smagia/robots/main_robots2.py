import spade

from robots_mqtt import RobotAgent


#receiver@jabbers.one receiver

async def main():
    robot_network = ['robot1@jabbers.one', 'robot2@jabbers.one', 'robot3@jabbers.one', 'robot4@jabbers.one']

    robot2 = RobotAgent("robot2@jabbers.one", "robot2", 300, 300, "robot2", 20, 5, 4, robot_network
                        , 'water_station@jabbers.one','energy_station@jabbers.one', 1.5, 2)
    await robot2.start(auto_register=True)

    robot2.web.start(hostname="127.0.0.1", port="11001")

    await spade.wait_until_finished(robot2)
    await robot2.stop()
    print("Receiver Agent finished")


if __name__ == "__main__":
    spade.run(main())

import spade
from robots_mqtt import RobotAgent


#robot1@jabbers.one robot1
#robot2@jabbers.one robot2
#robot3@jabbers.one robot3
#robot4@jabbers.one robot4


async def main():
    robot_network = ['robot1@jabbers.one', 'robot2@jabbers.one', 'robot3@jabbers.one', 'robot4@jabbers.one']

    robot1 = RobotAgent("robot1@jabbers.one", "robot1", 500, 150, "robot1", -8.81, 4.10, 10, robot_network
                        , 'water_station@jabbers.one', 'energy_station@jabbers.one', 2, "1")
    await robot1.start(auto_register=True)

    robot1.web.start(hostname="127.0.0.1", port="11000")

    await spade.wait_until_finished(robot1)
    await robot1.stop()
    print("Receiver Agent finished")


if __name__ == "__main__":
    spade.run(main())

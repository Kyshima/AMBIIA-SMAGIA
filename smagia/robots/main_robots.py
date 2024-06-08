from robots import RobotAgent
import spade


#robot1@jabbers.one robot1
#robot2@jabbers.one robot2
#robot3@jabbers.one robot3
#robot4@jabbers.one robot4

async def main():
    robot1 = RobotAgent("robot1@jabbers.one", "robot1", 500, 500, "robot1", 1, 1, 10)
    robot1.set("receiver_jid", "receiver@jabbers.one")
    await robot1.start(auto_register=True)

    robot2 = RobotAgent("robot2@jabbers.one", "robot2", 300, 300, "robot2", 9, 9, 8)
    robot2.set("receiver_jid", "receiver@jabbers.one")
    await robot2.start(auto_register=True)

    robot1.web.start(hostname="127.0.0.1", port="10002")
    robot2.web.start(hostname="127.0.0.1", port="10003")

    await spade.wait_until_finished(robot1)
    await robot2.stop()
    print("Receiver Agent finished")


if __name__ == "__main__":
    spade.run(main())

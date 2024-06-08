from receiver import ReceiverAgent
import spade



#receiver@jabbers.one receiver

async def main():
    receiver = ReceiverAgent("receiver@jabbers.one", "receiver")
    await receiver.start()
    print("Receiver Agent started!")

    receiver.web.start(hostname="127.0.0.1", port="10004")

    await spade.wait_until_finished(receiver)
    await receiver.stop()
    print("Receiver Agent finished")


if __name__ == "__main__":
    spade.run(main())

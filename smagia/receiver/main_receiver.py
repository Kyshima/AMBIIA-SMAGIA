from receiver import ReceiverAgent
import spade

#receiver@jabbers.one receiver

async def main():
    jid = "receiver@jabbers.one"
    password = "receiver"
    receiver = ReceiverAgent(jid, password)

    await receiver.start()
    print("Receiver Agent started!")
    receiver.web.start(hostname="127.0.0.1", port="1001")

    await spade.wait_until_finished(receiver)
    await receiver.stop()
    print("Receiver Agent finished")

if __name__ == "__main__":
    spade.run(main())
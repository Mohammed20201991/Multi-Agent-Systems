import logging
import mapc2020
import asyncio

logging.basicConfig(level=logging.DEBUG)

async def main():
    transport, protocol = await asyncio.get_running_loop().create_connection(
        lambda: mapc2020.AgentProtocol(user="agentA1", pw="1"),
        "127.0.0.1", 12300)

    await protocol.send_action("move", ["n"])

    await protocol.disconnected.wait()

asyncio.run(main())

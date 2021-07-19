"""A client for the 2020/21 edition of the Multi-Agent Programming Contest."""

from __future__ import annotations

import asyncio
import logging
import json

__version__ = "0.1.0"  # Remember to update setup.py

LOGGER = logging.getLogger(__name__)

class AgentException(RuntimeError):
    """Runtime error caused by misbehaving agent or simulation server."""

class AgentAuthFailed(AgentException):
    """Server rejected agent credentials."""

class AgentProtocol(asyncio.Protocol):
    def __init__(self, user: str, pw: str):
        self.user = user
        self.pw = pw

        self.transport = None
        self.disconnected = asyncio.Event()
        self.buffer = bytearray()
        self.fatal = None

    def connection_made(self, transport):
        self.transport = transport
        self.buffer.clear()
        self.disconnected.clear()
        LOGGER.info("%s: Connection made", self)

        self.send_message({
            "type": "auth-request",
            "content": {
                "user": self.user,
                "pw": self.pw,
            },
        })

    def send_message(self, message):
        LOGGER.debug("%s: << %s", self, message)
        self.transport.write(json.dumps(message).encode("utf-8") + b"\0")

    def connection_lost(self, exc):
        LOGGER.info("%s: Connection lost (error: %s)", self, exc)
        self.disconnected.set()

    def data_received(self, data):
        self.buffer.extend(data)
        while b"\0" in self.buffer:
            message_bytes, self.buffer = self.buffer.split(b"\0", 1)
            message = json.loads(message_bytes)
            LOGGER.debug("%s: >> %s", self, message)
            self.message_received(message)

    def message_received(self, message):
        if message["type"] == "auth-response":
            self.handle_auth_response(message["content"])
        else:
            LOGGER.warning("%s: Unknown message type: %s", self, message["type"])

    def handle_auth_response(self, response):
        if response["result"] != "ok":
            self.fatal = AgentAuthFailed()

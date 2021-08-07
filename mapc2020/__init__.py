"""A client for the 2020/21 edition of the Multi-Agent Programming Contest."""

from __future__ import annotations

import asyncio
import contextlib
import concurrent.futures
import logging
import json
import threading
import xml.etree.ElementTree as ET

from types import TracebackType
from typing import Any, Dict, List, Union, Generator, Type, Optional, Tuple, TypeVar, Coroutine, Callable

__version__ = "0.1.0"  # Remember to update setup.py

DirectionLiteral = str
RotationLiteral = str
T = TypeVar("T")

LOGGER = logging.getLogger(__name__)

TIMEOUT = 15

class AgentError(RuntimeError):
    """Runtime error caused by misbehaving agent or simulation server."""

class AgentTerminatedError(AgentError):
    """The engine connection was terminated unexpectedly."""

class AgentAuthError(AgentError):
    """Server rejected agent credentials."""

class AgentActionError(AgentError):
    """Agent action failed."""

class AgentProtocol(asyncio.Protocol):
    def __init__(self, user: str, pw: str):
        self.loop = asyncio.get_running_loop()

        self.user = user
        self.pw = pw

        self.transport: Optional[asyncio.BaseTransport] = None
        self.buffer = bytearray()

        self.state = None
        self.action_lock = asyncio.Lock()
        self.action_requested = asyncio.Event()
        self.disconnected = asyncio.Event()

    def connection_made(self, transport: asyncio.BaseTransport) -> None:
        self.transport = transport
        self.buffer.clear()

        self.static: asyncio.Future[Any] = asyncio.Future()
        self.dynamic: asyncio.Future[Any] = asyncio.Future()
        self.finished: asyncio.Future[Any] = asyncio.Future()

        self.state = None
        self.action_requested.clear()
        self.disconnected.clear()

        LOGGER.info("%s: Connection made", self)

        self.send_message({
            "type": "auth-request",
            "content": {
                "user": self.user,
                "pw": self.pw,
            },
        })

    def send_message(self, message: Any) -> None:
        LOGGER.debug("%s: << %s", self, message)
        self.transport.write(json.dumps(message).encode("utf-8") + b"\0")

    def connection_lost(self, exc):
        LOGGER.info("%s: Connection lost (error: %s)", self, exc)
        exc = exc or AgentTerminatedError()
        if not self.static.done():
            self.static.set_exception(exc)
        if not self.finished.done():
            self.finished.set_exception(exc)
        self.disconnected.set()

    def data_received(self, data):
        self.buffer.extend(data)
        while b"\0" in self.buffer:
            message_bytes, self.buffer = self.buffer.split(b"\0", 1)
            message = json.loads(message_bytes)
            LOGGER.debug("%s: >> %s", self, message)
            self.message_received(message)

    def message_received(self, message: Any) -> None:
        if message["type"] == "auth-response":
            self.handle_auth_response(message["content"])
        elif message["type"] == "sim-start":
            self.handle_sim_start(message["content"])
        elif message["type"] == "sim-end":
            self.handle_sim_end(message["content"])
        elif message["type"] == "request-action":
            self.handle_request_action(message["content"])
        elif message["type"] == "bye":
            self.handle_bye(message["content"])
        else:
            LOGGER.warning("%s: Unknown message type: %s", self, message["type"])

    def handle_auth_response(self, content: Any) -> None:
        if content["result"] != "ok":
            exc = AgentAuthError()
            self.static.set_exception(exc)
            self.finished.set_exception(exc)

    def handle_sim_start(self, content: Any) -> None:
        self.static.set_result(content["percept"])

    def handle_sim_end(self, content):
        self.finished.set_result(content)

    def handle_request_action(self, content):
        if self.dynamic.done():
            self.dynamic = asyncio.Future()
        self.dynamic.set_result(content["percept"])

        self.state = content
        self.action_requested.set()

    def handle_bye(self, _content):
        if not self.finished.done():
            self.finished.set_result(None)

    async def initialize(self):
        await self.static
        await self.dynamic

    async def send_action(self, tpe: str, params: List[Any]) -> AgentProtocol:
        async with self.action_lock:
            await self.static
            await asyncio.wait_for(self.action_requested.wait(), TIMEOUT)

            self.action_requested.clear()
            self.send_message({
                "type": "action",
                "content": {
                    "id": self.state["id"],
                    "type": tpe,
                    "p": params,
                }
            })
            await asyncio.wait_for(self.action_requested.wait(), TIMEOUT)

            if self.state["percept"]["lastAction"] != tpe:
                self.action_requested.clear()
                self.send_message({
                    "type": "action",
                    "content": {
                        "id": self.state["id"],
                        "type": tpe,
                        "p": params,
                    }
                })
                await asyncio.wait_for(self.action_requested.wait(), TIMEOUT)

            if self.state["percept"]["lastActionResult"] != "success":
                raise AgentActionError(self.state["percept"]["lastActionResult"])

            return self

    async def skip(self) -> AgentProtocol:
        return await self.send_action("skip", [])

    async def move(self, direction):
        assert direction in ["n", "s", "e", "w"]
        return await self.send_action("move", [direction])

    async def attach(self, direction):
        assert direction in ["n", "s", "e", "w"]
        return await self.send_action("attach", [direction])

    async def detach(self, direction):
        assert direction in ["n", "s", "e", "w"]
        return await self.send_action("detach", [direction])

    async def rotate(self, rotation):
        assert rotation in ["cw", "ccw"]
        return await self.send_action("rotate", [rotation])

    async def connect(self, agent, pos):
        x, y = pos
        return await self.send_action("connect", [agent, x, y])

    async def disconnect(self, pos1, pos2):
        x1, y1 = pos1
        x2, y2 = pos2
        return await self.send_action("disconnect", [x1, y1, x2, y2])

    async def request(self, direction):
        assert direction in ["n", "s", "e", "w"]
        return await self.send_action("request", [direction])

    async def submit(self, task):
        return await self.send_action("submit", [task])

    async def clear(self, pos):
        x, y = pos
        return await self.send_action("clear", [x, y])

    async def accept(self, task):
        return await self.send_action("accept", [task])

    def __repr__(self):
        return f"<AgentProtocol at {id(self):#x} (user={self.user!r})>"

    def _repr_svg_(self) -> str:
        vision = self.static.result()["vision"]
        dynamic = self.dynamic.result()

        svg = ET.Element("svg", _attrs({
            "xmlns": "http://www.w3.org/2000/svg",
            "version": "1.2",
            "baseProfile": "tiny",
            "viewBox": f"{-vision} {-vision} {2 * vision + 1} {2 * vision + 1}",
            "width": vision * 40,
            "height": vision * 40,
        }))

        # Fog of war.
        for x in range(-vision, vision + 1):
            for y in range(-vision, vision + 1):
                if abs(x) + abs(y) > vision:
                    ET.SubElement(svg, "rect", _attrs({
                        "x": x,
                        "y": y,
                        "width": 1,
                        "height": 1,
                        "fill": "black",
                        "opacity": "0.3",
                    }))

        # Terrain.
        for x, y in dynamic["terrain"].get("goal", []):
            ET.SubElement(svg, "rect", _attrs({
                "x": x,
                "y": y,
                "width": 1,
                "height": 1,
                "fill": "red",
                "opacity": 0.4,
            }))
        for x, y in dynamic["terrain"].get("obstacle", []):
            ET.SubElement(svg, "rect", _attrs({
                "x": x,
                "y": y,
                "width": 1,
                "height": 1,
                "fill": "#333",
            }))

        # Agent itself.
        draw_entity(svg, 0, 0, "A")

        # Things.
        for thing in dynamic["things"]:
            x = thing["x"]
            y = thing["y"]
            if thing["type"] == "entity":
                draw_entity(svg, x, y, thing["details"])
            elif thing["type"] == "taskboard":
                draw_block(svg, x, y, color = "#00ffff")
            elif thing["type"] == "dispenser":
                draw_block(svg, x, y, color = "#41470b")

        return ET.tostring(svg).decode("utf-8")

def draw_entity(svg, x, y, details):
    ET.SubElement(svg, "line", _attrs({
        "x1": x + 0.5,
        "y1": y + 0,
        "x2": x + 0.5,
        "y2": y + 1,
        "stroke": "black",
        "stroke-width": 0.2,
    }))
    ET.SubElement(svg, "line", _attrs({
        "x1": x + 0,
        "y1": y + 0.5,
        "x2": x + 1,
        "y2": y + 0.5,
        "stroke": "black",
        "stroke-width": 0.2,
    }))

def draw_block(svg, x, y, *, color):
    ET.SubElement(svg, "rect", _attrs({
        "x": x,
        "y": y,
        "width": 1,
        "height": 1,
        "fill": color,
    }))

def _attrs(attrs: Dict[str, Union[str, int, float, None]]) -> Dict[str, str]:
    return {k: str(v) for k, v in attrs.items() if v is not None}

def run_in_background(coroutine: Callable[[concurrent.futures.Future[T]], Coroutine[Any, Any, None]]) -> T:
    """
    Runs ``coroutine(future)`` in a new event loop on a background thread.

    Blocks on *future* and returns the result as soon as it is resolved.
    The coroutine and all remaining tasks continue running in the background
    until complete.
    """
    assert asyncio.iscoroutinefunction(coroutine)

    future: concurrent.futures.Future[T] = concurrent.futures.Future()

    def background() -> None:
        try:
            asyncio.run(coroutine(future))
            future.cancel()
        except Exception as exc:
            future.set_exception(exc)

    threading.Thread(target=background).start()
    return future.result()

class Agent:
    """
    Synchronous wrapper around a transport and agent protocol pair.

    Automatically closes the transport when used as a context manager.
    """

    def __init__(self, transport: asyncio.BaseTransport, protocol: AgentProtocol):
        self.transport = transport
        self.protocol = protocol

        self._shutdown_lock = threading.Lock()
        self._shutdown = False
        self.shutdown_event = asyncio.Event()

    @contextlib.contextmanager
    def _not_shut_down(self) -> Generator[None, None, None]:
        with self._shutdown_lock:
            if self._shutdown:
                raise AgentTerminatedError("agent event loop dead")
            yield

    @property
    def dynamic(self):
        """
        Most recent percept.

        See https://github.com/agentcontest/massim_2020/blob/master/docs/scenario.md#step-percept.
        """
        async def _get():
            return await self.protocol.dynamic

        with self._not_shut_down():
            future = asyncio.run_coroutine_threadsafe(_get(), self.protocol.loop)
        return future.result()

    @property
    def static(self):
        """
        Initial percept.

        See https://github.com/agentcontest/massim_2020/blob/master/docs/scenario.md#initial-percept.
        """
        async def _get():
            return await self.protocol.static

        with self._not_shut_down():
            future = asyncio.run_coroutine_threadsafe(_get(), self.protocol.loop)
        return future.result()

    def send_message(self, msg: Any) -> None:
        """
        Low-level method to send an arbitrary message.

        >>> agent.send_message({
        ...     "hello": "world",
        >>> })
        """
        async def _send():
            self.protocol.send_message(msg)

        with self._not_shut_down():
            future = asyncio.run_coroutine_threadsafe(_send(), self.protocol.loop)
        return future.result()

    def skip(self) -> Agent:
        """
        Skip this turn by doing nothing.

        >>> agent.skip()

        See https://github.com/agentcontest/massim_2020/blob/master/docs/scenario.md#skip
        for details.
        """
        with self._not_shut_down():
            coro = self.protocol.skip()
            future = asyncio.run_coroutine_threadsafe(coro, self.protocol.loop)
        future.result()
        return self

    def move(self, direction: DirectionLiteral) -> Agent:
        """
        Move in the specified direction.

        >>> agent.move("n")

        See https://github.com/agentcontest/massim_2020/blob/master/docs/scenario.md#move
        for details.
        """
        with self._not_shut_down():
            coro = self.protocol.move(direction)
            future = asyncio.run_coroutine_threadsafe(coro, self.protocol.loop)
        future.result()
        return self

    def attach(self, direction: DirectionLiteral) -> Agent:
        """
        Attach a thing to the agent.

        See https://github.com/agentcontest/massim_2020/blob/master/docs/scenario.md#attach.
        """
        with self._not_shut_down():
            coro = self.protocol.attach(direction)
            future = asyncio.run_coroutine_threadsafe(coro, self.protocol.loop)
        future.result()
        return self

    def detach(self, direction: DirectionLiteral) -> Agent:
        """
        Detach a thing from the agent.

        See https://github.com/agentcontest/massim_2020/blob/master/docs/scenario.md#detach.
        """
        with self._not_shut_down():
            coro = self.protocol.detach(direction)
            future = asyncio.run_coroutine_threadsafe(coro, self.protocol.loop)
        future.result()
        return self

    def rotate(self, rotation: RotationLiteral) -> Agent:
        """
        Rotate the agent.

        >>> agent.rotate("cw")

        See https://github.com/agentcontest/massim_2020/blob/master/docs/scenario.md#rotate.
        """
        with self._not_shut_down():
            coro = self.protocol.rotate(rotation)
            future = asyncio.run_coroutine_threadsafe(coro, self.protocol.loop)
        future.result()
        return self

    def connect(self, agent: str, pos: Tuple[int, int]) -> Agent:
        """
        Connect things with a cooperating agent.

        See https://github.com/agentcontest/massim_2020/blob/master/docs/scenario.md#connect.
        """
        with self._not_shut_down():
            coro = self.protocol.connect(agent, pos)
            future = asyncio.run_coroutine_threadsafe(coro, self.protocol.loop)
        future.result()
        return self

    def disconnect(self, pos1: Tuple[int, int], pos2: Tuple[int, int]) -> Agent:
        """
        Disconnect two attachments.

        See https://github.com/agentcontest/massim_2020/blob/master/docs/scenario.md#disconnect.
        """
        with self._not_shut_down():
            coro = self.protocol.disconnect(pos1, pos2)
            future = asyncio.run_coroutine_threadsafe(coro, self.protocol.loop)
        future.result()
        return self

    def request(self, direction: DirectionLiteral) -> Agent:
        """
        Request a new block from a dispenser.

        See https://github.com/agentcontest/massim_2020/blob/master/docs/scenario.md#request.
        """
        with self._not_shut_down():
            coro = self.protocol.request(direction)
            future = asyncio.run_coroutine_threadsafe(coro, self.protocol.loop)
        future.result()
        return self

    def submit(self, task: str) -> Agent:
        """
        Submit the pattern of things that are attached to the agent to complete
        a task.

        See https://github.com/agentcontest/massim_2020/blob/master/docs/scenario.md#submit.
        """
        with self._not_shut_down():
            coro = self.protocol.submit(task)
            future = asyncio.run_coroutine_threadsafe(coro, self.protocol.loop)
        future.result()
        return self

    def clear(self, pos: Tuple[int, int]) -> Agent:
        """
        Prepare to clear an area.

        >>> agent.clear((1,2))

        See https://github.com/agentcontest/massim_2020/blob/master/docs/scenario.md#clear.
        """
        with self._not_shut_down():
            coro = self.protocol.clear(pos)
            future = asyncio.run_coroutine_threadsafe(coro, self.protocol.loop)
        future.result()
        return self

    def accept(self, task: str) -> Agent:
        """
        Accept a task.

        https://github.com/agentcontest/massim_2020/blob/master/docs/scenario.md#accept.
        """
        with self._not_shut_down():
            coro = self.protocol.accept(task)
            future = asyncio.run_coroutine_threadsafe(coro, self.protocol.loop)
        future.result()
        return self

    def _repr_svg_(self) -> str:
        async def _get() -> str:
            return self.protocol._repr_svg_()

        with self._not_shut_down():
            future = asyncio.run_coroutine_threadsafe(_get(), self.protocol.loop)
        return future.result()

    def close(self) -> None:
        """
        Closes the transport and background event loop as soon as possible.
        """
        def _shutdown() -> None:
            self.transport.close()
            self.shutdown_event.set()

        with self._shutdown_lock:
            if not self._shutdown:
                self._shutdown = True
                self.protocol.loop.call_soon_threadsafe(_shutdown)

    @classmethod
    def open(cls, user: str, pw: str, *, host: str = "127.0.0.1", port: int = 12300) -> Agent:
        """
        Opens and initializes an agent connection.
        """
        async def _main(future: concurrent.futures.Future[Agent]) -> None:
            transport, protocol = await asyncio.wait_for(
                asyncio.get_running_loop().create_connection(lambda: AgentProtocol(user, pw), host, port),
                TIMEOUT)
            agent = cls(transport, protocol)
            try:
                await protocol.initialize()
                future.set_result(agent)
                await protocol.disconnected.wait()
            finally:
                agent.close()
            await agent.shutdown_event.wait()

        return run_in_background(_main)

    def __enter__(self) -> Agent:
        return self

    def __exit__(self, exc_type: Optional[Type[BaseException]], exc_value: Optional[BaseException], traceback: Optional[TracebackType]) -> None:
        self.close()

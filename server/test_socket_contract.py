import asyncio
import ast
from pathlib import Path

import auth
import main
from routes import system
from socket_contract import (
    SOCKET_CLIENT_EVENTS,
    SOCKET_SERVER_EVENTS,
    build_asyncapi_document,
)

# Resolve source paths relative to this test file so the suite passes
# regardless of the pytest invocation cwd (repo root or server/).
_SERVER = Path(__file__).resolve().parent


def setup_function():
    auth.reset_for_tests()


def test_asyncapi_route_is_registered():
    paths = {f"/api{getattr(route, 'path', '')}" for route in system.router.routes}

    assert "/api/docs/asyncapi" in paths


def test_asyncapi_route_exposes_documented_socket_events():
    payload = asyncio.run(system.asyncapi_doc())

    assert payload["asyncapi"] == "2.6.0"
    channels = set(payload["channels"])
    for event in SOCKET_CLIENT_EVENTS:
        assert f"socket.io/{event}" in channels
        assert "publish" in payload["channels"][f"socket.io/{event}"]
    for event in SOCKET_SERVER_EVENTS:
        assert f"socket.io/{event}" in channels
        assert "subscribe" in payload["channels"][f"socket.io/{event}"]


def test_socket_contract_matches_registered_client_event_names():
    tree = ast.parse((_SERVER / "sockets/events.py").read_text(encoding="utf-8"))
    registered = {"connect", "disconnect"}
    for node in ast.walk(tree):
        if not isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef)):
            continue
        for decorator in node.decorator_list:
            if (
                isinstance(decorator, ast.Call)
                and isinstance(decorator.func, ast.Attribute)
                and decorator.func.attr == "on"
                and decorator.args
                and isinstance(decorator.args[0], ast.Constant)
            ):
                registered.add(decorator.args[0].value)

    assert set(SOCKET_CLIENT_EVENTS) == registered


def test_socket_contract_covers_current_server_emit_literals():
    files = [
        _SERVER / "sockets/events.py",
        _SERVER / "routes/auth.py",
        _SERVER / "main.py",
        _SERVER / "bridge_health.py",
    ]
    emitted = set()
    for path in files:
        tree = ast.parse(path.read_text(encoding="utf-8"))
        for node in ast.walk(tree):
            if not isinstance(node, ast.Call) or not node.args:
                continue
            func = node.func
            if isinstance(func, ast.Attribute) and func.attr in {
                "emit",
                "_safe_emit",
                "_emit_authenticated",
            }:
                first = node.args[0]
                if isinstance(first, ast.Constant) and isinstance(first.value, str):
                    emitted.add(first.value)
            elif isinstance(func, ast.Name) and func.id == "_emit_authenticated":
                first = node.args[0]
                if isinstance(first, ast.Constant) and isinstance(first.value, str):
                    emitted.add(first.value)
    emitted.update(
        {
            "mission_completed",
            "mission_completion_degraded",
            "point_mission_event",
            "target_event",
        }
    )

    assert emitted <= set(SOCKET_SERVER_EVENTS)
    assert set(SOCKET_SERVER_EVENTS) - emitted == set()


def test_build_asyncapi_document_returns_defensive_copy():
    first = build_asyncapi_document()
    first["channels"]["socket.io/arm"]["publish"]["message"]["payload"]["mutated"] = True

    second = build_asyncapi_document()

    assert "mutated" not in second["channels"]["socket.io/arm"]["publish"]["message"]["payload"]

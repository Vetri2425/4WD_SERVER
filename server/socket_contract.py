"""Documented Socket.IO contract for the rover GCS."""

from __future__ import annotations

from copy import deepcopy

SOCKET_CLIENT_EVENTS = {
    "connect": {
        "summary": "Authenticate an operator session for this Socket.IO SID.",
        "payload": {"type": "object", "properties": {"token": {"type": "string"}}},
    },
    "disconnect": {
        "summary": "Socket.IO disconnect lifecycle event; releases joystick ownership.",
        "payload": {"type": "null"},
    },
    "arm": {
        "summary": "Arm or disarm the FCU. Disarm uses spray-safe shutdown.",
        "payload": {"type": "object", "properties": {"arm": {"type": "boolean"}}},
    },
    "set_mode": {
        "summary": "Set a non-OFFBOARD vehicle mode.",
        "payload": {"type": "object", "properties": {"mode": {"type": "string"}}},
    },
    "emergency_stop": {
        "summary": "Run the established e-stop path.",
        "payload": {"type": "object", "additionalProperties": True},
    },
    "joystick_acquire": {
        "summary": "Acquire a virtual joystick lease.",
        "payload": {"$ref": "#/components/schemas/JoystickAcquire"},
    },
    "joystick_command": {
        "summary": "Send a virtual joystick command under an active lease.",
        "payload": {"$ref": "#/components/schemas/JoystickCommand"},
    },
    "joystick_release": {
        "summary": "Release a virtual joystick lease.",
        "payload": {"$ref": "#/components/schemas/JoystickRelease"},
    },
    "mission_load": {
        "summary": "Load a mission path into the controller.",
        "payload": {"$ref": "#/components/schemas/MissionLoad"},
    },
    "mission_start": {
        "summary": "Start the resident or named mission.",
        "payload": {"$ref": "#/components/schemas/MissionStart"},
    },
    "mission_stop": {
        "summary": "Stop the mission without hard abort.",
        "payload": {"type": "object", "additionalProperties": True},
    },
    "mission_abort": {
        "summary": "Abort the mission with terminal cleanup.",
        "payload": {"type": "object", "additionalProperties": True},
    },
    "mission_pause": {
        "summary": "Pause a point mission into OFFBOARD hold.",
        "payload": {"type": "object", "additionalProperties": True},
    },
    "mission_resume": {
        "summary": "Resume a paused point mission.",
        "payload": {"type": "object", "properties": {"expected_generation": {"type": "integer"}}},
    },
    "point_continue": {
        "summary": "Advance a manual point mission after operator approval.",
        "payload": {"type": "object", "additionalProperties": True},
    },
    "mission_obstacle": {
        "summary": "Set the point-mode obstacle-clear hook.",
        "payload": {"type": "object", "properties": {"clear": {"type": "boolean"}}},
    },
    "point_skip": {
        "summary": "Skip the active or requested point leg.",
        "payload": {"$ref": "#/components/schemas/PointSkip"},
    },
    "mission_restart": {
        "summary": "Reset a resident mission and optionally start it again.",
        "payload": {"$ref": "#/components/schemas/MissionRestart"},
    },
    "request_params": {
        "summary": "Read selected PX4 parameters through MAVROS.",
        "payload": {
            "type": "object",
            "properties": {"names": {"type": "array", "items": {"type": "string"}}},
        },
    },
}

SOCKET_SERVER_EVENTS = {
    "arm_result": "Result of an arm/disarm request.",
    "auth_revoked": "Operator session was revoked by password rotation.",
    "bridge_health": "MAVROS bridge health transition or observe-only alert.",
    "bridge_recovery": "MAVROS bridge recovery attempt metadata.",
    "estop_result": "Result of an emergency stop request.",
    "gps_safety_abort": "GPS_SURVEYED runtime safety abort notification.",
    "joystick_acquired": "Virtual joystick lease acquisition result.",
    "joystick_error": "Virtual joystick rejection or runtime error.",
    "joystick_released": "Virtual joystick lease release result.",
    "mission_abort_result": "Mission abort result.",
    "mission_completed": "Continuous/dash mission completed cleanly.",
    "mission_completion_degraded": "Continuous/dash completion cleanup degraded.",
    "mission_error": "Mission command rejection or service error.",
    "mission_loaded": "Mission load result.",
    "mission_obstacle_result": "Obstacle-clear update result.",
    "mission_pause_result": "Point mission pause result.",
    "mission_restart_result": "Mission restart result.",
    "mission_resume_result": "Point mission resume result.",
    "mission_status": "Periodic mission status snapshot.",
    "mission_status_update": "Command-triggered mission state update.",
    "mission_stop_result": "Mission stop result.",
    "mode_result": "Result of a non-OFFBOARD mode request.",
    "params_result": "Parameter values from request_params.",
    "point_continue_result": "Manual point continue result.",
    "point_mission_event": "Bounded point mission event journal append.",
    "target_event": "Verified GPS original-target lifecycle event (terminal via mission_outcome).",
    "point_skip_result": "Point skip result.",
    "rover_disconnected": "FCU/MAVROS disconnected transition.",
    "safety_abort": "Telemetry watchdog safety abort notification.",
    "socket_error": "Socket command rejected before authentication.",
    "telemetry": "Periodic telemetry snapshot.",
}


def _message(name: str, summary: str, payload: dict) -> dict:
    return {
        "name": name,
        "title": name,
        "summary": summary,
        "payload": deepcopy(payload),
    }


def build_asyncapi_document() -> dict:
    channels = {}
    for event, spec in SOCKET_CLIENT_EVENTS.items():
        channels[f"socket.io/{event}"] = {
            "publish": {
                "operationId": f"client_emit_{event}",
                "message": _message(event, spec["summary"], spec["payload"]),
            }
        }
    for event, summary in SOCKET_SERVER_EVENTS.items():
        channels[f"socket.io/{event}"] = {
            "subscribe": {
                "operationId": f"server_emit_{event}",
                "message": _message(
                    event,
                    summary,
                    {"type": "object", "additionalProperties": True},
                ),
            }
        }
    return {
        "asyncapi": "2.6.0",
        "info": {
            "title": "Drawing Rover Socket.IO API",
            "version": "1.0.0",
            "description": "Current Socket.IO events exposed by rover-server.",
        },
        "servers": {
            "rover": {
                "url": "http://192.168.1.102:5001/socket.io",
                "protocol": "socket.io",
            }
        },
        "channels": channels,
        "components": {
            "schemas": {
                "JoystickAcquire": {
                    "type": "object",
                    "required": ["type", "session_id", "client_monotonic_ms"],
                    "properties": {
                        "type": {"const": "joystick_acquire"},
                        "session_id": {"type": "string"},
                        "client_monotonic_ms": {"type": "integer"},
                    },
                },
                "JoystickCommand": {
                    "type": "object",
                    "required": [
                        "type",
                        "session_id",
                        "lease_id",
                        "sequence",
                        "client_monotonic_ms",
                        "throttle",
                        "steering",
                    ],
                    "properties": {
                        "type": {"const": "joystick_command"},
                        "session_id": {"type": "string"},
                        "lease_id": {"type": "string"},
                        "sequence": {"type": "integer"},
                        "client_monotonic_ms": {"type": "integer"},
                        "deadman": {"type": "boolean"},
                        "throttle": {"type": "number", "minimum": -1, "maximum": 1},
                        "steering": {"type": "number", "minimum": -1, "maximum": 1},
                    },
                },
                "JoystickRelease": {
                    "type": "object",
                    "required": ["type", "session_id", "lease_id"],
                    "properties": {
                        "type": {"const": "joystick_release"},
                        "session_id": {"type": "string"},
                        "lease_id": {"type": "string"},
                    },
                },
                "MissionLoad": {
                    "type": "object",
                    "properties": {
                        "path_name": {"type": "string"},
                        "mission_file": {"type": "string"},
                    },
                },
                "MissionStart": {
                    "type": "object",
                    "properties": {
                        "path_name": {"type": "string"},
                        "mission_file": {"type": "string"},
                        "mission_id": {"type": "string"},
                        "auto_origin": {"type": "boolean"},
                    },
                },
                "PointSkip": {
                    "type": "object",
                    "required": ["point_index"],
                    "properties": {
                        "point_index": {"type": "integer"},
                        "expected_generation": {"type": "integer"},
                        "reason": {"type": "string"},
                    },
                },
                "MissionRestart": {
                    "type": "object",
                    "required": ["mission_id"],
                    "properties": {
                        "mission_id": {"type": "string"},
                        "stop_first": {"type": "boolean"},
                        "start_after_reset": {"type": "boolean"},
                        "auto_origin": {"type": "boolean"},
                    },
                },
            }
        },
    }

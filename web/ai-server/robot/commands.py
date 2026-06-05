"""Robot command queue management with reliable delivery.

Important behavior:
- A GET /api/robot/command does NOT permanently consume a command anymore.
- The command moves to an in-flight slot and is kept there until /api/robot/ack confirms it.
- If a browser/Postman opens the command URL, the ESP32 can still receive the same command later.
"""

from typing import Any

from config import DEFAULT_ROBOT_ID
from utils import normalize_robot_id, now_ms

# Commands that are waiting to be delivered.
robot_command_queues_by_id: dict[str, list[dict[str, Any]]] = {}

# Full command history by commandId.
robot_command_history_by_id: dict[str, dict[str, dict[str, Any]]] = {}

# Command already delivered to the robot but not ACKed yet.
robot_inflight_commands_by_id: dict[str, dict[str, Any]] = {}


def _normalize(robot_id: str | None) -> str:
    return normalize_robot_id(robot_id) or DEFAULT_ROBOT_ID


def append_command(robot_id: str, command: dict[str, Any]) -> dict[str, Any]:
    """Add a command to robot's queue."""
    robot_id = _normalize(robot_id)
    command_id = command.get("commandId") or f"CMD-{now_ms()}"
    payload = {
        "commandId": command_id,
        "robotId": robot_id,
        "createdAt": now_ms(),
        "safetyEnabled": True,
        **command,
        "commandId": command_id,
        "robotId": robot_id,
        "status": command.get("status") or "QUEUED",
    }
    robot_command_queues_by_id.setdefault(robot_id, []).append(payload)
    robot_command_history_by_id.setdefault(robot_id, {})[command_id] = payload
    return payload


def pop_next_command(robot_id: str) -> dict[str, Any] | None:
    """Return the command that the robot should execute next.

    Backward-compatible function name, but it now implements reliable delivery:
    a command is moved to an in-flight slot and is only cleared by ACK.
    """
    robot_id = _normalize(robot_id)

    inflight = robot_inflight_commands_by_id.get(robot_id)
    if inflight is not None and not inflight.get("ack"):
        inflight["status"] = "DELIVERED_TO_ROBOT"
        inflight["lastDeliveredAt"] = now_ms()
        inflight["deliveryCount"] = int(inflight.get("deliveryCount", 1)) + 1
        robot_command_history_by_id.setdefault(robot_id, {})[inflight["commandId"]] = inflight
        return inflight

    queue = robot_command_queues_by_id.setdefault(robot_id, [])
    command = queue.pop(0) if queue else None
    if command is not None:
        delivered_at = now_ms()
        command["status"] = "DELIVERED_TO_ROBOT"
        command["deliveredAt"] = command.get("deliveredAt") or delivered_at
        command["lastDeliveredAt"] = delivered_at
        command["deliveryCount"] = int(command.get("deliveryCount", 0)) + 1
        robot_inflight_commands_by_id[robot_id] = command
        robot_command_history_by_id.setdefault(robot_id, {})[command["commandId"]] = command
    return command


def acknowledge_command(robot_id: str, command_id: str | None, ack: bool) -> dict[str, Any] | None:
    """Update a command ACK state and release the in-flight slot."""
    robot_id = _normalize(robot_id)
    if not command_id:
        return None

    command = robot_command_history_by_id.setdefault(robot_id, {}).get(command_id)
    if command is None:
        return None

    command["ack"] = ack
    command["ackStatus"] = "ACKED" if ack else "REJECTED"
    command["acknowledgedAt"] = now_ms()
    command["status"] = command["ackStatus"]

    inflight = robot_inflight_commands_by_id.get(robot_id)
    if inflight is not None and inflight.get("commandId") == command_id:
        robot_inflight_commands_by_id.pop(robot_id, None)

    # Defensive cleanup: if the same commandId is still in queue, remove it.
    queue = robot_command_queues_by_id.setdefault(robot_id, [])
    robot_command_queues_by_id[robot_id] = [cmd for cmd in queue if cmd.get("commandId") != command_id]

    return command

"""Vision API routes."""

import shutil
import time
from pathlib import Path
from typing import Any

import cv2
import numpy as np
from fastapi import Body, File, Form, HTTPException, Query, UploadFile, WebSocket, WebSocketDisconnect, FastAPI

from config import (
    FRAMES_DIR,
    LATEST_FRAME_PATH,
    DEFAULT_ROBOT_ID,
    MODEL_NAME,
    EVENT_LABELS,
    MAX_FRAME_UPLOAD_BYTES,
    FRAME_MIN_INTERVAL_MS,
)
from connection import VisionConnectionManager
from utils import (
    normalize_robot_id,
    now_ms,
    frame_path_for_robot,
    make_vision_ws_payload,
    get_vision_session,
)
from vision.detection import get_model, model_error
from vision.processing import detect_objects, empty_result
from vision.roi import path_roi_config
from robot.telemetry import register_robot_if_needed, latest_robot_telemetry_by_id
from robot.commands import append_command, robot_command_queues_by_id, robot_inflight_commands_by_id

# WebSocket manager for vision updates
manager = VisionConnectionManager()

# Global state
latest_vision_results_by_robot_id: dict[str, dict[str, Any]] = {}
latest_frame_paths_by_robot_id: dict[str, Path] = {}
last_frame_received_at_by_robot_id: dict[str, int] = {}
last_vision_command_by_robot_id: dict[str, dict[str, Any]] = {}
vision_state_confirm_by_robot_id: dict[str, dict[str, Any]] = {}
ai_hold_by_robot_id: dict[str, dict[str, Any]] = {}

# Prevent AI Vision from flooding the ESP32 command queue at 3 FPS.
VISION_COMMAND_COOLDOWN_MS = 1500

# Do not let one noisy frame stop the robot at startup.
VISION_DANGER_CONFIRM_FRAMES = 2
VISION_RESUME_CONFIRM_FRAMES = 1

# AI is only allowed to stop/slow a robot that is already in a moving/following state.
# This prevents /api/frame startup frames from queuing PAUSE_TASK before the robot receives START_TASK/WMS_TASK.
AI_CONTROL_ACTIVE_STATES = {
    "WAITING_TASK",
    "FOLLOW_LINE",
    "FOLLOW_LINE_TO_TASK",
    "FOLLOW_LINE_WMS_TASK",
    "RUNNING",
    "MOVING",
    "GAP_BRIDGE",
}
AI_CONTROL_HOLD_STATES = {"PAUSED_BY_SERVER", "STOPPED_BY_SERVER"}

# Rotate every incoming ESP32 camera frame on the backend.
# ROTATE_90_COUNTERCLOCKWISE = cv2.ROTATE_90_COUNTERCLOCKWISE.
ROTATE_INCOMING_FRAME_CCW_90 = True


def get_latest_vision_result(robot_id: str) -> dict[str, Any]:
    """Return the latest vision result for a robot or an empty valid contract."""
    normalized = normalize_robot_id(robot_id)
    return latest_vision_results_by_robot_id.get(normalized) or empty_result(normalized, now_ms())


def get_latest_frame_response(robot_id: str) -> Any:
    """Return the latest frame response for a robot."""
    from fastapi.responses import FileResponse
    from utils import safe_robot_filename

    normalized = normalize_robot_id(robot_id)
    frame_path = latest_frame_paths_by_robot_id.get(normalized)
    if frame_path is None and normalized == DEFAULT_ROBOT_ID and LATEST_FRAME_PATH.exists():
        frame_path = LATEST_FRAME_PATH
    if frame_path is None or not frame_path.exists():
        raise HTTPException(status_code=404, detail=f"No frame has been received yet for {normalized}")

    return FileResponse(
        path=frame_path,
        media_type="image/jpeg",
        filename=f"latest_{safe_robot_filename(normalized)}.jpg",
        headers={"Cache-Control": "no-store"},
    )


def _same_ai_command_pending(robot_id: str, command_name: str, vision_state: str | None) -> bool:
    """Return True when the same AI Vision command is waiting or in-flight."""
    queue = robot_command_queues_by_id.setdefault(robot_id, [])
    inflight = robot_inflight_commands_by_id.get(robot_id)
    pending_commands = list(queue)
    if inflight is not None:
        pending_commands.append(inflight)
    return any(
        cmd.get("source") == "AI_VISION"
        and cmd.get("command") == command_name
        and cmd.get("visionState") == vision_state
        and not cmd.get("ack")
        for cmd in pending_commands
    )


def _robot_is_active_for_ai_control(current_robot_state: str) -> bool:
    """Return True when AI Vision is allowed to send stop/slow commands."""
    state = (current_robot_state or "").upper()
    return state in AI_CONTROL_ACTIVE_STATES or state.startswith("FOLLOW_LINE")


def _vision_state_confirmed(robot_id: str, vision_state: str, required_frames: int) -> bool:
    """Require the same AI state for N processed frames before queueing a command."""
    state = vision_state or "FOLLOW_LINE"
    record = vision_state_confirm_by_robot_id.get(robot_id)
    if record is None or record.get("state") != state:
        record = {"state": state, "count": 1, "updatedAt": now_ms()}
    else:
        record = {"state": state, "count": int(record.get("count", 0)) + 1, "updatedAt": now_ms()}
    vision_state_confirm_by_robot_id[robot_id] = record
    return int(record["count"]) >= required_frames


def queue_robot_command_from_vision(robot_id: str, result: dict[str, Any]) -> dict[str, Any] | None:
    """Convert AI Vision safety state into a command for the ESP32-S3 robot.

    This closes the loop:
    POST /api/frame -> detect_objects() -> safetyState -> robot command queue -> GET /api/robot/command.
    """
    robot_id = normalize_robot_id(robot_id)
    safety_state = result.get("safetyState") if isinstance(result.get("safetyState"), dict) else {}
    alert = result.get("alert") if isinstance(result.get("alert"), dict) else {}

    vision_state = str(safety_state.get("suggestedRobotState") or "FOLLOW_LINE")
    alert_action = str(alert.get("action") or "")
    alert_message = str(alert.get("message") or "")
    current_robot_state = str((latest_robot_telemetry_by_id.get(robot_id) or {}).get("state") or "")

    command: dict[str, Any] | None = None
    active_for_ai = _robot_is_active_for_ai_control(current_robot_state)
    holding_by_ai = current_robot_state in AI_CONTROL_HOLD_STATES and ai_hold_by_robot_id.get(robot_id) is not None

    if vision_state == "EMERGENCY_STOP_PERSON" or alert_action == "EMERGENCY_STOP":
        # Startup guard: do not stop a robot that has not started a task/motion yet.
        if not active_for_ai:
            return None
        if current_robot_state == "STOPPED_BY_SERVER":
            return None
        if not _vision_state_confirmed(robot_id, vision_state, VISION_DANGER_CONFIRM_FRAMES):
            return None
        command = {
            "command": "EMERGENCY_STOP",
            "reason": "AI_VISION_PERSON",
            "priority": "CRITICAL",
            "message": alert_message or "AI Vision phát hiện người trong vùng di chuyển.",
        }
    elif vision_state in {"STOP_AND_RECHECK", "STOP_AND_SCAN"} or alert_action in {"STOP_AND_RECHECK", "STOP_AND_SCAN"}:
        # Startup guard: do not pause before START_TASK/WMS_TASK has made the robot active.
        if not active_for_ai:
            return None
        if current_robot_state in AI_CONTROL_HOLD_STATES:
            return None
        if not _vision_state_confirmed(robot_id, vision_state, VISION_DANGER_CONFIRM_FRAMES):
            return None
        command = {
            "command": "PAUSE_TASK",
            "reason": "AI_VISION_OBSTACLE",
            "priority": "HIGH",
            "message": alert_message or "AI Vision phát hiện vật cản trong vùng di chuyển.",
        }
    elif vision_state == "SLOW_DOWN_AND_MONITOR" or alert_action in {"SLOW_DOWN_AND_MONITOR", "SLOW_DOWN_AND_SCAN"}:
        # Do not slow down during WAITING_TASK/boot; only slow an actively moving robot.
        if not active_for_ai:
            return None
        if not _vision_state_confirmed(robot_id, vision_state, VISION_DANGER_CONFIRM_FRAMES):
            return None
        command = {
            "command": "SET_SPEED",
            "reason": "AI_VISION_NEAR_PATH",
            "priority": "MEDIUM",
            "baseSpeed": 35,
            "maxSpeed": 55,
            "message": alert_message or "AI Vision phát hiện vật thể gần đường đi, giảm tốc để theo dõi.",
        }
    elif vision_state == "RESUME_LINE":
        # Only AI can auto-resume a hold that AI itself created.
        # This prevents camera clear frames from cancelling a manual/emergency stop.
        if not holding_by_ai:
            return None
        if not _vision_state_confirmed(robot_id, vision_state, VISION_RESUME_CONFIRM_FRAMES):
            return None
        command = {
            "command": "RESUME_TASK",
            "reason": "AI_VISION_CLEAR",
            "priority": "NORMAL",
            "baseSpeed": 50,
            "maxSpeed": 80,
            "message": "AI Vision xác nhận vùng di chuyển đã an toàn, cho robot chạy tiếp.",
        }
    else:
        _vision_state_confirmed(robot_id, vision_state, 999999)
        return None

    command_name = str(command["command"])
    now = now_ms()
    signature = f"{command_name}:{command.get('reason')}:{vision_state}"
    last = last_vision_command_by_robot_id.get(robot_id)

    if last and last.get("signature") == signature and now - int(last.get("queuedAt", 0)) < VISION_COMMAND_COOLDOWN_MS:
        return None

    if _same_ai_command_pending(robot_id, command_name, vision_state):
        return None

    queued_command = append_command(robot_id, {
        **command,
        "source": "AI_VISION",
        "visionState": vision_state,
        "alertAction": alert_action,
        "detectionSummary": {
            "detected": result.get("detected"),
            "objectCount": result.get("objectCount"),
            "alert": alert,
            "safetyState": safety_state,
        },
    })

    if command_name in {"PAUSE_TASK", "EMERGENCY_STOP"}:
        ai_hold_by_robot_id[robot_id] = {
            "command": command_name,
            "reason": command.get("reason"),
            "visionState": vision_state,
            "queuedAt": now,
            "commandId": queued_command.get("commandId"),
        }
    elif command_name == "RESUME_TASK":
        ai_hold_by_robot_id.pop(robot_id, None)

    last_vision_command_by_robot_id[robot_id] = {
        "signature": signature,
        "queuedAt": now,
        "commandId": queued_command.get("commandId"),
    }
    return queued_command


def register_vision_routes(app: FastAPI) -> None:
    """Register all vision-related endpoints."""

    @app.get("/")
    async def health_check() -> dict[str, Any]:
        """Health check endpoint with service information."""
        return {
            "status": "running",
            "mode": "no-training",
            "service": "WMS Robot Vision AI Server",
            "model": MODEL_NAME,
            "modelReady": get_model() is not None,
            "modelError": model_error,
            "eventLabels": EVENT_LABELS,
            "visionFilter": {
                "backgroundCalibrated": get_vision_session(DEFAULT_ROBOT_ID).background_gray_frame is not None,
                "backgroundCalibratedAt": get_vision_session(DEFAULT_ROBOT_ID).background_calibrated_at,
                "staticObstacleRequiresBackground": True,
                "rejectFullFrameObstacle": True,
                "smallObstacleMinAreaRatio": 0.008,
                "minObstacleAreaRatio": 0.03,
                "maxObstacleAreaRatio": 0.45,
                "fullFrameCoverageRatio": 0.85,
                "roi": {"xMin": 0.15, "xMax": 0.85, "yMin": 0.35},
                "pathRoi": path_roi_config(),
                "clearFrameRequired": 3,
                "personClearFrameRequired": 5,
                "endpoints": [
                    "POST /api/vision/calibrate-background",
                    "POST /api/vision/clear-background",
                    "GET /api/vision/background-status",
                ],
            },
            "robotTelemetry": {
                "enabled": True,
                "endpoints": [
                    "POST /api/robots",
                    "GET /api/robots",
                    "GET /api/robots/{robotId}",
                    "POST /api/robot/telemetry",
                    f"GET /api/robot/latest?robotId={DEFAULT_ROBOT_ID}",
                    "POST /api/robot/task",
                    f"GET /api/robot/command?robotId={DEFAULT_ROBOT_ID}",
                    "POST /api/robot/ack",
                    "WS /ws/robot",
                ],
            },
        }

    @app.post("/api/vision/calibrate-background")
    async def calibrate_background(robotId: str = Query(DEFAULT_ROBOT_ID)) -> dict[str, Any]:
        """Calibrate background for static obstacle detection."""
        robot_id = normalize_robot_id(robotId)
        session = get_vision_session(robot_id)
        frame_path = latest_frame_paths_by_robot_id.get(robot_id) or (LATEST_FRAME_PATH if robot_id == DEFAULT_ROBOT_ID else None)

        if frame_path is None or not frame_path.exists():
            raise HTTPException(status_code=404, detail=f"No latest frame available for calibration of {robot_id}")

        image = cv2.imread(str(frame_path), cv2.IMREAD_COLOR)
        if image is None:
            raise HTTPException(status_code=500, detail="Cannot read latest frame for calibration")

        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        gray = cv2.GaussianBlur(gray, (5, 5), 0)
        session.background_gray_frame = gray.copy()
        session.previous_gray_frame = gray.copy()
        session.background_calibrated_at = now_ms()

        return {
            "ok": True,
            "robotId": robot_id,
            "calibrated": True,
            "calibratedAt": session.background_calibrated_at,
            "message": "Background calibrated. Static objects currently visible will be ignored.",
            "note": "Static OpenCV obstacles are now detected only as differences from this background",
        }

    @app.post("/api/vision/clear-background")
    async def clear_background(robotId: str = Query(DEFAULT_ROBOT_ID)) -> dict[str, Any]:
        """Clear background calibration."""
        robot_id = normalize_robot_id(robotId)
        session = get_vision_session(robot_id)
        session.background_gray_frame = None
        session.background_calibrated_at = None
        session.previous_gray_frame = None
        return {
            "ok": True,
            "robotId": robot_id,
            "calibrated": False,
            "message": "Background calibration cleared",
            "note": "Static OpenCV obstacle detection is disabled until calibration is performed again",
        }

    @app.get("/api/vision/background-status")
    async def background_status(robotId: str = Query(DEFAULT_ROBOT_ID)) -> dict[str, Any]:
        """Get background calibration status."""
        robot_id = normalize_robot_id(robotId)
        session = get_vision_session(robot_id)
        frame_path = latest_frame_paths_by_robot_id.get(robot_id)
        return {
            "robotId": robot_id,
            "calibrated": session.background_gray_frame is not None,
            "calibratedAt": session.background_calibrated_at,
            "hasLatestFrame": bool(frame_path and frame_path.exists()),
            "staticObstacleRequiresBackground": True,
            "filter": {
                "minAreaRatio": 0.03,
                "smallObstacleMinAreaRatio": 0.008,
                "maxAreaRatio": 0.45,
                "fullFrameCoverageRatio": 0.85,
                "roiXMin": 0.15,
                "roiXMax": 0.85,
                "roiYMin": 0.35,
                "pathRoi": path_roi_config(),
                "clearFrameRequired": 3,
                "personClearFrameRequired": 5,
            },
        }

    @app.post("/api/frame")
    async def receive_frame(
        file: UploadFile | None = File(None),
        frame: UploadFile | None = File(None),
        robotId: str | None = Form(None),
        robotIdQuery: str | None = Query(None, alias="robotId"),
        capturedAtMs: int | None = Form(None),
    ) -> dict[str, Any]:
        """Receive a realtime camera frame from ESP32-S3 and process vision.

        Backward compatibility: old clients can upload the file field as `file`;
        ESP32-S3 firmware can use the simpler documented field name `frame`.
        """
        timestamp = now_ms()
        server_received_at_ms = timestamp
        robot_id = normalize_robot_id(robotId or robotIdQuery)
        register_robot_if_needed(robot_id)

        previous_received_at = last_frame_received_at_by_robot_id.get(robot_id, 0)
        if timestamp - previous_received_at < FRAME_MIN_INTERVAL_MS:
            latest = latest_vision_results_by_robot_id.get(robot_id)
            if latest is not None:
                return {
                    **latest,
                    "rateLimited": True,
                    "message": "Frame bị bỏ qua để tránh quá tải; khuyến nghị ESP32-S3 gửi 1-2 fps cho AI Vision.",
                }

        upload = frame or file
        if upload is None:
            raise HTTPException(status_code=400, detail="Thiếu file ảnh. Dùng multipart field `frame` hoặc `file`.")

        try:
            contents = await upload.read()
        except Exception as exc:
            raise HTTPException(status_code=400, detail=f"Cannot read uploaded frame: {exc}") from exc

        if not contents:
            raise HTTPException(status_code=400, detail="Uploaded frame is empty")
        if len(contents) > MAX_FRAME_UPLOAD_BYTES:
            raise HTTPException(status_code=413, detail="Ảnh realtime quá lớn. Khuyến nghị JPEG 320x240 hoặc 640x480, tối đa 2 MB.")

        last_frame_received_at_by_robot_id[robot_id] = timestamp

        image_array = np.frombuffer(contents, dtype=np.uint8)
        image = cv2.imdecode(image_array, cv2.IMREAD_COLOR)
        if image is None:
            raise HTTPException(status_code=400, detail="Uploaded file is not a valid JPEG image")

        # Rotate on the backend instead of on ESP32.
        # This avoids ESP32 JPEG re-encode malloc failures and keeps upload lightweight.
        rotatedByBackend = False
        if ROTATE_INCOMING_FRAME_CCW_90:
            image = cv2.rotate(image, cv2.ROTATE_90_COUNTERCLOCKWISE)
            rotatedByBackend = True

        frame_path = frame_path_for_robot(robot_id)
        ok = cv2.imwrite(str(frame_path), image)
        if not ok:
            raise HTTPException(status_code=500, detail="Cannot save latest frame")
        latest_frame_paths_by_robot_id[robot_id] = frame_path
        if robot_id == DEFAULT_ROBOT_ID:
            shutil.copyfile(frame_path, LATEST_FRAME_PATH)

        process_start = time.perf_counter()
        latest_result = detect_objects(image, robot_id, timestamp)
        processing_time_ms = round((time.perf_counter() - process_start) * 1000, 2)
        server_result_at_ms = now_ms()
        capture_to_server_ms = server_received_at_ms - capturedAtMs if capturedAtMs is not None else None
        capture_to_result_ms = server_result_at_ms - capturedAtMs if capturedAtMs is not None else None
        latest_result["performance"] = {
            "processingTimeMs": processing_time_ms,
            "frameWidth": int(image.shape[1]),
            "frameHeight": int(image.shape[0]),
        }
        latest_result["latency"] = {
            "capturedAtMs": capturedAtMs,
            "serverReceivedAtMs": server_received_at_ms,
            "serverResultAtMs": server_result_at_ms,
            "captureToServerMs": capture_to_server_ms,
            "serverProcessingMs": processing_time_ms,
            "captureToResultMs": capture_to_result_ms,
        }
        latest_result["imageUrl"] = f"/api/latest-frame?robotId={robot_id}&t={timestamp}"
        latest_result["frameTransform"] = {
            "rotatedByBackend": rotatedByBackend,
            "rotation": "CCW_90" if rotatedByBackend else "NONE",
        }
        vision_command = queue_robot_command_from_vision(robot_id, latest_result)
        if vision_command is not None:
            latest_result["robotCommand"] = vision_command

        latest_vision_results_by_robot_id[robot_id] = latest_result
        await manager.broadcast(make_vision_ws_payload(latest_result))
        return latest_result

    @app.get("/api/latest-frame")
    async def latest_frame(robotId: str = Query(DEFAULT_ROBOT_ID)) -> Any:
        """Get latest frame for robot."""
        return get_latest_frame_response(robotId)

    @app.get("/api/latest-result")
    async def latest_detection_result(robotId: str = Query(DEFAULT_ROBOT_ID)) -> dict[str, Any]:
        """Get latest vision detection result."""
        return get_latest_vision_result(robotId)

    @app.websocket("/ws/vision")
    async def vision_websocket(websocket: WebSocket) -> None:
        """WebSocket endpoint for vision updates."""
        await manager.connect(websocket)
        try:
            if latest_vision_results_by_robot_id:
                for result in latest_vision_results_by_robot_id.values():
                    await websocket.send_json(make_vision_ws_payload(result))

            while True:
                await websocket.receive_text()
        except WebSocketDisconnect:
            manager.disconnect(websocket)
        except Exception:
            manager.disconnect(websocket)
            import asyncio
            await asyncio.sleep(0)

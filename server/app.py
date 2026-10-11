import os
import sys
import json
import asyncio
import subprocess
import threading
import time
from pathlib import Path
from typing import List, Optional, Dict, Any

from fastapi import FastAPI, WebSocket, WebSocketDisconnect, HTTPException
from fastapi.staticfiles import StaticFiles
from fastapi.responses import FileResponse, JSONResponse
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel

from serial_comm import SerialManager

BASE_DIR = Path(__file__).resolve().parent
PROJECT_DIR = BASE_DIR.parent
FRONTEND_DIR = PROJECT_DIR / "frontend"
FIRMWARE_DIR = PROJECT_DIR / "firmware" / "robotic_arm"

app = FastAPI(title="6-DOF Robotic Arm Real Physics Controller", version="4.0.0")

app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

serial_mgr = SerialManager(port=None, baudrate=115200)
connected_clients: List[WebSocket] = []
loop: Optional[asyncio.AbstractEventLoop] = None

# Routine execution lock
routine_lock = threading.Lock()
routine_running = False

# Inventory Database
INVENTORY_DATABASE: Dict[str, Dict[str, Any]] = {
    "INV-BOX-01": {
        "id": "INV-BOX-01",
        "name": "Box #1 (Electronics Module)",
        "sku": "SKU-ELEC-01",
        "category": "Components",
        "pick_base": 45.0,
        "drop_base": 135.0,
        "is_reverse": False,
        "location": "Station A (45°)",
        "target_bin": "Drop Bin 1 (135°)",
        "status": "Ready for Pickup",
        "scans_count": 0,
        "last_scanned": None
    },
    "INV-BOX-02": {
        "id": "INV-BOX-02",
        "name": "Box #2 (Mechanical Tools)",
        "sku": "SKU-TOOL-02",
        "category": "Tools",
        "pick_base": 45.0,
        "drop_base": 160.0,
        "is_reverse": False,
        "location": "Station A (45°)",
        "target_bin": "Drop Bin 2 (160°)",
        "status": "Ready for Pickup",
        "scans_count": 0,
        "last_scanned": None
    },
    "INV-BOX-03": {
        "id": "INV-BOX-03",
        "name": "Box #3 (Sensors & Actuators)",
        "sku": "SKU-SENS-03",
        "category": "Electronics",
        "pick_base": 60.0,
        "drop_base": 135.0,
        "is_reverse": False,
        "location": "Station B (60°)",
        "target_bin": "Drop Bin 1 (135°)",
        "status": "Ready for Pickup",
        "scans_count": 0,
        "last_scanned": None
    },
    "RETURN-BIN-01": {
        "id": "RETURN-BIN-01",
        "name": "Return Bin 1 to Station A (Reverse)",
        "sku": "SKU-RET-01",
        "category": "Restock",
        "pick_base": 135.0,
        "drop_base": 45.0,
        "is_reverse": True,
        "location": "Drop Bin 1 (135°)",
        "target_bin": "Station A (45°)",
        "status": "In Drop Bin",
        "scans_count": 0,
        "last_scanned": None
    }
}

# Models
class ServoCommand(BaseModel):
    channel: int
    angle: float

class MoveAllCommand(BaseModel):
    angles: List[float]
    speed: Optional[float] = None

class SpeedCommand(BaseModel):
    speed: float

class RangeCommand(BaseModel):
    channel: int
    maxAngle: float

class PortConnectCommand(BaseModel):
    port: str
    baudrate: Optional[int] = 115200

class RoutineCommand(BaseModel):
    reverse: Optional[bool] = False
    pick_base: Optional[float] = 45.0
    drop_base: Optional[float] = 135.0

class PhysicsConfigCommand(BaseModel):
    payload_mass_kg: Optional[float] = None
    compensation_enabled: Optional[bool] = None
    compensation_gain: Optional[float] = None

class QrScanRequest(BaseModel):
    qr_data: str

class IkSolveRequest(BaseModel):
    x: float
    y: float
    z: float

class IkMoveRequest(BaseModel):
    x: float
    y: float
    z: float
    duration: Optional[float] = 1.5
    num_steps: Optional[int] = 20

# WebSocket broadcast helper
async def broadcast_ws(message: dict):
    if not connected_clients:
        return
    text = json.dumps(message)
    dead_clients = []
    for ws in connected_clients:
        try:
            await ws.send_text(text)
        except Exception:
            dead_clients.append(ws)
    for ws in dead_clients:
        if ws in connected_clients:
            connected_clients.remove(ws)

def on_serial_status(status_data: Dict[str, Any]):
    if loop and loop.is_running():
        asyncio.run_coroutine_threadsafe(
            broadcast_ws({"type": "telemetry", "data": status_data}),
            loop
        )

def on_serial_log(direction: str, text: str):
    if loop and loop.is_running():
        asyncio.run_coroutine_threadsafe(
            broadcast_ws({"type": "log", "direction": direction, "text": text, "timestamp": time.time()}),
            loop
        )

serial_mgr.add_status_listener(on_serial_status)
serial_mgr.add_log_listener(on_serial_log)

@app.on_event("startup")
async def startup_event():
    global loop
    loop = asyncio.get_running_loop()

# Static files
app.mount("/static", StaticFiles(directory=str(FRONTEND_DIR)), name="static")

@app.api_route("/", methods=["GET", "HEAD"])
async def get_index():
    return FileResponse(FRONTEND_DIR / "index.html")

@app.websocket("/ws")
async def websocket_endpoint(websocket: WebSocket):
    await websocket.accept()
    connected_clients.append(websocket)
    try:
        # Send initial full state immediately
        await websocket.send_text(json.dumps({
            "type": "telemetry",
            "data": serial_mgr.status
        }))
        while True:
            data = await websocket.receive_text()
            try:
                msg = json.loads(data)
                action = msg.get("action")
                if action == "ping":
                    await websocket.send_text(json.dumps({"type": "pong"}))
                elif action == "set_servo":
                    ch = msg.get("channel")
                    ang = msg.get("angle")
                    if ch is not None and ang is not None:
                        serial_mgr.send_raw(f"SET {ch} {float(ang):.2f}")
                elif action == "hold":
                    ch = msg.get("channel")
                    if ch is not None:
                        serial_mgr.send_raw(f"HOLD {ch}")
                elif action == "range":
                    ch = msg.get("channel")
                    max_a = msg.get("maxAngle")
                    if ch is not None and max_a is not None:
                        serial_mgr.send_raw(f"RANGE {ch} {float(max_a):.1f}")
                elif action == "raw":
                    cmd = msg.get("cmd")
                    if cmd:
                        serial_mgr.send_raw(cmd)
            except json.JSONDecodeError:
                pass
    except WebSocketDisconnect:
        if websocket in connected_clients:
            connected_clients.remove(websocket)
    except Exception:
        if websocket in connected_clients:
            connected_clients.remove(websocket)

# REST Endpoints
@app.get("/api/status")
async def get_status():
    return serial_mgr.status

@app.get("/api/ports")
async def get_ports():
    return {
        "ports": serial_mgr.list_available_ports(),
        "current": serial_mgr.port,
        "connected": serial_mgr.is_connected,
        "simulated": serial_mgr.is_simulated
    }

@app.post("/api/connect")
async def connect_port(cmd: PortConnectCommand):
    serial_mgr.is_flashing = False
    serial_mgr.auto_reconnect = True
    success = serial_mgr.connect(cmd.port, cmd.baudrate or 115200)
    return {"success": success, "port": cmd.port, "connected": serial_mgr.is_connected}

@app.post("/api/disconnect")
async def disconnect_port():
    serial_mgr.disconnect(for_flash=True)
    return {"success": True, "connected": False}

@app.post("/api/simulation")
async def toggle_simulation(enable: bool = True):
    if enable:
        serial_mgr.enable_simulation_mode()
    else:
        serial_mgr.is_simulated = False
        serial_mgr.status["simulated"] = False
    return {"simulated": serial_mgr.is_simulated}

@app.post("/api/set_servo")
async def set_servo(cmd: ServoCommand):
    if not (0 <= cmd.channel < 6):
        raise HTTPException(status_code=400, detail="Channel must be 0 to 5")
    angle = max(0.0, min(180.0, float(cmd.angle)))
    success = serial_mgr.send_raw(f"SET {cmd.channel} {angle:.2f}")
    return {"success": success, "channel": cmd.channel, "angle": angle}

@app.post("/api/move_all")
async def move_all(cmd: MoveAllCommand):
    if len(cmd.angles) != 6:
        raise HTTPException(status_code=400, detail="Must provide exactly 6 angles (CH 0-5)")
    cmd_str = f"MOVE {' '.join(f'{a:.2f}' for a in cmd.angles)}"
    if cmd.speed is not None:
        cmd_str += f" {cmd.speed:.1f}"
    success = serial_mgr.send_raw(cmd_str)
    return {"success": success, "angles": cmd.angles}

@app.post("/api/speed")
async def set_speed(cmd: SpeedCommand):
    spd = max(5.0, min(360.0, float(cmd.speed)))
    success = serial_mgr.send_raw(f"SPEED {spd:.1f}")
    return {"success": success, "speed": spd}

@app.post("/api/attach")
async def attach_all():
    success = serial_mgr.send_raw("ATTACH")
    return {"success": success, "attached": True}

@app.post("/api/detach")
async def detach_all():
    success = serial_mgr.send_raw("DETACH")
    return {"success": success, "attached": False}

@app.post("/api/isolate/{channel}")
async def isolate_channel(channel: int):
    if not (0 <= channel < 6):
        raise HTTPException(status_code=400, detail="Channel must be 0 to 5")
    success = serial_mgr.send_raw(f"ISOLATE {channel}")
    return {"success": success, "channel": channel}

@app.post("/api/relax/{channel}")
async def relax_channel(channel: int):
    if not (0 <= channel < 6):
        raise HTTPException(status_code=400, detail="Channel must be 0 to 5")
    success = serial_mgr.send_raw(f"RELAX {channel}")
    return {"success": success, "channel": channel}

@app.post("/api/hold/{channel}")
async def hold_channel(channel: int):
    if not (0 <= channel < 6):
        raise HTTPException(status_code=400, detail="Channel must be 0 to 5")
    success = serial_mgr.send_raw(f"HOLD {channel}")
    return {"success": success, "channel": channel}

@app.post("/api/range")
async def range_channel(cmd: RangeCommand):
    if not (0 <= cmd.channel < 6):
        raise HTTPException(status_code=400, detail="Channel must be 0 to 5")
    max_a = max(45.0, min(360.0, float(cmd.maxAngle)))
    success = serial_mgr.send_raw(f"RANGE {cmd.channel} {max_a:.1f}")
    return {"success": success, "channel": cmd.channel, "maxAngle": max_a}

# Safe Homing and Planning Endpoints
@app.post("/api/pose/home_safe")
async def home_safe_endpoint():
    """
    Universal collision-free return to Home Position (0.00, 0.00, 420.00) mm:
    Phase 1: Vertical clearance lift while maintaining base orientation to prevent table strikes.
    Phase 2: Slew base to 90.00° and lock all joints into safe standby configuration.
    """
    cur = serial_mgr.status.get("cur", [90.0, 90.0, 90.0, 90.0, 90.0, 20.0])
    plan = serial_mgr.physics.plan_safe_homing_path(cur)

    def run_safe_homing_bg():
        serial_mgr._notify_log("SYS", "🏠 Initiating Collision-Free Return to Safe Home (0, 0, 420mm)...")
        for idx, wp in enumerate(plan["waypoints"]):
            serial_mgr._notify_log("SYS", f"[{wp['phase'].upper()} ({idx+1}/{len(plan['waypoints'])})] {wp['description']}")
            target = wp["target_angles"]
            dur = wp["duration"]
            cur_phase = serial_mgr.status.get("cur", cur)
            traj = serial_mgr.physics.generate_joint_trajectory(cur_phase, target, duration_sec=dur, num_steps=20)
            dt = dur / float(len(traj))
            for pt in traj:
                ang = pt["angles"]
                serial_mgr.send_raw(f"MOVE {' '.join(f'{a:.2f}' for a in ang)}")
                time.sleep(dt)
            serial_mgr.send_raw(f"MOVE {' '.join(f'{a:.2f}' for a in target)}")
            time.sleep(0.2)
        serial_mgr._notify_log("SYS", "✅ Successfully Returned to Safe Home: (0.00, 0.00, 420.00) mm")

    threading.Thread(target=run_safe_homing_bg, daemon=True).start()

    return {
        "success": True,
        "action": "home_safe",
        "home_coords": plan["home_coords"],
        "phases": len(plan["waypoints"]),
        "plan": plan
    }

@app.get("/api/trajectory/home_plan")
async def get_home_plan_endpoint():
    """Returns the multi-phase safe homing plan computed from current live angles."""
    cur = serial_mgr.status.get("cur", [90.0, 90.0, 90.0, 90.0, 90.0, 20.0])
    return serial_mgr.physics.plan_safe_homing_path(cur)

@app.post("/api/pose/{name}")
async def set_pose(name: str):
    name_clean = name.upper().strip()
    if name_clean in ["HOME", "STANDBY", "HOME_SAFE"]:
        return await home_safe_endpoint()

    valid_poses = {
        "PICK": "PICK",
        "LIFT": "LIFT",
        "DROP": "DROP",
        "CLAW": "CLAW"
    }
    if name_clean not in valid_poses:
        raise HTTPException(status_code=400, detail=f"Unknown pose: {name}")
    success = serial_mgr.send_raw(valid_poses[name_clean])
    return {"success": success, "pose": name_clean}

# =============================================================================
# INVERSE KINEMATICS & JOINT-SPACE MOTION PLANNING ENDPOINTS (RxInfer Principles)
# =============================================================================
@app.post("/api/ik/solve")
async def solve_ik_endpoint(req: IkSolveRequest):
    """Solves analytical/DLS IK for given Cartesian coordinates (mm)."""
    cur = serial_mgr.status.get("cur", [90.0, 90.0, 90.0, 90.0, 90.0, 20.0])
    result = serial_mgr.physics.inverse_kinematics_3d(req.x, req.y, req.z, cur_angles=cur)
    return result

@app.post("/api/ik/move")
async def move_to_xyz_endpoint(req: IkMoveRequest):
    """
    Solves IK for target (X, Y, Z) and executes smooth quintic minimum-jerk
    trajectory in joint space between current configuration and target.
    """
    cur = serial_mgr.status.get("cur", [90.0, 90.0, 90.0, 90.0, 90.0, 20.0])
    ik_sol = serial_mgr.physics.inverse_kinematics_3d(req.x, req.y, req.z, cur_angles=cur)

    if not ik_sol["reachable"] and ik_sol["position_error_mm"] > 40.0:
        raise HTTPException(
            status_code=400,
            detail=f"Target Cartesian coordinate ({req.x}, {req.y}, {req.z}) is unreachable (reach error: {ik_sol['position_error_mm']}mm)"
        )

    target_angles = ik_sol["solved_angles"]
    duration = max(0.5, min(10.0, float(req.duration or 1.5)))
    steps = max(10, min(50, int(req.num_steps or 25)))
    traj = serial_mgr.physics.generate_joint_trajectory(cur, target_angles, duration_sec=duration, num_steps=steps)

    def run_traj_background():
        dt = duration / float(len(traj))
        for pt in traj:
            ang = pt["angles"]
            serial_mgr.send_raw(f"MOVE {' '.join(f'{a:.2f}' for a in ang)}")
            time.sleep(dt)
        # Settle at exact target pose
        serial_mgr.send_raw(f"MOVE {' '.join(f'{a:.2f}' for a in target_angles)}")

    threading.Thread(target=run_traj_background, daemon=True).start()

    return {
        "success": True,
        "target": [round(req.x, 2), round(req.y, 2), round(req.z, 2)],
        "solved_angles": target_angles,
        "achieved_coords": ik_sol["achieved_coords"],
        "position_error_mm": ik_sol["position_error_mm"],
        "duration_sec": duration,
        "trajectory_steps": len(traj)
    }

# Hardcoded Pick & Drop Routine Execution
@app.post("/api/routine/pick_drop")
async def run_pick_drop_routine(cmd: RoutineCommand):
    rev_str = "REVERSE" if cmd.reverse else "PICKDROP"
    success = serial_mgr.send_raw(rev_str)
    return {
        "success": success,
        "routine": "pick_drop",
        "reverse": cmd.reverse,
        "pick_base": cmd.pick_base,
        "drop_base": cmd.drop_base
    }

@app.post("/api/routine/halt")
async def halt_routine():
    serial_mgr.send_raw("STOP")
    return {"success": True, "halted": True}

# Physics Configuration
@app.post("/api/physics/config")
async def configure_physics(cmd: PhysicsConfigCommand):
    if cmd.payload_mass_kg is not None:
        serial_mgr.physics.set_payload_mass(cmd.payload_mass_kg)
    if cmd.compensation_enabled is not None:
        serial_mgr.physics.set_compensation_enabled(cmd.compensation_enabled)
    if cmd.compensation_gain is not None:
        serial_mgr.physics.set_compensation_gain(cmd.compensation_gain)
    serial_mgr._update_physics_state()
    serial_mgr._notify_listeners(serial_mgr.status)
    return {
        "success": True,
        "physics": serial_mgr.status.get("physics")
    }

# Ultrasonic Sonar
@app.get("/api/sonar")
async def read_sonar():
    serial_mgr.send_raw("DIST")
    return {"distance_cm": serial_mgr.status.get("dist", -1.0)}

# Firmware Flash Utility
@app.post("/api/flash")
async def flash_firmware():
    port = serial_mgr.port or "/dev/ttyUSB0"
    serial_mgr.disconnect(for_flash=True)
    time.sleep(1.2)

    arduino_cli = shutil_which("arduino-cli") or "/usr/bin/arduino-cli" or str(Path.home() / ".local/bin/arduino-cli")
    fqbn = "arduino:avr:uno"

    try:
        # Compile
        comp = subprocess.run(
            [arduino_cli, "compile", "--fqbn", fqbn, str(FIRMWARE_DIR)],
            capture_output=True, text=True, timeout=30
        )
        if comp.returncode != 0:
            serial_mgr.resume_after_flash(port)
            return JSONResponse(status_code=500, content={"success": False, "stage": "compile", "error": comp.stderr})

        # Upload
        up = subprocess.run(
            [arduino_cli, "upload", "-p", port, "--fqbn", fqbn, str(FIRMWARE_DIR)],
            capture_output=True, text=True, timeout=30
        )
        time.sleep(1.5)
        serial_mgr.resume_after_flash(port)

        if up.returncode == 0:
            return {"success": True, "message": "Firmware compiled and uploaded successfully!"}
        else:
            return JSONResponse(status_code=500, content={"success": False, "stage": "upload", "error": up.stderr})
    except Exception as e:
        serial_mgr.resume_after_flash(port)
        return JSONResponse(status_code=500, content={"success": False, "error": str(e)})

def shutil_which(cmd: str) -> Optional[str]:
    import shutil
    return shutil.which(cmd)

# Inventory Management API
@app.get("/api/inventory")
async def get_inventory():
    return {"items": list(INVENTORY_DATABASE.values())}

@app.post("/api/scan_qr")
async def process_qr_scan(req: QrScanRequest):
    code = req.qr_data.strip()
    item = INVENTORY_DATABASE.get(code)
    if not item:
        item = {
            "id": code,
            "name": f"Scanned Item ({code})",
            "sku": f"SKU-{code[:6]}",
            "category": "Custom Scanned",
            "pick_base": 45.0,
            "drop_base": 135.0,
            "is_reverse": False,
            "location": "Custom Station",
            "target_bin": "Bin 1",
            "status": "Scanned Custom",
            "scans_count": 0,
            "last_scanned": None
        }
        INVENTORY_DATABASE[code] = item

    item["scans_count"] += 1
    item["last_scanned"] = time.strftime("%Y-%m-%d %H:%M:%S")
    item["status"] = "Processing Pick & Drop"

    # Trigger pick & drop routine
    is_rev = item.get("is_reverse", False)
    serial_mgr.send_raw("REVERSE" if is_rev else "PICKDROP")

    return {
        "success": True,
        "item": item,
        "routine_triggered": True,
        "is_reverse": is_rev
    }

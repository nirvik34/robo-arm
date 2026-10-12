"""
High-Speed Serial Communication & Physics Telemetry Manager for 6-DOF Robotic Arm
Connects to Arduino Uno over USB/Serial (/dev/ttyUSB0 or /dev/ttyACM0) at 115200 baud.
Handles bi-directional JSON messaging, live 6-channel angle telemetry,
and real multi-link physics & gravity calculations.
"""

import serial
import serial.tools.list_ports
import threading
import time
import json
import logging
from typing import Optional, Callable, Dict, Any, List

from physics import RoboticArmPhysics

logger = logging.getLogger("RoboticArmSerial")
logging.basicConfig(level=logging.INFO)

class SerialManager:
    def __init__(self, port: Optional[str] = None, baudrate: int = 115200):
        if not port:
            detected = self.list_available_ports()
            ard_ports = [p["port"] for p in detected if p["is_arduino"]]
            if ard_ports:
                port = ard_ports[0]
            elif detected:
                port = detected[0]["port"]
            else:
                port = "/dev/ttyUSB0"

        self.port = port
        self.baudrate = baudrate
        self.ser: Optional[serial.Serial] = None
        self.is_connected = False
        self.is_simulated = False
        self.auto_reconnect = True

        # Physics engine
        self.physics = RoboticArmPhysics()

        # Telemetry State (All 6 channels active: 0=Base, 1=Shoulder, 2=Elbow, 3=Wrist Pitch, 4=Wrist Roll, 5=Claw)
        self.status: Dict[str, Any] = {
            "attached": False,
            "speed": 70.0,
            "cur": [90.00, 90.00, 90.00, 90.00, 90.00, 20.00],
            "tgt": [90.00, 90.00, 90.00, 90.00, 90.00, 20.00],
            "active": [False, False, False, False, False, False],
            "dist": -1.0,
            "routine": 0,
            "pca": True,
            "port": self.port,
            "baudrate": self.baudrate,
            "connected": False,
            "simulated": False,
            "physics": self.physics.compute_full_physics_frame(
                [90.0, 90.0, 90.0, 90.0, 90.0, 20.0],
                [90.0, 90.0, 90.0, 90.0, 90.0, 20.0]
            )
        }

        self._running = False
        self._thread: Optional[threading.Thread] = None
        self._sim_thread: Optional[threading.Thread] = None
        self._watchdog_thread: Optional[threading.Thread] = None
        self._lock = threading.RLock()
        self.listeners: List[Callable[[Dict[str, Any]], None]] = []
        self.log_listeners: List[Callable[[str, str], None]] = []

        # Auto-connect if port exists
        self.connect(self.port, self.baudrate)

        # Watchdog loop
        self._watchdog_running = True
        self._watchdog_thread = threading.Thread(target=self._watchdog_loop, daemon=True)
        self._watchdog_thread.start()

    @staticmethod
    def list_available_ports() -> List[Dict[str, Any]]:
        ports = []
        for p in serial.tools.list_ports.comports():
            if p.device.startswith("/dev/ttyS") and p.hwid == "n/a":
                continue

            hwid_lower = (p.hwid or "").lower()
            desc_lower = (p.description or "").lower()
            dev_lower = p.device.lower()

            is_arduino = (
                "2341:" in hwid_lower or      # Official Arduino VID
                "1a86:" in hwid_lower or      # QinHeng CH340
                "10c4:" in hwid_lower or      # Silicon Labs CP210x
                "0403:" in hwid_lower or      # FTDI FT232
                "arduino" in desc_lower or 
                "ch340" in desc_lower or
                "ttyacm" in dev_lower or
                "ttyusb" in dev_lower
            )
            desc = p.description or p.device
            if is_arduino and ("uno" not in desc.lower() and "arduino" not in desc.lower()):
                desc = f"Arduino Uno / Controller ({p.description or p.device})"

            ports.append({
                "port": p.device,
                "description": desc,
                "hwid": p.hwid,
                "is_arduino": is_arduino
            })

        ports.sort(key=lambda x: not x["is_arduino"])
        return ports

    def add_status_listener(self, callback: Callable[[Dict[str, Any]], None]):
        self.listeners.append(callback)

    def add_log_listener(self, callback: Callable[[str, str], None]):
        self.log_listeners.append(callback)

    def _notify_listeners(self, data: Dict[str, Any]):
        for cb in self.listeners:
            try:
                cb(data)
            except Exception as e:
                logger.error(f"Error in status listener: {e}")

    def _notify_log(self, direction: str, text: str):
        logger.info(f"[{direction}] {text}")
        for cb in self.log_listeners:
            try:
                cb(direction, text)
            except Exception as e:
                logger.error(f"Error in log listener: {e}")

    def _update_physics_state(self):
        """Update live forward kinematics and gravity torques from current angles."""
        try:
            self.status["physics"] = self.physics.compute_full_physics_frame(
                self.status["cur"],
                self.status["tgt"]
            )
        except Exception as e:
            logger.error(f"Error updating physics: {e}")

    def connect(self, port: Optional[str] = None, baudrate: Optional[int] = None) -> bool:
        with self._lock:
            if port:
                self.port = port
            if baudrate:
                self.baudrate = baudrate

            if self.ser and self.ser.is_open:
                try:
                    self.ser.close()
                except Exception:
                    pass
                self.ser = None

            try:
                logger.info(f"Connecting to serial port {self.port} at {self.baudrate} baud...")
                self.ser = serial.Serial(self.port, self.baudrate, timeout=1.0)
                self.is_connected = True
                self.is_simulated = False
                self.status["connected"] = True
                self.status["simulated"] = False
                self.status["port"] = self.port

                self._running = True
                self._thread = threading.Thread(target=self._reader_loop, daemon=True)
                self._thread.start()

                time.sleep(1.8) # Allow bootloader reset
                self.send_raw("STATUS")
                self.send_raw("DIST")
                self._notify_log("SYS", f"Connected to Arduino Uno on {self.port} ({self.baudrate} baud)")
                self._update_physics_state()
                self._notify_listeners(self.status)
                return True
            except Exception as e:
                logger.warning(f"Could not open port {self.port}: {e}. Enabling Virtual Physics Simulation mode.")
                self.ser = None
                self.is_connected = False
                self.status["connected"] = False
                self.enable_simulation_mode()
                return False

    def disconnect(self, for_flash: bool = False):
        with self._lock:
            self.is_flashing = for_flash
            if for_flash:
                self.auto_reconnect = False
            self._running = False
            if self.ser and self.ser.is_open:
                try:
                    self.ser.close()
                except Exception:
                    pass
            self.ser = None
            self.is_connected = False
            self.status["connected"] = False
            if self._thread and self._thread.is_alive():
                try:
                    self._thread.join(timeout=1.0)
                except Exception:
                    pass
            self._notify_log("SYS", "Disconnected from serial port (port released).")
            self._notify_listeners(self.status)

    def resume_after_flash(self, port: Optional[str] = None):
        self.is_flashing = False
        self.auto_reconnect = True
        time.sleep(1.5) # allow board bootloader to start sketch
        return self.connect(port or self.port, self.baudrate)

    def enable_simulation_mode(self):
        with self._lock:
            self.is_simulated = True
            self.status["simulated"] = True
            self.status["connected"] = False
            self.status["attached"] = True
            self.status["active"] = [True] * 6
            self.status["pca"] = True
            if not self._sim_thread or not self._sim_thread.is_alive():
                self._sim_thread = threading.Thread(target=self._sim_loop, daemon=True)
                self._sim_thread.start()
            self._notify_log("SYS", "Virtual Physics & Arm Simulation Active (50Hz Real-Time FK/Gravity)")
            self._update_physics_state()
            self._notify_listeners(self.status)

    def _sim_loop(self):
        """50Hz virtual simulation loop when hardware is offline."""
        dt = 0.02 # 20ms = 50Hz
        while self.is_simulated:
            time.sleep(dt)
            updated = False
            speed = self.status.get("speed", 70.0)
            max_step = speed * dt

            cur = list(self.status["cur"])
            tgt = list(self.status["tgt"])

            for i in range(6):
                if cur[i] != tgt[i]:
                    diff = tgt[i] - cur[i]
                    if abs(diff) <= max_step:
                        cur[i] = tgt[i]
                    else:
                        cur[i] += max_step if diff > 0 else -max_step
                    cur[i] = round(cur[i], 2)
                    updated = True

            if updated:
                self.status["cur"] = cur
                self._update_physics_state()
                self._notify_listeners(self.status)

    def _watchdog_loop(self):
        while self._watchdog_running:
            time.sleep(3.0)
            if self.auto_reconnect and not getattr(self, 'is_flashing', False) and not self.is_connected and not self.is_simulated:
                try:
                    ports = self.list_available_ports()
                    ard_ports = [p["port"] for p in ports if p["is_arduino"]]
                    candidate = ard_ports[0] if ard_ports else (ports[0]["port"] if ports else None)
                    if candidate:
                        self.connect(candidate, self.baudrate)
                except Exception:
                    pass

    def _reader_loop(self):
        buf = ""
        while self._running and self.ser and self.ser.is_open:
            try:
                line_bytes = self.ser.readline()
                if not line_bytes:
                    continue
                line = line_bytes.decode("utf-8", errors="replace").strip()
                if not line:
                    continue

                self._notify_log("RECV", line)

                if line.startswith("{") and line.endswith("}"):
                    try:
                        data = json.loads(line)
                        m_type = data.get("type")
                        if m_type == "status":
                            for k in ["attached", "speed", "cur", "tgt", "active", "dist", "routine", "pca"]:
                                if k in data:
                                    self.status[k] = data[k]
                            self.status["connected"] = True
                            self.status["simulated"] = False
                            self._update_physics_state()
                            self._notify_listeners(self.status)
                        elif m_type == "init":
                            self._notify_log("SYS", f"Device Init: {data}")
                    except json.JSONDecodeError:
                        pass
                elif line.startswith("[STEP"):
                    self._notify_log("ROUTINE", line)
                elif "Routine Complete" in line:
                    self.status["routine"] = 0
                    self._notify_listeners(self.status)

            except serial.SerialException as e:
                logger.error(f"Serial port disconnected: {e}")
                self.is_connected = False
                self.status["connected"] = False
                self._notify_log("SYS", f"Serial connection lost: {e}")
                self._notify_listeners(self.status)
                break
            except Exception as e:
                logger.error(f"Error in reader loop: {e}")
                time.sleep(0.05)

    def send_raw(self, cmd: str) -> bool:
        cmd_clean = cmd.strip()
        self._notify_log("SEND", cmd_clean)

        if self.is_simulated or not self.ser or not self.ser.is_open:
            # Handle commands in simulation mode
            self._handle_sim_command(cmd_clean)
            return True

        try:
            self.ser.write((cmd_clean + "\n").encode("utf-8"))
            self.ser.flush()
            return True
        except Exception as e:
            logger.error(f"Failed to write to serial: {e}")
            self._notify_log("SYS", f"Write failed: {e}")
            return False

    def _handle_sim_command(self, cmd: str):
        """Processes commands inside simulation mode."""
        parts = cmd.split()
        if not parts:
            return

        c = parts[0].upper()
        if c == "ATTACH":
            self.status["attached"] = True
            self.status["active"] = [True] * 6
        elif c in ["DETACH", "STOP", "HALT"]:
            self.status["attached"] = False
            self.status["active"] = [False] * 6
            self.status["routine"] = 0
        elif c == "ISOLATE" and len(parts) >= 2:
            try:
                ch = int(parts[1])
                if 0 <= ch < 6:
                    self.status["attached"] = True
                    self.status["active"] = [False] * 6
                    self.status["active"][ch] = True
            except ValueError:
                pass
        elif c in ["RELAX", "OFF"] and len(parts) >= 2:
            try:
                ch = int(parts[1])
                if 0 <= ch < 6:
                    self.status["active"][ch] = False
                    if not any(self.status["active"]):
                        self.status["attached"] = False
            except ValueError:
                pass
        elif c == "SET" and len(parts) >= 3:
            try:
                ch = int(parts[1])
                val = float(parts[2])
                if 0 <= ch < 6:
                    val = max(0.0, min(360.0, val))
                    self.status["attached"] = True
                    self.status["active"][ch] = True
                    self.status["tgt"][ch] = round(val, 2)
                    self.status["cur"][ch] = round(val, 2)
                    self._update_physics_state()
                    self._notify_listeners(self.status)
            except ValueError:
                pass
        elif c == "HOLD" and len(parts) >= 2:
            try:
                ch = int(parts[1])
                if 0 <= ch < 6:
                    self.status["attached"] = True
                    self.status["active"][ch] = True
                    self.status["tgt"][ch] = self.status["cur"][ch]
                    self._notify_listeners(self.status)
            except ValueError:
                pass
        elif c == "MOVE" and len(parts) >= 7:
            try:
                vals = [float(p) for p in parts[1:7]]
                self.status["attached"] = True
                self.status["active"] = [True] * 6
                self.status["tgt"] = [round(max(0.0, min(180.0, v)), 2) for v in vals]
                if len(parts) >= 8:
                    self.status["speed"] = float(parts[7])
            except ValueError:
                pass
        elif c == "SPEED" and len(parts) >= 2:
            try:
                self.status["speed"] = float(parts[1])
            except ValueError:
                pass
        elif c in ["HOME", "STANDBY"]:
            self.status["tgt"] = [90.0, 90.0, 90.0, 90.0, 90.0, 20.0]
        elif c == "PICK":
            self.status["tgt"] = [45.0, 60.0, 50.0, 105.0, 90.0, 20.0]
        elif c == "LIFT":
            self.status["tgt"] = [45.0, 95.0, 100.0, 85.0, 90.0, 80.0]
        elif c == "DROP":
            self.status["tgt"] = [135.0, 65.0, 55.0, 100.0, 90.0, 20.0]
        elif c == "CLAW":
            cur_claw = self.status["tgt"][5]
            self.status["tgt"][5] = 20.0 if cur_claw > 40.0 else 80.0
        elif c == "DIST":
            self.status["dist"] = 14.5
        elif c in ["PICKDROP", "RUN"]:
            p_base = 45.0
            d_base = 135.0
            if len(parts) >= 3:
                try:
                    p_base = float(parts[1])
                    d_base = float(parts[2])
                except ValueError:
                    pass
            threading.Thread(target=self._simulate_routine, args=(False, p_base, d_base), daemon=True).start()
        elif c == "REVERSE":
            p_base = 45.0
            d_base = 135.0
            if len(parts) >= 3:
                try:
                    p_base = float(parts[1])
                    d_base = float(parts[2])
                except ValueError:
                    pass
            threading.Thread(target=self._simulate_routine, args=(True, p_base, d_base), daemon=True).start()

        self._update_physics_state()
        self._notify_listeners(self.status)

    def _simulate_routine(self, reverse: bool, pick_b: float = 45.0, drop_b: float = 135.0):
        """Simulates full pick & drop routine in virtual physics mode."""
        self.status["routine"] = 1
        src_b = drop_b if reverse else pick_b
        dst_b = pick_b if reverse else drop_b

        steps = [
            ("STEP 1: Sonar payload detection", [self.status["cur"][0], 90.0, 90.0, 90.0, 90.0, 20.0], 0.7),
            ("STEP 2: Align Base & Open Claw", [src_b, 90.0, 90.0, 90.0, 90.0, 20.0], 1.0),
            ("STEP 3: Reach down onto payload", [src_b, 60.0, 50.0, 105.0, 90.0, 20.0], 1.2),
            ("STEP 4: Clamping gripper", [src_b, 60.0, 50.0, 105.0, 90.0, 80.0], 0.7),
            ("STEP 5: Vertical lift with gravity compensation", [src_b, 95.0, 100.0, 85.0, 90.0, 80.0], 1.2),
            ("STEP 6: Transferring to drop station", [dst_b, 95.0, 100.0, 85.0, 90.0, 80.0], 1.4),
            ("STEP 7: Lowering into drop bin", [dst_b, 65.0, 55.0, 100.0, 90.0, 80.0], 1.1),
            ("STEP 8: Releasing payload", [dst_b, 65.0, 55.0, 100.0, 90.0, 20.0], 0.7),
            ("STEP 9: Safe Home Return", [90.0, 90.0, 90.0, 90.0, 90.0, 20.0], 1.1),
        ]

        total = len(steps)
        for idx, (desc, tgt, duration) in enumerate(steps, 1):
            if not self.status.get("routine", 0):
                break
            self.status["routine"] = idx
            self.status["routine_step"] = idx
            self.status["routine_desc"] = desc
            self.status["routine_total_steps"] = total
            self._notify_log("ROUTINE", f"[{idx}/{total}] {desc}")
            self.status["tgt"] = list(tgt)
            self._notify_listeners(self.status)
            time.sleep(duration)

        self.status["routine"] = 0
        self.status["routine_step"] = 0
        self.status["routine_desc"] = "Standby / Ready"
        self._notify_log("ROUTINE", "✅ Autonomous Routine Complete!")
        self._notify_listeners(self.status)


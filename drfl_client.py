"""Doosan DRFL (read-only) bindings and background client for M1013 monitoring.

Uses the DRFLWin64.dll shipped with the VisualGrip package. No servo, mode,
motion or I/O commands are issued.
"""

from __future__ import annotations

import ctypes as C
import os
import threading
import time
from dataclasses import dataclass, field
from typing import Callable, Optional

NUM_JOINT = 6
NUM_TASK = 6
DEFAULT_PORT = 12345

DEFAULT_ROBOTS = {
    "R1": "192.168.137.101",
    "R2": "192.168.137.102",
}

ROBOT_STATE_NAMES = {
    0: "INITIALIZING",
    1: "STANDBY",
    2: "MOVING",
    3: "SAFE_OFF",
    4: "TEACHING",
    5: "SAFE_STOP",
    6: "EMERGENCY_STOP",
    7: "HOMMING",
    8: "RECOVERY",
    9: "SAFE_STOP2",
    10: "SAFE_OFF2",
    11: "RESERVED1",
    12: "RESERVED2",
    13: "RESERVED3",
    14: "RESERVED4",
    15: "NOT_READY",
}

ROBOT_MODE_NAMES = {
    0: "MANUAL",
    1: "AUTONOMOUS",
    2: "RECOVERY",
    3: "BACKDRIVE",
    4: "MEASURE",
    5: "INITIALIZE",
}

SPEED_MODE_NAMES = {0: "NORMAL", 1: "REDUCED"}

PROGRAM_STATE_NAMES = {0: "PLAY", 1: "STOP", 2: "HOLD"}


class _MonitorState(C.Structure):
    _pack_ = 1
    _fields_ = [("_iActualMode", C.c_ubyte), ("_iActualSpace", C.c_ubyte)]


class _MonitorJoint(C.Structure):
    _pack_ = 1
    _fields_ = [
        ("_fActualPos", C.c_float * NUM_JOINT),
        ("_fActualAbs", C.c_float * NUM_JOINT),
        ("_fActualVel", C.c_float * NUM_JOINT),
        ("_fActualErr", C.c_float * NUM_JOINT),
        ("_fTargetPos", C.c_float * NUM_JOINT),
        ("_fTargetVel", C.c_float * NUM_JOINT),
    ]


class _MonitorTask(C.Structure):
    _pack_ = 1
    _fields_ = [
        ("_fActualPos", (C.c_float * NUM_JOINT) * 2),
        ("_fActualVel", C.c_float * NUM_JOINT),
        ("_fActualErr", C.c_float * NUM_JOINT),
        ("_fTargetPos", C.c_float * NUM_JOINT),
        ("_fTargetVel", C.c_float * NUM_JOINT),
        ("_iSolutionSpace", C.c_ubyte),
        ("_fRotationMatrix", (C.c_float * 3) * 3),
    ]


class _MonitorTorque(C.Structure):
    _pack_ = 1
    _fields_ = [
        ("_fDynamicTor", C.c_float * NUM_JOINT),
        ("_fActualJTS", C.c_float * NUM_JOINT),
        ("_fActualEJT", C.c_float * NUM_JOINT),
        ("_fActualETT", C.c_float * NUM_JOINT),
    ]


class _MonitorMisc(C.Structure):
    _pack_ = 1
    _fields_ = [
        ("_dSyncTime", C.c_double),
        ("_iActualDI", C.c_ubyte * 6),
        ("_iActualDO", C.c_ubyte * 6),
        ("_iActualBK", C.c_ubyte * 6),
        ("_iActualBT", C.c_uint * 5),
        ("_fActualMC", C.c_float * NUM_JOINT),
        ("_fActualMT", C.c_float * NUM_JOINT),
    ]


class _MonitorControl(C.Structure):
    _pack_ = 1
    _fields_ = [
        ("_tState", _MonitorState),
        ("_tJoint", _MonitorJoint),
        ("_tTask", _MonitorTask),
        ("_tTorque", _MonitorTorque),
    ]


class _MonitoringData(C.Structure):
    # Controller sends a packed 517-byte record (see DRFL manual 2.3.2).
    _pack_ = 1
    _fields_ = [("_tCtrl", _MonitorControl), ("_tMisc", _MonitorMisc)]


class _SystemVersion(C.Structure):
    _fields_ = [
        (name, C.c_char * 32)
        for name in (
            "_szSmartTp",
            "_szController",
            "_szInterpreter",
            "_szInverter",
            "_szSafetyBoard",
            "_szRobotSerial",
            "_szRobotModel",
            "_szJTSBoard",
            "_szFlangeBoard",
        )
    ]


_MONITOR_CB = C.CFUNCTYPE(None, C.POINTER(_MonitoringData))


class DrflLibrary:
    def __init__(self, dll_dir: str):
        self.dll_dir = os.path.abspath(dll_dir)
        self._dll_dir_handle = os.add_dll_directory(self.dll_dir)
        self.dll = C.WinDLL(os.path.join(self.dll_dir, "DRFLWin64.dll"))
        self._bind()

    def _bind(self):
        d = self.dll
        d._CreateRobotControl.restype = C.c_void_p
        d._DestroyRobotControl.argtypes = [C.c_void_p]
        d._open_connection.argtypes = [C.c_void_p, C.c_char_p, C.c_uint]
        d._open_connection.restype = C.c_int
        d._close_connection.argtypes = [C.c_void_p]
        d._close_connection.restype = C.c_int
        d._get_system_version.argtypes = [C.c_void_p, C.POINTER(_SystemVersion)]
        d._get_system_version.restype = C.c_int
        d._get_robot_state.argtypes = [C.c_void_p]
        d._get_robot_state.restype = C.c_int
        d._get_robot_mode.argtypes = [C.c_void_p]
        d._get_robot_mode.restype = C.c_int
        d._get_robot_speed_mode.argtypes = [C.c_void_p]
        d._get_robot_speed_mode.restype = C.c_int
        d._get_program_state.argtypes = [C.c_void_p]
        d._get_program_state.restype = C.c_int
        d._set_on_monitoring_data.argtypes = [C.c_void_p, _MONITOR_CB]
        d._set_on_monitoring_data.restype = None


@dataclass
class RobotStatus:
    connected: bool = False
    connecting: bool = False
    error: str = ""
    ip: str = ""
    model: str = ""
    serial: str = ""
    controller: str = ""
    interpreter: str = ""
    state: int = -1
    mode: int = -1
    speed_mode: int = -1
    program_state: int = -1
    ctrl_mode: int = -1
    ctrl_space: int = -1
    rate_hz: float = 0.0
    samples: int = 0
    last_sample_wall: float = 0.0
    last_device_time: float = 0.0
    seq: int = 0


def _f(values, n=NUM_JOINT):
    return tuple(float(v) for v in values[:n])


def _field_text(raw: bytes) -> str:
    text = raw.decode(errors="ignore").strip("\x00").strip()
    return text if text.isprintable() else ""


class RobotClient:
    """One TCP monitoring client (read-only) running in its own thread."""

    def __init__(
        self,
        lib: DrflLibrary,
        name: str,
        ip: str,
        on_sample: Callable[[str, dict], None],
        on_status: Callable[[str, RobotStatus], None],
    ):
        self.lib = lib
        self.name = name
        self.ip = ip
        self.on_sample = on_sample
        self.on_status = on_status
        self.status = RobotStatus(ip=ip)
        self._thread: Optional[threading.Thread] = None
        self._stop = threading.Event()
        self._ctrl: Optional[int] = None
        self._cb = None
        self._lock = threading.RLock()
        self._sample_times: list[float] = []

    # -- lifecycle -----------------------------------------------------
    def start(self):
        self._thread = threading.Thread(target=self._run, name=f"drfl-{self.name}", daemon=True)
        self._thread.start()

    def stop(self, timeout: float = 3.0):
        self._stop.set()
        if self._thread:
            self._thread.join(timeout)

    def _run(self):
        while not self._stop.is_set():
            if self._ctrl is None:
                self._connect()
                if self._ctrl is None:
                    self._stop.wait(2.0)
                    continue

            self._poll_status()
            if time.time() - self.status.last_sample_wall > 3.0:
                self._set_error("数据中断，正在重连")
                self._disconnect()
                self._stop.wait(1.0)
                continue
            self._stop.wait(0.5)

        self._disconnect()

    # -- connection ----------------------------------------------------
    def _connect(self):
        with self._lock:
            self.status.connecting = True
            self._publish()
            ctrl = self.lib.dll._CreateRobotControl()
            if not ctrl:
                self.status.connecting = False
                self._set_error("无法创建控制句柄")
                return
            ok = self.lib.dll._open_connection(ctrl, self.ip.encode("ascii"), DEFAULT_PORT)
            if not ok:
                self.lib.dll._DestroyRobotControl(ctrl)
                self.status.connecting = False
                self._set_error(f"连接失败 {self.ip}:{DEFAULT_PORT}")
                return

            self._ctrl = ctrl
            ver = _SystemVersion()
            if self.lib.dll._get_system_version(ctrl, C.byref(ver)):
                self.status.controller = _field_text(ver._szController)
                self.status.interpreter = _field_text(ver._szInterpreter)
                self.status.model = _field_text(ver._szRobotModel)
                self.status.serial = _field_text(ver._szRobotSerial)

            self._cb = _MONITOR_CB(self._on_data)
            self.lib.dll._set_on_monitoring_data(ctrl, self._cb)

            now = time.time()
            self.status.connected = True
            self.status.connecting = False
            self.status.error = ""
            self.status.last_sample_wall = now
            self.status.samples = 0
            self.status.seq = 0
            self._sample_times.clear()
            self._sample_times.append(now)
            self._publish()

    def _disconnect(self):
        with self._lock:
            ctrl = self._ctrl
            self._ctrl = None
            self.status.connected = False
            self.status.connecting = False
            self.status.rate_hz = 0.0
            if ctrl:
                try:
                    self.lib.dll._close_connection(ctrl)
                finally:
                    self.lib.dll._DestroyRobotControl(ctrl)
            self._publish()

    def _set_error(self, text: str):
        self.status.error = text
        self._publish()

    # -- data ----------------------------------------------------------
    def _on_data(self, p):
        try:
            if not p or self._ctrl is None:
                return
            d = p.contents
            ctrl = d._tCtrl
            s = ctrl._tState
            j = ctrl._tJoint
            t = ctrl._tTask
            q = ctrl._tTorque
            m = d._tMisc

            self.status.seq += 1
            self.status.samples += 1
            now = time.time()
            self.status.last_sample_wall = now
            self.status.last_device_time = float(m._dSyncTime)
            self.status.ctrl_mode = int(s._iActualMode)
            self.status.ctrl_space = int(s._iActualSpace)
            self._sample_times.append(now)
            if len(self._sample_times) > 50:
                del self._sample_times[:-50]

            sample = {
                "t": now,
                "device_time": float(m._dSyncTime),
                "seq": self.status.seq,
                "ctrl_mode": int(s._iActualMode),
                "ctrl_space": int(s._iActualSpace),
                "robot_state": int(self.status.state),
                "robot_mode": int(self.status.mode),
                "robot_speed_mode": int(self.status.speed_mode),
                "pos": _f(j._fActualPos),
                "pos_abs": _f(j._fActualAbs),
                "vel": _f(j._fActualVel),
                "pos_err": _f(j._fActualErr),
                "target_pos": _f(j._fTargetPos),
                "target_vel": _f(j._fTargetVel),
                "tool_pos": _f(t._fActualPos[0]),
                "flange_pos": _f(t._fActualPos[1]),
                "task_vel": _f(t._fActualVel),
                "task_err": _f(t._fActualErr),
                "target_task_pos": _f(t._fTargetPos),
                "target_task_vel": _f(t._fTargetVel),
                "solution_space": int(t._iSolutionSpace),
                "rotation": tuple(float(t._fRotationMatrix[r][c]) for r in range(3) for c in range(3)),
                "dynamic_torque": _f(q._fDynamicTor),
                "joint_torque_sensor": _f(q._fActualJTS),
                "external_joint_torque": _f(q._fActualEJT),
                "external_tcp_force": _f(q._fActualETT),
                "motor_current": _f(m._fActualMC),
                "motor_temperature": _f(m._fActualMT),
                "digital_input": tuple(int(v) for v in m._iActualDI),
                "digital_output": tuple(int(v) for v in m._iActualDO),
                "brake": tuple(int(v) for v in m._iActualBK),
                "buttons": tuple(int(v) for v in m._iActualBT),
            }
            self.on_sample(self.name, sample)
        except Exception:  # never let an exception cross the native callback
            pass

    def _poll_status(self):
        with self._lock:
            if self._ctrl is None:
                return
            try:
                self.status.state = int(self.lib.dll._get_robot_state(self._ctrl))
                self.status.mode = int(self.lib.dll._get_robot_mode(self._ctrl))
                self.status.speed_mode = int(self.lib.dll._get_robot_speed_mode(self._ctrl))
                self.status.program_state = int(self.lib.dll._get_program_state(self._ctrl))
            except Exception as exc:
                self._set_error(str(exc))
                return

            times = self._sample_times
            if len(times) >= 2:
                span = times[-1] - times[0]
                if span > 0:
                    self.status.rate_hz = (len(times) - 1) / span
            self._publish()

    def _publish(self):
        try:
            self.on_status(self.name, self.status)
        except Exception:
            pass

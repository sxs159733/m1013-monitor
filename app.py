from __future__ import annotations

import copy
import csv
import json
import os
import queue
import sys
import threading
import time
import traceback
from collections import deque
from datetime import datetime
from pathlib import Path

import numpy as np
import pyqtgraph as pg
from PySide6 import QtCore, QtGui, QtWidgets

from drfl_client import (
    DEFAULT_ROBOTS,
    PROGRAM_STATE_NAMES,
    ROBOT_MODE_NAMES,
    ROBOT_STATE_NAMES,
    SPEED_MODE_NAMES,
    DrflLibrary,
    RobotClient,
    RobotStatus,
)

BASE_DIR = Path(__file__).resolve().parent
LOG_DIR = BASE_DIR / "logs"
CONFIG_PATH = BASE_DIR / "config.json"
APP_TITLE = "斗山 M1013 · 双机实时监测台"

J = 6
COLORS = ["#4fc3f7", "#ffb74d", "#81c784", "#ef7d7d", "#b58cf0", "#f2e06b"]
ROBOT_TITLES = {"R1": "机械臂 1", "R2": "机械臂 2"}

GROUPS = [
    {"key": "pos", "label": "关节位置", "names": [f"J{i}" for i in range(1, 7)], "unit": "deg"},
    {"key": "pos_abs", "label": "关节位置(绝对编码器)", "names": [f"J{i}" for i in range(1, 7)], "unit": "deg"},
    {"key": "vel", "label": "关节速度", "names": [f"J{i}" for i in range(1, 7)], "unit": "deg/s"},
    {"key": "pos_err", "label": "关节跟随误差", "names": [f"J{i}" for i in range(1, 7)], "unit": "deg"},
    {"key": "target_pos", "label": "目标关节位置", "names": [f"J{i}" for i in range(1, 7)], "unit": "deg"},
    {"key": "target_vel", "label": "目标关节速度", "names": [f"J{i}" for i in range(1, 7)], "unit": "deg/s"},
    {"key": "dynamic_torque", "label": "动态力矩", "names": [f"J{i}" for i in range(1, 7)], "unit": "Nm"},
    {"key": "joint_torque_sensor", "label": "关节力矩传感器", "names": [f"J{i}" for i in range(1, 7)], "unit": "Nm"},
    {"key": "external_joint_torque", "label": "外部关节力矩", "names": [f"J{i}" for i in range(1, 7)], "unit": "Nm"},
    {"key": "external_tcp_force", "label": "外部六维力/力矩", "names": ["Fx", "Fy", "Fz", "Tx", "Ty", "Tz"], "unit": "N / Nm"},
    {"key": "tool_pos", "label": "TCP 位置", "names": ["X", "Y", "Z", "Rx", "Ry", "Rz"], "unit": "mm / deg"},
    {"key": "flange_pos", "label": "法兰位置", "names": ["X", "Y", "Z", "Rx", "Ry", "Rz"], "unit": "mm / deg"},
    {"key": "task_vel", "label": "TCP 速度", "names": ["Vx", "Vy", "Vz", "Wx", "Wy", "Wz"], "unit": "mm/s / deg/s"},
    {"key": "task_err", "label": "任务空间跟随误差", "names": ["X", "Y", "Z", "Rx", "Ry", "Rz"], "unit": "mm / deg"},
    {"key": "target_task_pos", "label": "目标 TCP 位置", "names": ["X", "Y", "Z", "Rx", "Ry", "Rz"], "unit": "mm / deg"},
    {"key": "target_task_vel", "label": "目标 TCP 速度", "names": ["Vx", "Vy", "Vz", "Wx", "Wy", "Wz"], "unit": "mm/s / deg/s"},
    {"key": "motor_current", "label": "电机电流", "names": [f"J{i}" for i in range(1, 7)], "unit": "A"},
    {"key": "motor_temperature", "label": "电机/逆变器温度", "names": [f"J{i}" for i in range(1, 7)], "unit": "°C"},
]

PARAM_ROWS = [
    ("关节位置", "pos", "deg", 3, "增量编码器关节角"),
    ("绝对位置", "pos_abs", "deg", 3, "绝对编码器关节角"),
    ("关节速度", "vel", "deg/s", 3, ""),
    ("跟随误差", "pos_err", "deg", 3, "指令与实际位置之差"),
    ("目标位置", "target_pos", "deg", 3, ""),
    ("目标速度", "target_vel", "deg/s", 3, ""),
    ("动态力矩", "dynamic_torque", "Nm", 3, "控制器动力学模型计算值"),
    ("力矩传感器", "joint_torque_sensor", "Nm", 3, "JTS 关节力矩传感器"),
    ("外部关节力矩", "external_joint_torque", "Nm", 3, "EJT 滤波后的外部关节力矩"),
    ("电机电流", "motor_current", "A", 3, ""),
    ("温度", "motor_temperature", "°C", 2, "逆变器温度"),
    ("TCP 位置", "tool_pos", "mm/deg", 3, "基坐标系 TCP 位姿"),
    ("法兰位置", "flange_pos", "mm/deg", 3, "基坐标系法兰位姿"),
    ("TCP 速度", "task_vel", "mm/s,deg/s", 3, "基坐标系"),
    ("任务误差", "task_err", "mm/deg", 3, ""),
    ("目标 TCP 位置", "target_task_pos", "mm/deg", 3, ""),
    ("目标 TCP 速度", "target_task_vel", "mm/s,deg/s", 3, ""),
    ("外部六维力", "external_tcp_force", "N/Nm", 3, "ETT 工具坐标系外部力/力矩"),
]

ARRAY_CSV_FIELDS = [
    ("pos", ["j1", "j2", "j3", "j4", "j5", "j6"], "deg"),
    ("pos_abs", ["j1", "j2", "j3", "j4", "j5", "j6"], "deg"),
    ("vel", ["j1", "j2", "j3", "j4", "j5", "j6"], "deg/s"),
    ("pos_err", ["j1", "j2", "j3", "j4", "j5", "j6"], "deg"),
    ("target_pos", ["j1", "j2", "j3", "j4", "j5", "j6"], "deg"),
    ("target_vel", ["j1", "j2", "j3", "j4", "j5", "j6"], "deg/s"),
    ("dynamic_torque", ["j1", "j2", "j3", "j4", "j5", "j6"], "Nm"),
    ("joint_torque_sensor", ["j1", "j2", "j3", "j4", "j5", "j6"], "Nm"),
    ("external_joint_torque", ["j1", "j2", "j3", "j4", "j5", "j6"], "Nm"),
    ("external_tcp_force", ["fx", "fy", "fz", "tx", "ty", "tz"], "N,Nm"),
    ("tool_pos", ["x", "y", "z", "rx", "ry", "rz"], "mm,deg"),
    ("flange_pos", ["x", "y", "z", "rx", "ry", "rz"], "mm,deg"),
    ("task_vel", ["vx", "vy", "vz", "wx", "wy", "wz"], "mm/s,deg/s"),
    ("task_err", ["x", "y", "z", "rx", "ry", "rz"], "mm,deg"),
    ("target_task_pos", ["x", "y", "z", "rx", "ry", "rz"], "mm,deg"),
    ("target_task_vel", ["vx", "vy", "vz", "wx", "wy", "wz"], "mm/s,deg/s"),
    ("motor_current", ["j1", "j2", "j3", "j4", "j5", "j6"], "A"),
    ("motor_temperature", ["j1", "j2", "j3", "j4", "j5", "j6"], "degC"),
    ("rotation", [f"r{r}{c}" for r in range(3) for c in range(3)], ""),
    ("digital_input", [f"di{i}" for i in range(1, 7)], ""),
    ("digital_output", [f"do{i}" for i in range(1, 7)], ""),
    ("brake", [f"bk{i}" for i in range(1, 7)], ""),
    ("buttons", [f"bt{i}" for i in range(1, 6)], ""),
]

CSV_SCALARS = [
    ("robot", None),
    ("wall_time_iso", None),
    ("unix_time", "t"),
    ("device_time", "device_time"),
    ("seq", "seq"),
    ("ctrl_mode", "ctrl_mode"),
    ("ctrl_space", "ctrl_space"),
    ("robot_state", "robot_state"),
    ("robot_mode", "robot_mode"),
    ("robot_speed_mode", "robot_speed_mode"),
    ("solution_space", "solution_space"),
]


def iso_time(ts: float) -> str:
    return datetime.fromtimestamp(ts).isoformat(timespec="milliseconds")


def csv_header() -> list[str]:
    header = [name for name, _ in CSV_SCALARS]
    for key, names, unit in ARRAY_CSV_FIELDS:
        suffix = f"[{unit}]" if unit else ""
        header.extend(f"{key}_{n}{suffix}" for n in names)
    return header


def csv_row(robot: str, s: dict) -> list:
    row = [robot, iso_time(s["t"])]
    for name, key in CSV_SCALARS[2:]:
        value = s.get(key, "")
        row.append(int(value) if key in ("seq", "ctrl_mode", "ctrl_space", "robot_state",
                                         "robot_mode", "robot_speed_mode", "solution_space") else value)
    for key, _names, _unit in ARRAY_CSV_FIELDS:
        row.extend(s.get(key, ()))
    return row


def load_config() -> dict:
    cfg = {
        "robot1_ip": DEFAULT_ROBOTS["R1"],
        "robot2_ip": DEFAULT_ROBOTS["R2"],
        "auto_connect": True,
        "window_sec": 10,
        "group": "关节位置",
        "dll_dir": "",
    }
    try:
        data = json.loads(CONFIG_PATH.read_text(encoding="utf-8"))
        if isinstance(data, dict):
            cfg.update(data)
    except Exception:
        pass
    return cfg


def save_config(cfg: dict):
    try:
        CONFIG_PATH.write_text(json.dumps(cfg, ensure_ascii=False, indent=2), encoding="utf-8")
    except Exception:
        pass


def resolve_dll_dir(cfg: dict) -> str:
    candidates = []
    if cfg.get("dll_dir"):
        candidates.append(Path(cfg["dll_dir"]))
    candidates.extend([
        BASE_DIR / "drfl",
        BASE_DIR,
        BASE_DIR / "_shared_extract" / "VisualGrip_Win64_20261024_heightfix",
    ])
    for folder in candidates:
        try:
            if (folder / "DRFLWin64.dll").is_file():
                return str(folder)
        except Exception:
            continue
    folder = QtWidgets.QFileDialog.getExistingDirectory(
        None,
        "请选择包含 DRFLWin64.dll 的文件夹",
        str(BASE_DIR),
    )
    if folder:
        cfg["dll_dir"] = folder
        save_config(cfg)
        return folder
    raise FileNotFoundError("未找到 DRFLWin64.dll")


def state_color(state: int) -> str:
    if state == 1:
        return "#22c55e"
    if state == 2:
        return "#3b82f6"
    if state == 4:
        return "#f59e0b"
    if state in (3, 5, 6, 9, 10):
        return "#ef4444"
    if state in (8, 15):
        return "#f59e0b"
    return "#8a94a6"


def name_of(mapping: dict, value: int) -> str:
    return mapping.get(value, f"未知({value})") if value >= 0 else "—"


class Led(QtWidgets.QLabel):
    def __init__(self, size: int = 16):
        super().__init__("●")
        self.setFixedWidth(size)
        self.setAlignment(QtCore.Qt.AlignCenter)
        self.set_state("#5b6675", "未连接")

    def set_state(self, color: str, tip: str):
        self.setStyleSheet(f"color:{color}; font-size:{self.width()}px;")
        self.setToolTip(tip)


class StatusCard(QtWidgets.QFrame):
    def __init__(self, robot: str):
        super().__init__()
        self.robot = robot
        self.setObjectName("card")
        self.setMinimumWidth(440)
        lay = QtWidgets.QGridLayout(self)
        lay.setContentsMargins(16, 12, 16, 12)
        lay.setHorizontalSpacing(10)
        lay.setVerticalSpacing(4)

        self.title = QtWidgets.QLabel(f"{ROBOT_TITLES[robot]}")
        self.title.setObjectName("cardTitle")
        self.led = Led(14)
        self.badge = QtWidgets.QLabel("未连接")
        self.badge.setObjectName("badge")
        self.badge.setAlignment(QtCore.Qt.AlignCenter)

        self.ip = QtWidgets.QLabel("—")
        self.ip.setObjectName("muted")
        self.device = QtWidgets.QLabel("型号 — · 序列号 —")
        self.device.setObjectName("muted")
        self.ver = QtWidgets.QLabel("控制器 — · 解释器 —")
        self.ver.setObjectName("muted")
        self.state = QtWidgets.QLabel("状态 — · 模式 — · 速度 —")
        self.ctrl = QtWidgets.QLabel("控制 — · 程序 — · 解空间 —")
        self.rate = QtWidgets.QLabel("采样 — · 周期 — · 样本 —")
        self.error = QtWidgets.QLabel("")
        self.error.setObjectName("errorText")
        self.error.setWordWrap(True)
        self.error.hide()

        lay.addWidget(self.title, 0, 0)
        lay.addWidget(self.led, 0, 1)
        lay.addWidget(self.badge, 0, 2)
        lay.addWidget(self.ip, 1, 0, 1, 3)
        lay.addWidget(self.device, 2, 0, 1, 3)
        lay.addWidget(self.ver, 3, 0, 1, 3)
        lay.addWidget(self.state, 4, 0, 1, 3)
        lay.addWidget(self.ctrl, 5, 0, 1, 3)
        lay.addWidget(self.rate, 6, 0, 1, 3)
        lay.addWidget(self.error, 7, 0, 1, 3)
        lay.setColumnStretch(0, 1)

    def update_status(self, st: RobotStatus | None, ip: str):
        self.ip.setText(f"IP {ip}  ·  只读监测")
        if st is None:
            self.led.set_state("#5b6675", "未连接")
            self.badge.setText("未连接")
            self.badge.setStyleSheet("background:#2a3442;color:#9aa7b8;border-radius:4px;padding:2px 10px;font-weight:600;")
            self.device.setText("型号 — · 序列号 —")
            self.ver.setText("控制器 — · 解释器 —")
            self.state.setText("状态 — · 模式 — · 速度 —")
            self.ctrl.setText("控制 — · 程序 — · 解空间 —")
            self.rate.setText("采样 — · 周期 — · 样本 —")
            self.error.hide()
            return

        if st.connected:
            color = state_color(st.state)
            self.led.set_state(color, "已连接")
        elif st.connecting:
            self.led.set_state("#f59e0b", "连接中")
        elif st.error:
            self.led.set_state("#ef4444", st.error)
        else:
            self.led.set_state("#5b6675", "未连接")

        state_text = name_of(ROBOT_STATE_NAMES, st.state)
        self.badge.setText(state_text)
        bg = state_color(st.state) if st.connected else "#2a3442"
        fg = "#081018" if st.connected and st.state in (1, 2, 4, 3, 5, 6, 9, 10) else "#9aa7b8"
        self.badge.setStyleSheet(f"background:{bg};color:{fg};border-radius:4px;padding:2px 10px;font-weight:600;")

        self.device.setText(f"型号 {st.model or '—'} · 序列号 {st.serial or '—'}")
        self.ver.setText(f"控制器 {st.controller or '—'} · 解释器 {st.interpreter or '—'}")
        self.state.setText(
            f"状态 {state_text} · 模式 {name_of(ROBOT_MODE_NAMES, st.mode)} · "
            f"速度 {name_of(SPEED_MODE_NAMES, st.speed_mode)}"
        )
        ctrl_mode = {0: "位置控制", 1: "力矩控制"}.get(st.ctrl_mode, "—")
        ctrl_space = {1: "关节空间", 2: "任务空间"}.get(st.ctrl_space, "—")
        self.ctrl.setText(
            f"控制 {ctrl_mode}/{ctrl_space} · 程序 {name_of(PROGRAM_STATE_NAMES, st.program_state)}"
        )
        period = f"{1000.0 / st.rate_hz:.1f} ms" if st.rate_hz > 0.1 else "—"
        self.rate.setText(
            f"采样 {st.rate_hz:.1f} Hz · 周期 {period} · 样本 {st.samples:,}"
        )
        if st.error:
            self.error.setText(st.error)
            self.error.show()
        else:
            self.error.hide()


class ParamsPanel(QtWidgets.QGroupBox):
    def __init__(self, robot: str):
        super().__init__(ROBOT_TITLES[robot])
        self.robot = robot
        self._items: dict[tuple[int, int], QtWidgets.QTableWidgetItem] = {}
        self._values: dict[str, tuple] = {}

        lay = QtWidgets.QVBoxLayout(self)
        lay.setContentsMargins(10, 12, 10, 8)
        lay.setSpacing(6)

        self.status_line = QtWidgets.QLabel("未连接")
        self.status_line.setObjectName("panelStatus")
        self.status_line.setWordWrap(True)
        lay.addWidget(self.status_line)

        self.table = QtWidgets.QTableWidget(len(PARAM_ROWS), 8)
        self.table.setHorizontalHeaderLabels(
            ["参数", "1 / X", "2 / Y", "3 / Z", "4 / Rx", "5 / Ry", "6 / Rz", "单位"]
        )
        self.table.verticalHeader().setVisible(False)
        self.table.setSelectionMode(QtWidgets.QAbstractItemView.NoSelection)
        self.table.setEditTriggers(QtWidgets.QAbstractItemView.NoEditTriggers)
        self.table.setShowGrid(False)
        self.table.setAlternatingRowColors(True)
        self.table.setFocusPolicy(QtCore.Qt.NoFocus)
        hh = self.table.horizontalHeader()
        hh.setSectionResizeMode(0, QtWidgets.QHeaderView.Fixed)
        self.table.setColumnWidth(0, 118)
        for col in range(1, 7):
            hh.setSectionResizeMode(col, QtWidgets.QHeaderView.Stretch)
        hh.setSectionResizeMode(7, QtWidgets.QHeaderView.Fixed)
        self.table.setColumnWidth(7, 86)
        self.table.verticalHeader().setDefaultSectionSize(27)
        self.table.setMinimumHeight(160)
        self.table.setSizePolicy(
            QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Ignored
        )

        value_font = QtGui.QFont("Consolas", 9)
        for row, (label, key, unit, dec, tip) in enumerate(PARAM_ROWS):
            item = QtWidgets.QTableWidgetItem(label)
            if tip:
                item.setToolTip(tip)
            self.table.setItem(row, 0, item)
            for col in range(6):
                cell = QtWidgets.QTableWidgetItem("—")
                cell.setTextAlignment(QtCore.Qt.AlignRight | QtCore.Qt.AlignVCenter)
                cell.setFont(value_font)
                self.table.setItem(row, col + 1, cell)
                self._items[(row, col)] = cell
            unit_item = QtWidgets.QTableWidgetItem(unit)
            unit_item.setForeground(QtGui.QColor("#7f8ea3"))
            self.table.setItem(row, 7, unit_item)

        lay.addWidget(self.table, 1)

        self.io_line = QtWidgets.QLabel("DI — · DO — · 抱闸 — · 按钮 —")
        self.io_line.setObjectName("mutedMono")
        self.io_line.setWordWrap(True)
        lay.addWidget(self.io_line)

        self.rot_label = QtWidgets.QLabel("旋转矩阵 —")
        self.rot_label.setObjectName("mutedMono")
        self.rot_label.setWordWrap(True)
        lay.addWidget(self.rot_label)

    def update_data(self, sample: dict | None, st: RobotStatus | None):
        if sample is None:
            if st and st.error:
                self.status_line.setText(f"<span style='color:#ef4444'>{st.error}</span>")
            elif st and st.connected:
                self.status_line.setText("已连接，等待数据…")
            else:
                self.status_line.setText("未连接")
            for item in self._items.values():
                if item.text() != "—":
                    item.setText("—")
            self.io_line.setText("DI — · DO — · 抱闸 — · 按钮 —")
            self.rot_label.setText("旋转矩阵 —")
            return

        mode = {0: "位置控制", 1: "力矩控制"}.get(sample.get("ctrl_mode", -1), "—")
        space = {1: "关节空间", 2: "任务空间"}.get(sample.get("ctrl_space", -1), "—")
        state_text = name_of(ROBOT_STATE_NAMES, sample.get("robot_state", -1))
        self.status_line.setText(
            f"<span style='color:{state_color(sample.get('robot_state', -1))}'>●</span> "
            f"{state_text} · {mode}/{space} · 解空间 {sample.get('solution_space', '—')} · "
            f"设备时间 {sample.get('device_time', 0):.3f}"
        )

        for row, (_label, key, _unit, dec, _tip) in enumerate(PARAM_ROWS):
            values = sample.get(key)
            if not values:
                continue
            for col in range(6):
                text = f"{values[col]:.{dec}f}"
                item = self._items[(row, col)]
                if item.text() != text:
                    item.setText(text)

        di = " ".join(str(v) for v in sample.get("digital_input", ()))
        do = " ".join(str(v) for v in sample.get("digital_output", ()))
        bk = " ".join(str(v) for v in sample.get("brake", ()))
        bt = " ".join(str(v) for v in sample.get("buttons", ()))
        self.io_line.setText(f"DI {di} · DO {do} · 抱闸 {bk} · 按钮 {bt}")

        r = sample.get("rotation")
        if r and len(r) == 9:
            self.rot_label.setText(
                "R = [%6.3f %6.3f %6.3f]  [%6.3f %6.3f %6.3f]  [%6.3f %6.3f %6.3f]"
                % tuple(r)
            )


class CsvRecorder:
    def __init__(self, root: Path):
        self.root = root
        self.is_recording = False
        self.folder: Path | None = None
        self.paths: dict[str, Path] = {}
        self.counts = {"R1": 0, "R2": 0}
        self.started_at = 0.0
        self._queue: queue.Queue = queue.Queue()
        self._thread: threading.Thread | None = None
        self._files: dict[str, object] = {}
        self._writers: dict[str, csv.writer] = {}
        self._last_flush = 0.0
        self._lock = threading.Lock()

    def start(self, ips: dict[str, str]):
        with self._lock:
            if self.is_recording:
                return
            stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            self.folder = self.root / f"run_{stamp}"
            self.folder.mkdir(parents=True, exist_ok=True)
            header = csv_header()
            for robot in ("R1", "R2"):
                path = self.folder / f"{robot}.csv"
                f = open(path, "w", newline="", encoding="utf-8-sig")
                w = csv.writer(f)
                w.writerow(header)
                f.flush()
                self.paths[robot] = path
                self._files[robot] = f
                self._writers[robot] = w
                self.counts[robot] = 0
            (self.folder / "session.json").write_text(
                json.dumps(
                    {
                        "started_at": datetime.now().isoformat(timespec="seconds"),
                        "robots": ips,
                        "note": "Read-only DRFL monitoring stream. Units are embedded in CSV headers.",
                    },
                    ensure_ascii=False,
                    indent=2,
                ),
                encoding="utf-8",
            )
            self.started_at = time.time()
            self.is_recording = True
            self._last_flush = time.time()
            self._thread = threading.Thread(target=self._run, name="csv-recorder", daemon=True)
            self._thread.start()

    def put(self, robot: str, sample: dict):
        if self.is_recording:
            try:
                self._queue.put_nowait((robot, sample))
            except queue.Full:
                pass

    def stop(self):
        with self._lock:
            if not self.is_recording:
                return
            self.is_recording = False
            self._queue.put(None)
            if self._thread:
                self._thread.join(3.0)
            for robot, f in self._files.items():
                try:
                    f.flush()
                    f.close()
                except Exception:
                    pass
            self._files.clear()
            self._writers.clear()

    def _run(self):
        while True:
            try:
                item = self._queue.get(timeout=0.5)
            except queue.Empty:
                item = "tick"
            if item is None:
                break
            if item != "tick":
                robot, sample = item
                writer = self._writers.get(robot)
                if writer is not None:
                    try:
                        writer.writerow(csv_row(robot, sample))
                        self.counts[robot] += 1
                    except Exception:
                        pass
            now = time.time()
            if now - self._last_flush >= 1.0:
                for f in self._files.values():
                    try:
                        f.flush()
                    except Exception:
                        pass
                self._last_flush = now
        for f in self._files.values():
            try:
                f.flush()
                f.close()
            except Exception:
                pass
        self._files.clear()
        self._writers.clear()


QSS = """
QWidget {
    background: #0b0f14;
    color: #d7e1ee;
    font-family: "Microsoft YaHei UI", "Segoe UI";
    font-size: 9pt;
}
QMainWindow { background: #0b0f14; }
QFrame#card {
    background: #121820;
    border: 1px solid #223041;
    border-radius: 10px;
}
QLabel#cardTitle { font-size: 13pt; font-weight: 700; color: #eaf2fb; }
QLabel#muted { color: #8b9aad; }
QLabel#mutedMono { color: #8b9aad; font-family: Consolas, "Microsoft YaHei UI"; font-size: 8.5pt; }
QLabel#errorText { color: #ff7b7b; font-size: 8.5pt; }
QLabel#panelStatus { color: #a9b8c9; font-size: 9pt; }
QGroupBox {
    background: #121820;
    border: 1px solid #223041;
    border-radius: 10px;
    margin-top: 14px;
    padding-top: 8px;
    font-weight: 600;
}
QGroupBox::title {
    subcontrol-origin: margin;
    left: 14px;
    padding: 0 6px;
    color: #8fb3d9;
}
QPushButton {
    background: #1a2431;
    border: 1px solid #2d3d50;
    border-radius: 6px;
    padding: 6px 14px;
    color: #d7e1ee;
}
QPushButton:hover { background: #223041; border-color: #3b5573; }
QPushButton:pressed { background: #16202c; }
QPushButton:checked { background: #2a1a1d; border-color: #7f2d35; color: #ffb4b4; }
QPushButton#accent { background: #1d4f8f; border-color: #2f81f7; color: #ffffff; font-weight: 600; }
QPushButton#accent:hover { background: #2462ad; }
QPushButton#danger:checked { background: #6e1f25; border-color: #ef4444; color: #ffe1e1; }
QLineEdit {
    background: #0e141b;
    border: 1px solid #2d3d50;
    border-radius: 6px;
    padding: 5px 8px;
    color: #d7e1ee;
    selection-background-color: #2f81f7;
}
QLineEdit:focus { border-color: #2f81f7; }
QComboBox {
    background: #121a24;
    border: 1px solid #2d3d50;
    border-radius: 6px;
    padding: 5px 10px;
    min-width: 150px;
}
QComboBox::drop-down { border: none; width: 20px; }
QComboBox QAbstractItemView {
    background: #121a24;
    border: 1px solid #2d3d50;
    selection-background-color: #1d4f8f;
}
QCheckBox { spacing: 6px; color: #aebccd; }
QCheckBox::indicator {
    width: 14px; height: 14px; border-radius: 3px;
    border: 1px solid #3b4c61; background: #0e141b;
}
QCheckBox::indicator:checked { background: #2f81f7; border-color: #2f81f7; }
QTabWidget::pane { border: 1px solid #223041; border-radius: 8px; top: -1px; background: #0e141b; }
QTabBar::tab {
    background: transparent;
    padding: 8px 18px;
    margin-right: 4px;
    color: #8b9aad;
    border-bottom: 2px solid transparent;
}
QTabBar::tab:selected { color: #eaf2fb; border-bottom: 2px solid #2f81f7; }
QTabBar::tab:hover { color: #d7e1ee; }
QTableWidget {
    background: #0e141b;
    alternate-background-color: #101822;
    border: none;
    gridline-color: transparent;
    color: #cfdae7;
}
QHeaderView::section {
    background: #18222e;
    color: #8fb3d9;
    border: none;
    padding: 5px 6px;
    font-weight: 600;
}
QPlainTextEdit {
    background: #0b1016;
    border: 1px solid #223041;
    border-radius: 6px;
    color: #a9b8c9;
    font-family: Consolas, "Microsoft YaHei UI";
    font-size: 8.5pt;
}
QStatusBar { color: #7f8ea3; }
QLabel#sectionHint { color: #6f8095; font-size: 8.5pt; }
"""


class MainWindow(QtWidgets.QMainWindow):
    def __init__(self, lib: DrflLibrary, cfg: dict):
        super().__init__()
        self.lib = lib
        self.cfg = cfg
        self.setWindowTitle(APP_TITLE)
        self.resize(1680, 980)
        self.setMinimumSize(1280, 760)

        self.sample_q: queue.Queue = queue.Queue()
        self.status_q: queue.Queue = queue.Queue()
        self.history = {"R1": deque(maxlen=6000), "R2": deque(maxlen=6000)}
        self.latest: dict[str, dict] = {}
        self.status: dict[str, RobotStatus] = {}
        self.clients: dict[str, RobotClient] = {}
        self.paused = False
        self.window_sec = int(cfg.get("window_sec", 10))
        self.channel_state: dict[str, list[bool]] = {}
        self.curves: dict[str, list] = {"R1": [], "R2": []}
        self._plot_ref_time = time.time()
        self.recorder = CsvRecorder(LOG_DIR)
        self._last_params_update = 0.0

        self._build_ui()
        self._apply_group()
        self._update_record_info()

        self.frame_timer = QtCore.QTimer(self)
        self.frame_timer.timeout.connect(self._on_frame)
        self.frame_timer.start(80)

        self.slow_timer = QtCore.QTimer(self)
        self.slow_timer.timeout.connect(self._on_slow)
        self.slow_timer.start(300)

        self.log("监测台已启动：两类斗山 M1013，DRFL 只读监测，不发送运动/伺服指令。")
        self.log(f"DRFL: {lib.dll_dir}")

    # ------------------------------------------------------------------ UI
    def _build_ui(self):
        central = QtWidgets.QWidget()
        self.setCentralWidget(central)
        root = QtWidgets.QVBoxLayout(central)
        root.setContentsMargins(14, 12, 14, 8)
        root.setSpacing(10)

        root.addWidget(self._build_toolbar())
        root.addLayout(self._build_cards())

        self.tabs = QtWidgets.QTabWidget()
        self.tabs.addTab(self._build_curves_tab(), "实时曲线")
        self.tabs.addTab(self._build_params_tab(), "全部参数")
        self.tabs.addTab(self._build_record_tab(), "记录 / 导出")
        root.addWidget(self.tabs, 1)

        self.statusBar().showMessage("只读监测 · 不请求控制权 · 不发伺服/运动指令")

    def _build_toolbar(self) -> QtWidgets.QWidget:
        bar = QtWidgets.QFrame()
        bar.setObjectName("card")
        lay = QtWidgets.QHBoxLayout(bar)
        lay.setContentsMargins(16, 10, 16, 10)
        lay.setSpacing(10)

        title_box = QtWidgets.QVBoxLayout()
        title_box.setSpacing(0)
        title = QtWidgets.QLabel("DOOSAN M1013")
        title.setObjectName("cardTitle")
        sub = QtWidgets.QLabel("双机位姿 / 速度 / 力矩实时监测")
        sub.setObjectName("muted")
        title_box.addWidget(title)
        title_box.addWidget(sub)
        lay.addLayout(title_box)
        lay.addSpacing(18)

        self.ip_edits: dict[str, QtWidgets.QLineEdit] = {}
        self.connect_buttons: dict[str, QtWidgets.QPushButton] = {}
        for robot in ("R1", "R2"):
            lab = QtWidgets.QLabel(robot)
            lab.setStyleSheet("color:#8fb3d9;font-weight:700;")
            edit = QtWidgets.QLineEdit(self.cfg.get(f"robot{robot[-1]}_ip", DEFAULT_ROBOTS[robot]))
            edit.setFixedWidth(132)
            edit.setPlaceholderText("控制器 IP")
            btn = QtWidgets.QPushButton("连接")
            btn.setFixedWidth(72)
            btn.clicked.connect(lambda _=False, r=robot: self._toggle_robot(r))
            self.ip_edits[robot] = edit
            self.connect_buttons[robot] = btn
            lay.addWidget(lab)
            lay.addWidget(edit)
            lay.addWidget(btn)

        all_btn = QtWidgets.QPushButton("全部连接")
        all_btn.setObjectName("accent")
        all_btn.clicked.connect(self.connect_all)
        all_off = QtWidgets.QPushButton("全部断开")
        all_off.clicked.connect(self.disconnect_all)
        lay.addWidget(all_btn)
        lay.addWidget(all_off)
        lay.addStretch(1)

        self.pause_btn = QtWidgets.QPushButton("暂停曲线")
        self.pause_btn.setCheckable(True)
        self.pause_btn.toggled.connect(self._toggle_pause)
        lay.addWidget(self.pause_btn)

        self.record_btn = QtWidgets.QPushButton("● 开始记录")
        self.record_btn.setObjectName("danger")
        self.record_btn.setCheckable(True)
        self.record_btn.toggled.connect(self._toggle_record)
        lay.addWidget(self.record_btn)

        open_btn = QtWidgets.QPushButton("打开日志")
        open_btn.clicked.connect(self._open_log_folder)
        lay.addWidget(open_btn)
        return bar

    def _build_cards(self) -> QtWidgets.QHBoxLayout:
        lay = QtWidgets.QHBoxLayout()
        lay.setSpacing(10)
        self.cards = {robot: StatusCard(robot) for robot in ("R1", "R2")}
        for card in self.cards.values():
            lay.addWidget(card, 1)
        return lay

    def _build_curves_tab(self) -> QtWidgets.QWidget:
        tab = QtWidgets.QWidget()
        lay = QtWidgets.QVBoxLayout(tab)
        lay.setContentsMargins(10, 10, 10, 8)
        lay.setSpacing(8)

        controls = QtWidgets.QHBoxLayout()
        controls.setSpacing(10)
        controls.addWidget(QtWidgets.QLabel("信号"))
        self.group_combo = QtWidgets.QComboBox()
        self.group_combo.addItems([g["label"] for g in GROUPS])
        idx = max(0, self.group_combo.findText(self.cfg.get("group", "关节位置")))
        self.group_combo.setCurrentIndex(idx)
        self.group_combo.currentIndexChanged.connect(self._apply_group)
        controls.addWidget(self.group_combo)
        controls.addSpacing(12)

        self.channel_boxes: list[QtWidgets.QCheckBox] = []
        for i in range(6):
            cb = QtWidgets.QCheckBox(f"J{i + 1}")
            cb.setChecked(True)
            cb.toggled.connect(self._on_channel_toggle)
            self.channel_boxes.append(cb)
            controls.addWidget(cb)
        controls.addSpacing(12)

        controls.addWidget(QtWidgets.QLabel("时间窗"))
        self.window_combo = QtWidgets.QComboBox()
        for sec in (5, 10, 20, 30, 60, 120, 300):
            self.window_combo.addItem(f"{sec} s", sec)
        wi = max(0, self.window_combo.findData(self.window_sec))
        self.window_combo.setCurrentIndex(wi)
        self.window_combo.currentIndexChanged.connect(self._on_window_change)
        controls.addWidget(self.window_combo)

        self.autoscale_cb = QtWidgets.QCheckBox("自动缩放 Y")
        self.autoscale_cb.setChecked(True)
        self.autoscale_cb.toggled.connect(self._on_autoscale)
        controls.addWidget(self.autoscale_cb)
        controls.addStretch(1)
        self.curve_hint = QtWidgets.QLabel("")
        self.curve_hint.setObjectName("sectionHint")
        controls.addWidget(self.curve_hint)
        lay.addLayout(controls)

        pg.setConfigOptions(antialias=True, background="#0b0f14", foreground="#b9c7d8")
        self.plot_widget = pg.GraphicsLayoutWidget()
        self.plot_widget.setBackground("#0b0f14")
        self.plot_r1 = self.plot_widget.addPlot(row=0, col=0)
        self.plot_r2 = self.plot_widget.addPlot(row=1, col=0)
        self.plot_r2.setXLink(self.plot_r1)
        self.plot_widget.ci.layout.setRowStretchFactor(0, 1)
        self.plot_widget.ci.layout.setRowStretchFactor(1, 1)
        lay.addWidget(self.plot_widget, 1)

        hint = QtWidgets.QLabel(
            "提示：曲线为控制器只读监测流；力矩单位 Nm，TCP 位置单位 mm/deg。"
            "M1013 的外部六维力由关节力矩估计，奇异位形附近不可作为测量值使用。"
        )
        hint.setObjectName("sectionHint")
        hint.setWordWrap(True)
        lay.addWidget(hint)
        return tab

    def _build_params_tab(self) -> QtWidgets.QWidget:
        tab = QtWidgets.QWidget()
        lay = QtWidgets.QVBoxLayout(tab)
        lay.setContentsMargins(10, 10, 10, 8)
        lay.setSpacing(8)
        row = QtWidgets.QHBoxLayout()
        row.setSpacing(8)
        self.params_panels = {robot: ParamsPanel(robot) for robot in ("R1", "R2")}
        for panel in self.params_panels.values():
            row.addWidget(panel, 1)
        lay.addLayout(row, 1)
        note = QtWidgets.QLabel(
            "全部字段来自 MONITORING_DATA（517 字节）：关节 INC/ABS 位置、速度、跟随误差、"
            "目标值，任务空间工具/法兰位姿，动态力矩、JTS 力矩传感器、EJT 外部关节力矩、"
            "ETT 外部六维力，电流、温度、DI/DO、抱闸与按钮。"
        )
        note.setWordWrap(True)
        note.setObjectName("sectionHint")
        lay.addWidget(note)
        return tab

    def _build_record_tab(self) -> QtWidgets.QWidget:
        tab = QtWidgets.QWidget()
        lay = QtWidgets.QVBoxLayout(tab)
        lay.setContentsMargins(10, 10, 10, 8)
        lay.setSpacing(10)

        box = QtWidgets.QGroupBox("CSV 数据记录")
        bl = QtWidgets.QGridLayout(box)
        bl.setContentsMargins(14, 16, 14, 12)
        bl.setHorizontalSpacing(12)
        bl.setVerticalSpacing(8)

        self.rec_state = QtWidgets.QLabel("未记录")
        self.rec_state.setStyleSheet("color:#8b9aad;font-weight:600;")
        self.rec_paths = QtWidgets.QLabel("—")
        self.rec_paths.setObjectName("mutedMono")
        self.rec_paths.setWordWrap(True)
        self.rec_stats = QtWidgets.QLabel("样本 — · 时长 — · 文件 —")
        self.rec_stats.setObjectName("muted")

        self.rec_start_btn = QtWidgets.QPushButton("开始记录")
        self.rec_start_btn.setObjectName("accent")
        self.rec_start_btn.clicked.connect(lambda: self.record_btn.setChecked(True))
        self.rec_stop_btn = QtWidgets.QPushButton("停止记录")
        self.rec_stop_btn.clicked.connect(lambda: self.record_btn.setChecked(False))
        folder_btn = QtWidgets.QPushButton("打开日志文件夹")
        folder_btn.clicked.connect(self._open_log_folder)

        bl.addWidget(self.rec_state, 0, 0, 1, 2)
        bl.addWidget(self.rec_paths, 1, 0, 1, 2)
        bl.addWidget(self.rec_stats, 2, 0, 1, 2)
        bl.addWidget(self.rec_start_btn, 0, 2)
        bl.addWidget(self.rec_stop_btn, 1, 2)
        bl.addWidget(folder_btn, 2, 2)
        bl.setColumnStretch(0, 1)
        lay.addWidget(box)

        export_box = QtWidgets.QGroupBox("导出当前曲线窗口")
        el = QtWidgets.QHBoxLayout(export_box)
        el.setContentsMargins(14, 16, 14, 12)
        el.addWidget(QtWidgets.QLabel("把当前显示的时间窗（两台臂）导出为 CSV"))
        el.addStretch(1)
        exp_btn = QtWidgets.QPushButton("导出…")
        exp_btn.clicked.connect(self._export_window)
        el.addWidget(exp_btn)
        lay.addWidget(export_box)

        log_box = QtWidgets.QGroupBox("事件日志")
        ll = QtWidgets.QVBoxLayout(log_box)
        ll.setContentsMargins(10, 14, 10, 10)
        self.event_log = QtWidgets.QPlainTextEdit()
        self.event_log.setReadOnly(True)
        self.event_log.setMaximumBlockCount(2000)
        ll.addWidget(self.event_log)
        lay.addWidget(log_box, 1)
        return tab

    # ------------------------------------------------------------- actions
    def log(self, text: str):
        stamp = datetime.now().strftime("%H:%M:%S")
        self.event_log.appendPlainText(f"[{stamp}] {text}")
        print(f"[{stamp}] {text}")

    def _enqueue_sample(self, robot: str, sample: dict):
        self.sample_q.put((robot, sample))

    def _enqueue_status(self, robot: str, st: RobotStatus):
        self.status_q.put((robot, copy.copy(st)))

    def _current_ips(self) -> dict[str, str]:
        return {r: self.ip_edits[r].text().strip() for r in ("R1", "R2")}

    def _toggle_robot(self, robot: str):
        if robot in self.clients:
            self.disconnect_robot(robot)
        else:
            self.connect_robot(robot)

    def connect_robot(self, robot: str):
        if robot in self.clients:
            return
        ip = self.ip_edits[robot].text().strip()
        if not ip:
            self.statusBar().showMessage(f"{robot} 未填写 IP", 4000)
            return
        self.log(f"{robot} 连接 {ip}:12345")
        worker = RobotClient(
            self.lib,
            robot,
            ip,
            self._enqueue_sample,
            self._enqueue_status,
        )
        self.clients[robot] = worker
        worker.start()
        self.connect_buttons[robot].setText("断开")
        self.cfg[f"robot{robot[-1]}_ip"] = ip
        save_config(self.cfg)

    def disconnect_robot(self, robot: str):
        worker = self.clients.pop(robot, None)
        if worker:
            self.log(f"{robot} 主动断开")
            worker.stop()
        self.connect_buttons[robot].setText("连接")
        self.status.pop(robot, None)
        self.latest.pop(robot, None)
        self.statusBar().showMessage(f"{robot} 已断开", 3000)

    def connect_all(self):
        for robot in ("R1", "R2"):
            if robot not in self.clients:
                self.connect_robot(robot)

    def disconnect_all(self):
        for robot in list(self.clients):
            self.disconnect_robot(robot)

    def _toggle_pause(self, checked: bool):
        self.paused = checked
        self.pause_btn.setText("继续曲线" if checked else "暂停曲线")
        self.curve_hint.setText("曲线已暂停（数据仍在采集/记录）" if checked else "")
        self.log("曲线暂停" if checked else "曲线继续")

    def _toggle_record(self, checked: bool):
        if checked and not self.recorder.is_recording:
            try:
                self.recorder.start(self._current_ips())
                self.record_btn.setText("■ 停止记录")
                self.rec_state.setText("● 正在记录")
                self.rec_state.setStyleSheet("color:#ff7b7b;font-weight:700;")
                self.log(f"开始记录：{self.recorder.folder}")
                self._update_record_info()
            except Exception as exc:
                self.record_btn.setChecked(False)
                QtWidgets.QMessageBox.warning(self, "记录失败", str(exc))
        elif not checked and self.recorder.is_recording:
            self.recorder.stop()
            self.record_btn.setText("● 开始记录")
            self.rec_state.setText("已停止")
            self.rec_state.setStyleSheet("color:#8b9aad;font-weight:600;")
            self.log(f"记录停止：{self.recorder.folder}")
            self._update_record_info()

    def _open_log_folder(self):
        LOG_DIR.mkdir(parents=True, exist_ok=True)
        try:
            os.startfile(str(LOG_DIR))
        except Exception as exc:
            self.statusBar().showMessage(f"无法打开目录：{exc}", 5000)

    def _export_window(self):
        if not any(self.history[r] for r in ("R1", "R2")):
            self.statusBar().showMessage("暂无数据可导出", 4000)
            return
        group = GROUPS[self.group_combo.currentIndex()]
        default = BASE_DIR / f"curve_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
        path, _ = QtWidgets.QFileDialog.getSaveFileName(self, "导出当前窗口", str(default), "CSV (*.csv)")
        if not path:
            return
        now = time.time()
        cutoff = now - self.window_sec
        try:
            with open(path, "w", newline="", encoding="utf-8-sig") as f:
                w = csv.writer(f)
                w.writerow(["robot", "wall_time_iso", "unix_time"] + group["names"])
                for robot in ("R1", "R2"):
                    for s in self.history[robot]:
                        if s["t"] < cutoff:
                            continue
                        w.writerow([robot, iso_time(s["t"]), f"{s['t']:.6f}"]
                                   + [f"{v:.6f}" for v in s.get(group["key"], ())])
            self.log(f"导出曲线窗口：{path}")
            self.statusBar().showMessage(f"已导出 {path}", 6000)
        except Exception as exc:
            QtWidgets.QMessageBox.warning(self, "导出失败", str(exc))

    # -------------------------------------------------------------- plotting
    def _apply_group(self, *_args):
        group = GROUPS[self.group_combo.currentIndex()]
        state = self.channel_state.setdefault(group["key"], [True] * 6)
        self.cfg["group"] = group["label"]
        for name, plot in (("R1", self.plot_r1), ("R2", self.plot_r2)):
            plot.clear()
            legend = plot.addLegend(offset=(10, 8))
            try:
                legend.setBrush(pg.mkBrush(10, 15, 22, 190))
                legend.setPen(pg.mkPen("#263241"))
            except Exception:
                pass
            plot.setLabel("left", f'{group["label"]} [{group["unit"]}]')
            plot.setLabel("bottom", "时间 [s]  （0 = 现在）")
            plot.showGrid(x=True, y=True, alpha=0.22)
            plot.setMenuEnabled(False)
            plot.setTitle(
                f'<span style="color:#8fb3d9;font-size:11pt">{ROBOT_TITLES[name]}'
                f' · {group["label"]}</span>'
            )
            curves = []
            for i in range(6):
                curve = plot.plot(
                    pen=pg.mkPen(COLORS[i], width=1.8),
                    name=group["names"][i],
                )
                curve.setDownsampling(auto=True, method="peak")
                curve.setClipToView(True)
                curve.setVisible(state[i])
                curves.append(curve)
            self.curves[name] = curves
        for i, cb in enumerate(self.channel_boxes):
            cb.blockSignals(True)
            cb.setText(group["names"][i])
            cb.setChecked(state[i])
            cb.blockSignals(False)
        self._on_autoscale(self.autoscale_cb.isChecked())
        self._update_plots(force=True)

    def _on_channel_toggle(self, _checked: bool):
        group = GROUPS[self.group_combo.currentIndex()]
        state = [cb.isChecked() for cb in self.channel_boxes]
        self.channel_state[group["key"]] = state
        for name in ("R1", "R2"):
            for i, curve in enumerate(self.curves[name]):
                curve.setVisible(state[i])
        self._update_plots(force=True)

    def _on_window_change(self, *_args):
        self.window_sec = int(self.window_combo.currentData())
        self.cfg["window_sec"] = self.window_sec
        self._update_plots(force=True)

    def _on_autoscale(self, checked: bool):
        for plot in (self.plot_r1, self.plot_r2):
            vb = plot.getViewBox()
            vb.enableAutoRange(axis="y", enable=checked)
            if not checked:
                vb.disableAutoRange(axis="y")

    def _drain(self):
        try:
            while True:
                robot, sample = self.sample_q.get_nowait()
                self.history[robot].append(sample)
                self.latest[robot] = sample
                if self.recorder.is_recording:
                    self.recorder.put(robot, sample)
        except queue.Empty:
            pass
        try:
            while True:
                robot, st = self.status_q.get_nowait()
                self.status[robot] = st
        except queue.Empty:
            pass

    def _on_frame(self):
        self._drain()
        if not self.paused:
            self._update_plots()

    def _update_plots(self, force: bool = False):
        group = GROUPS[self.group_combo.currentIndex()]
        now = time.time()
        self._plot_ref_time = now
        window = self.window_sec
        cutoff = now - window
        for robot, plot in (("R1", self.plot_r1), ("R2", self.plot_r2)):
            data = self.history[robot]
            if not data:
                continue
            samples = [s for s in data if s["t"] >= cutoff]
            if len(samples) < 1:
                continue
            xs = np.fromiter((s["t"] - now for s in samples), dtype=float, count=len(samples))
            for i, curve in enumerate(self.curves[robot]):
                if not curve.isVisible():
                    continue
                ys = np.fromiter((s[group["key"]][i] for s in samples), dtype=float, count=len(samples))
                curve.setData(xs, ys)
            plot.setXRange(-window, 0, padding=0.0)

    def _on_slow(self):
        self._drain()
        for robot in ("R1", "R2"):
            worker = self.clients.get(robot)
            st = self.status.get(robot)
            if worker is None and st is not None and st.connected:
                continue
            self.cards[robot].update_status(st, self.ip_edits[robot].text().strip())
            self.params_panels[robot].update_data(
                self.latest.get(robot), st
            )
            if worker is None:
                self.connect_buttons[robot].setText("连接")
            elif st and st.connected:
                self.connect_buttons[robot].setText("断开")
            else:
                self.connect_buttons[robot].setText("取消" if st and st.connecting else "断开")
        self._update_record_info()

    def _update_record_info(self):
        r = self.recorder
        if r.is_recording and r.folder:
            self.rec_paths.setText(
                f"{r.paths.get('R1', '—')}\n{r.paths.get('R2', '—')}"
            )
            duration = time.time() - r.started_at
            size = 0
            for p in r.paths.values():
                try:
                    size += p.stat().st_size
                except Exception:
                    pass
            self.rec_stats.setText(
                f"样本 R1 {r.counts['R1']:,} · R2 {r.counts['R2']:,} · "
                f"时长 {duration:.0f} s · 文件 {size / 1024:.1f} KB"
            )
        elif r.folder:
            self.rec_paths.setText(f"上次：{r.folder}")
            self.rec_stats.setText(
                f"样本 R1 {r.counts['R1']:,} · R2 {r.counts['R2']:,} · 已停止"
            )
        else:
            self.rec_paths.setText("尚未开始记录")
            self.rec_stats.setText("样本 — · 时长 — · 文件 —")

    # -------------------------------------------------------------- shutdown
    def closeEvent(self, event: QtGui.QCloseEvent):
        self.cfg["robot1_ip"] = self.ip_edits["R1"].text().strip()
        self.cfg["robot2_ip"] = self.ip_edits["R2"].text().strip()
        self.cfg["window_sec"] = self.window_sec
        self.cfg["group"] = GROUPS[self.group_combo.currentIndex()]["label"]
        save_config(self.cfg)
        if self.recorder.is_recording:
            self.recorder.stop()
        for worker in list(self.clients.values()):
            worker.stop()
        self.clients.clear()
        event.accept()


def install_excepthook():
    def hook(exc_type, exc_value, exc_tb):
        text = "".join(traceback.format_exception(exc_type, exc_value, exc_tb))
        try:
            (BASE_DIR / "app_error.log").write_text(
                f"{datetime.now().isoformat()}\n{text}", encoding="utf-8"
            )
        except Exception:
            pass
        print(text, file=sys.stderr)
        app = QtWidgets.QApplication.instance()
        if app is not None:
            QtWidgets.QMessageBox.critical(None, "程序异常", text[-2000:])

    sys.excepthook = hook


def main() -> int:
    QtWidgets.QApplication.setHighDpiScaleFactorRoundingPolicy(
        QtCore.Qt.HighDpiScaleFactorRoundingPolicy.PassThrough
    )
    app = QtWidgets.QApplication(sys.argv)
    app.setStyle("Fusion")
    app.setFont(QtGui.QFont("Microsoft YaHei UI", 9))
    app.setStyleSheet(QSS)
    install_excepthook()

    cfg = load_config()
    try:
        lib = DrflLibrary(resolve_dll_dir(cfg))
    except Exception as exc:
        QtWidgets.QMessageBox.critical(
            None,
            "DRFL 加载失败",
            f"无法加载 DRFLWin64.dll。\n\n{exc}\n\n"
            "请把 DRFLWin64.dll、PocoNet64.dll、PocoFoundation64.dll、"
            "msvcp100.dll、msvcr100.dll 放到程序的 drfl 文件夹，"
            "或选择原始 VisualGrip 文件夹。",
        )
        return 1

    win = MainWindow(lib, cfg)
    win.show()
    if cfg.get("auto_connect", True):
        QtCore.QTimer.singleShot(500, win.connect_all)
    return app.exec()


if __name__ == "__main__":
    sys.exit(main())

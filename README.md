# 斗山 M1013 · 双机实时监测台

基于 Doosan DRFL（Windows）的 M1013 机械臂输出参数实时监控工具，支持**双机同时监测**。纯只读：不发送任何伺服、模式、运动或 I/O 指令，仅订阅控制器的监控数据流。

![Python](https://img.shields.io/badge/Python-3.10%2B-blue) ![Platform](https://img.shields.io/badge/Platform-Windows%20x64-lightgrey) ![UI](https://img.shields.io/badge/UI-PySide6%20%2B%20pyqtgraph-green)

## 功能

- **双机实时监测**：R1 / R2 两台 M1013 独立连接、独立线程，状态卡片显示连接状态、机器人状态/模式、采样率（Hz）、周期与样本计数
- **18 组参数曲线**（pyqtgraph 实时绘图，可调时间窗）：
  - 关节侧：关节位置（增量/绝对编码器）、关节速度、跟随误差、目标位置/速度、动态力矩、JTS 关节力矩传感器、EJT 外部关节力矩、电机电流、电机/逆变器温度
  - 任务侧：TCP 位置、法兰位置、TCP 速度、任务空间跟随误差、目标 TCP 位置/速度、ETT 外部六维力/力矩
- **参数数值面板**：全部参数实时数值表格
- **记录 / 导出**：一键录制全量数据到 `logs/run_<时间戳>/`，每台机器人一个 CSV（含单位表头）+ `session.json` 会话元信息
- **自动重连**：数据中断 3 秒自动断开重连；连接失败自动重试
- **配置持久化**：IP、时间窗、默认参数组、DLL 目录保存在 `config.json`

## 环境要求

- Windows x64，Python 3.10+
- 依赖：`PySide6`、`pyqtgraph`、`numpy`（见 `requirements.txt`）
- **DRFL 运行库**（本仓库不包含，需自行获取，见下节）

## 获取 DRFL 运行库

出于许可考虑，本仓库**不分发**斗山官方 DLL。请从 [斗山机器人开发者网站](https://doosanrobotics.com/developer) 下载 DRFL Windows 包（或从已有的 VisualGrip / Doosan SDK 安装中提取），将以下文件放入项目根目录的 `drfl/` 文件夹：

```
drfl/
├── DRFLWin64.dll        # 必需
├── PocoFoundation64.dll # DRFL 依赖
├── PocoNet64.dll        # DRFL 依赖
├── msvcp100.dll         # VC++ 2010 运行库
└── msvcr100.dll         # VC++ 2010 运行库
```

也可以在 `config.json` 的 `dll_dir` 字段指向任意包含 `DRFLWin64.dll` 的目录；都找不到时程序会弹出文件夹选择对话框。

## 快速开始

```bat
:: 方式一：双击启动（首次自动创建 .venv 并安装依赖）
启动监测台.bat

:: 方式二：手动
python -m venv .venv
.venv\Scripts\pip install -r requirements.txt
.venv\Scripts\python app.py
```

默认连接 `192.168.137.101`（R1）与 `192.168.137.102`（R2），端口 12345（DRFL 默认）。可在界面或 `config.json` 中修改：

```json
{
  "robot1_ip": "192.168.137.101",
  "robot2_ip": "192.168.137.102",
  "auto_connect": true,
  "window_sec": 10,
  "group": "外部六维力/力矩",
  "dll_dir": ""
}
```

## 项目结构

```
├── app.py            # 主程序：UI、曲线、数值面板、CSV 录制
├── drfl_client.py    # DRFL ctypes 绑定 + 只读监控客户端（517 字节监控包解析）
├── config.json       # 运行配置
├── requirements.txt
├── 启动监测台.bat     # Windows 一键启动（自动建 venv）
└── drfl/             # DRFL 运行库（自行获取，不随仓库分发）
```

## 安全说明

- 客户端只调用 `_open_connection`、`_get_*` 状态查询与 `_set_on_monitoring_data` 监控回调，**不发送任何控制指令**，可安全地在机器人运行其他任务时并行监测。
- 监测数据流来自控制器打包的 517 字节记录（DRFL 手册 2.3.2），采样率由控制器决定（界面实时显示）。

## 许可

本项目源码可自由使用参考。DRFL 及 Poco/VC++ 运行库版权归斗山机器人及各自权利人所有，请勿通过本仓库再分发。
# Learning Paths

> Choose the path that matches what you want to do.

---

## Path 1: Understand the System (30 min)

| Step | Read | What you'll learn |
|------|------|-------------------|
| 1 | [README.md](../README.md) | What this project does, architecture overview |
| 2 | [ARCHITECTURE.md](ARCHITECTURE.md) § 1-5 | 3-process model, SHM protocol, data flow |
| 3 | [ARCHITECTURE.md](ARCHITECTURE.md) § 6-7 | Why no Nav2, navigation pipeline, state machine |

## Path 2: Deploy to V2N (30 min)

| Step | Read | What you'll learn |
|------|------|-------------------|
| 1 | [HARDWARE_SETUP.md](HARDWARE_SETUP.md) § 1-3 | Physical robot, wiring, connections |
| 2 | [V2N_SETUP_GUIDE.md](V2N_SETUP_GUIDE.md) | Full deployment from scratch |

## Path 3: Develop and Test on PC (15 min)

| Step | Read | What you'll learn |
|------|------|-------------------|
| 1 | [DEVELOPER_GUIDE.md](DEVELOPER_GUIDE.md) | Clone, setup, test, deploy workflow |

## Path 4: Tune Navigation (1 hour)

| Step | Read | What you'll learn |
|------|------|-------------------|
| 1 | [NAVIGATION.md](NAVIGATION.md) § 1-3 | Pipeline, state machine, all 6 states |
| 2 | [NAVIGATION.md](NAVIGATION.md) § 4-5 | VFH obstacle avoidance, speed ramp |
| 3 | [NAVIGATION.md](NAVIGATION.md) § 6-7 | Detection filtering, Kalman tracker |
| 4 | [NAVIGATION.md](NAVIGATION.md) § 8-9 | Arrival detection, sensor offsets |
| 5 | [GUI_GUIDE.md](GUI_GUIDE.md) § 8 | Settings GUI for live tuning |

## Path 5: Modify Detection (30 min)

| Step | Read | What you'll learn |
|------|------|-------------------|
| 1 | [ARCHITECTURE.md](ARCHITECTURE.md) § 3-4 | Camera process, SHM channels |
| 2 | `target_nav/app/camera_worker.py` docstring | Stream vs pipe mode, detection pipeline |
| 3 | `target_nav/app/detection_filters.py` docstring | Temporal tracking |
| 4 | [drpai/README.md](../drpai/README.md) | C++ binary, config.ini, build |

## Path 6: Understand the GUI (20 min)

| Step | Read | What you'll learn |
|------|------|-------------------|
| 1 | [GUI_GUIDE.md](GUI_GUIDE.md) § 1-7 | Layout, panels, controls, shortcuts |
| 2 | [GUI_GUIDE.md](GUI_GUIDE.md) § 8 | Settings window with all parameters |
| 3 | [GUI_GUIDE.md](GUI_GUIDE.md) § 9-10 | Display setup and theme |

## Quick Reference

| I want to... | Go to... |
|--------------|----------|
| Run the robot | Press Start on touchscreen, or SSH → `/root/gui.sh` |
| Push code to V2N | `./scripts/sync.sh` |
| Run unit tests | `pytest test/unit/` |
| Run hardware tests | `./scripts/run_tests.sh --check` |
| Change navigation speed | Settings → Navigate → Speed tab |
| Change detection confidence | Settings → Sensors → Detection tab |
| Tune DRP-AI inference | Settings → Sensors → DRP-AI tab → Apply & Restart |
| Calibrate distance | Settings → Sensors → Calibration → place target → CALIBRATE |
| Check robot status | `ssh root@192.168.50.1` → `systemctl status robot` |
| View live topics | `ros2 topic list` (connect to RZV2N_Robot WiFi first) |

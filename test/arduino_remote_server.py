#!/usr/bin/env python3
"""
Arduino Remote Server
=====================

Run this on the V2N device to allow remote control of Arduino over network.

Protocol (matches rzv2n-arduino-motor-controller firmware):

  Movement:   FWD,speed,ticks   (speed>0, ticks>0 required)
              BWD,speed,ticks
              LEFT,speed,ticks
              RIGHT,speed,ticks
              TURN,speed,ticks  (positive ticks=CCW, negative=CW)
              DIAGFL,speed,ticks / DIAGFR / DIAGBL / DIAGBR

  Control:    STOP              (emergency stop)
              READ              (read encoders - returns 4 lines)
              SYNC              (run calibration)

  Responses:  READY / DONE / BUSY / ERROR: msg

Usage:
    python3 arduino_remote_server.py
    python3 arduino_remote_server.py --port 5555 --arduino /dev/ttyACM0
"""

import socket
import threading
import json
import argparse
import time
import serial
import glob as globmod


class ArduinoBridge:
    """Serial communication matching rzv2n-arduino-motor-controller firmware.

    Uses non-blocking motor commands: send MOVE and return immediately.
    A background thread reads all serial output from the Arduino.
    """

    BAUD_RATE = 115200
    TIMEOUT = 0.1  # Short timeout for non-blocking reads
    LARGE_TICKS = 99999  # Large tick count - movement runs until STOP

    def __init__(self, port: str = "/dev/ttyACM0"):
        self.port = port
        self.serial = None
        self.connected = False
        self._lock = threading.Lock()
        self._reader_thread = None
        self._running = False
        self._last_lines = []  # Recent serial output for queries
        # Encoder baseline tracking (auto-reset before each move)
        self._enc_raw = {"FL": 0, "FR": 0, "RL": 0, "RR": 0}
        self._enc_baseline = {"FL": 0, "FR": 0, "RL": 0, "RR": 0}

    def connect(self) -> tuple:
        try:
            print(f"[Arduino] Connecting to {self.port}...")
            self.serial = serial.Serial(
                port=self.port,
                baudrate=self.BAUD_RATE,
                timeout=self.TIMEOUT
            )

            print("[Arduino] Waiting for Arduino reset (2s)...")
            time.sleep(2.0)
            self.serial.reset_input_buffer()

            # Read startup messages
            print("[Arduino] Reading startup...")
            start = time.time()
            while time.time() - start < 3.0:
                if self.serial.in_waiting:
                    line = self.serial.readline().decode('utf-8', errors='ignore').strip()
                    if line:
                        print(f"[Arduino] << {line}")
                    if 'READY' in line.upper():
                        break
                time.sleep(0.1)

            self.connected = True
            print("[Arduino] Connected!")

            # Start background reader
            self._running = True
            self._reader_thread = threading.Thread(target=self._reader_loop, daemon=True)
            self._reader_thread.start()

            return True, "Connected"
        except Exception as e:
            print(f"[Arduino] Failed: {e}")
            return False, str(e)

    def disconnect(self):
        self._running = False
        if self._reader_thread:
            self._reader_thread.join(timeout=1.0)
        self._send("STOP")
        time.sleep(0.1)
        if self.serial:
            self.serial.close()
        self.serial = None
        self.connected = False

    def _reader_loop(self):
        """Background thread: continuously reads serial output from Arduino."""
        while self._running and self.connected:
            try:
                if self.serial and self.serial.in_waiting:
                    with self._lock:
                        line = self.serial.readline().decode('utf-8', errors='ignore').strip()
                    if line:
                        if line.startswith("ENC,"):
                            self._parse_enc(line)
                        elif line == "ENC_RESET":
                            # Firmware reset encoders - zero our tracking
                            for key in ["FL", "FR", "RL", "RR"]:
                                self._enc_raw[key] = 0
                        print(f"[Arduino] << {line}")
                        self._last_lines.append(line)
                        if len(self._last_lines) > 50:
                            self._last_lines = self._last_lines[-50:]
                else:
                    time.sleep(0.01)
            except Exception:
                time.sleep(0.05)

    def get_encoders(self) -> dict:
        """Return latest encoder values (updated by firmware stream)."""
        return dict(self._enc_raw)

    def _parse_enc(self, line: str):
        """Parse ENC,FL:1234,FR:5678,RL:9012,RR:3456,t_us:... into _enc_raw."""
        for part in line.split(","):
            for key in ["FL", "FR", "RL", "RR"]:
                if part.startswith(f"{key}:"):
                    try:
                        self._enc_raw[key] = int(part.split(":")[1])
                    except ValueError:
                        pass

    def reset_encoder_baseline(self):
        """Reset baseline to 0 - firmware does ENC_RESET on each MOVE."""
        for key in ["FL", "FR", "RL", "RR"]:
            self._enc_baseline[key] = 0
            self._enc_raw[key] = 0
        print(f"[Arduino] Encoder baseline reset to zero")

    def _send(self, cmd: str) -> str:
        """Send command to Arduino (fire-and-forget, non-blocking)."""
        if not self.connected or not self.serial:
            return "ERROR: Not connected"

        with self._lock:
            try:
                print(f"[Arduino] >> {cmd}")
                self.serial.write(f"{cmd}\n".encode('utf-8'))
                self.serial.flush()
                return "OK"
            except Exception as e:
                return f"ERROR: {e}"

    def _send_and_wait(self, cmd: str, wait_for: str = None, timeout: float = 2.0) -> str:
        """Send command and wait for a specific response keyword."""
        self._last_lines.clear()
        self._send(cmd)

        if not wait_for:
            time.sleep(0.3)
            return '\n'.join(self._last_lines[-10:]) if self._last_lines else "OK"

        start = time.time()
        while time.time() - start < timeout:
            for line in self._last_lines:
                if wait_for in line:
                    return '\n'.join(self._last_lines[-10:])
            time.sleep(0.05)

        return '\n'.join(self._last_lines[-10:]) if self._last_lines else "TIMEOUT"

    def stop(self) -> str:
        """Stop all motors immediately."""
        return self._send("STOP")

    def move(self, direction: str, speed: int, ticks: int) -> str:
        """Send timed movement command. Speed>0 and ticks>0 required by firmware."""
        speed = max(20, min(255, speed))
        ticks = max(1, ticks)
        return self._send(f"{direction},{speed},{ticks}")

    def turn(self, speed: int, ticks: int, clockwise: bool = False) -> str:
        """Send turn command. Positive ticks=CCW, negative=CW."""
        speed = max(20, min(255, speed))
        ticks = max(1, abs(ticks))
        if clockwise:
            ticks = -ticks
        return self._send(f"TURN,{speed},{ticks}")

    def read_encoders(self) -> str:
        return self._send_and_wait("READ", wait_for="ENC")

    def calibrate(self) -> str:
        return self._send_and_wait("SYNC", wait_for="DONE", timeout=10.0)

    def motor_command(self, direction: str, speed: int = 100, ticks: int = 0) -> str:
        """Handle GUI button press: start movement.

        ticks=0 (default) -> use LARGE_TICKS (runs until stopped)
        ticks>0           -> use specified ticks (finite move)
        Button release    -> STOP
        """
        if direction == 'STOP':
            return self.stop()

        s = max(20, min(255, speed))
        t = ticks if ticks > 0 else self.LARGE_TICKS

        # Stop any ongoing movement first (firmware returns BUSY if MOVING)
        self._send("STOP")
        time.sleep(0.02)  # Brief pause for state transition

        # Reset encoder baseline so this move starts from zero
        self.reset_encoder_baseline()

        # Map GUI direction to firmware MOVE commands
        cmd_map = {
            'FWD':    f"FWD,{s},{t}",
            'BWD':    f"BWD,{s},{t}",
            'LEFT':   f"LEFT,{s},{t}",
            'RIGHT':  f"RIGHT,{s},{t}",
            'TURNL':  f"TURN,{s},{t}",        # positive ticks = CCW
            'TURNR':  f"TURN,{s},{-t}",       # negative ticks = CW
            'DIAGFL': f"DIAGFL,{s},{t}",
            'DIAGFR': f"DIAGFR,{s},{t}",
            'DIAGBL': f"DIAGBL,{s},{t}",
            'DIAGBR': f"DIAGBR,{s},{t}",
        }

        if direction not in cmd_map:
            return f"ERROR: Unknown direction {direction}"

        return self._send(cmd_map[direction])


class ArduinoRemoteServer:
    """TCP server for remote Arduino control."""

    def __init__(self, host: str = "0.0.0.0", port: int = 5555):
        self.host = host
        self.port = port
        self.arduino = None
        self.server_socket = None
        self.running = False

    def start(self, arduino_port: str = "/dev/ttyACM0"):
        self.arduino = ArduinoBridge(arduino_port)
        success, msg = self.arduino.connect()
        if not success:
            print(f"[Server] Arduino failed: {msg}")
            return False

        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.server_socket.bind((self.host, self.port))
        self.server_socket.listen(1)

        self.running = True
        print(f"[Server] Running on {self.host}:{self.port}")
        print("[Server] Waiting for connections...")

        while self.running:
            try:
                self.server_socket.settimeout(1.0)
                try:
                    client, addr = self.server_socket.accept()
                    print(f"[Server] Client: {addr}")
                    self._handle_client(client)
                except socket.timeout:
                    continue
            except Exception as e:
                if self.running:
                    print(f"[Server] Error: {e}")

        return True

    def _handle_client(self, client: socket.socket):
        client.settimeout(1.0)

        while self.running:
            try:
                data = client.recv(1024)
                if not data:
                    break

                try:
                    cmd = json.loads(data.decode('utf-8'))
                    response = self._process(cmd)
                    client.send(json.dumps(response).encode('utf-8'))
                except json.JSONDecodeError:
                    client.send(json.dumps({"error": "Invalid JSON"}).encode('utf-8'))

            except socket.timeout:
                continue
            except Exception as e:
                print(f"[Server] Client error: {e}")
                break

        # Stop motors when client disconnects
        self.arduino.stop()
        client.close()
        print("[Server] Client disconnected")

    def _process(self, cmd: dict) -> dict:
        action = cmd.get("action", "")

        if action == "ping":
            return {"status": "ok", "connected": self.arduino.connected}

        elif action == "motor":
            direction = cmd.get("cmd", "STOP")
            speed = cmd.get("speed", 100)
            ticks = cmd.get("ticks", 0)
            response = self.arduino.motor_command(direction, speed, ticks)
            return {"status": "ok", "response": response}

        elif action == "move":
            # Timed movement: {"action": "move", "dir": "FWD", "speed": 100, "ticks": 1000}
            direction = cmd.get("dir", "FWD")
            speed = cmd.get("speed", 100)
            ticks = cmd.get("ticks", 1000)
            response = self.arduino.move(direction, speed, ticks)
            return {"status": "ok", "response": response}

        elif action == "command":
            cmd_name = cmd.get("cmd", "")
            if cmd_name == "SYNC":
                response = self.arduino.calibrate()
            elif cmd_name == "READ":
                response = self.arduino.read_encoders()
            elif cmd_name == "RESET":
                response = self.arduino.stop()
            elif cmd_name == "STOP":
                response = self.arduino.stop()
            else:
                response = self.arduino._send(cmd_name)
            return {"status": "ok", "response": response}

        elif action == "get_encoders":
            enc = self.arduino.get_encoders()
            return {"status": "ok", "encoders": enc}

        elif action == "disconnect":
            self.arduino.stop()
            return {"status": "ok"}

        else:
            return {"error": f"Unknown action: {action}"}

    def stop(self):
        self.running = False
        if self.arduino:
            self.arduino.disconnect()
        if self.server_socket:
            self.server_socket.close()
        print("[Server] Stopped")


def find_arduino_port():
    ports = globmod.glob('/dev/ttyACM*') + globmod.glob('/dev/ttyUSB*')
    return ports[0] if ports else '/dev/ttyACM0'


def main():
    parser = argparse.ArgumentParser(description="Arduino Remote Server")
    parser.add_argument("--host", default="0.0.0.0", help="Bind address")
    parser.add_argument("--port", type=int, default=5555, help="Server port")
    parser.add_argument("--arduino", default=None, help="Arduino serial port")
    args = parser.parse_args()

    arduino_port = args.arduino or find_arduino_port()
    print(f"[Server] Arduino port: {arduino_port}")

    server = ArduinoRemoteServer(args.host, args.port)

    try:
        server.start(arduino_port)
    except KeyboardInterrupt:
        print("\n[Server] Shutting down...")
    finally:
        server.stop()


if __name__ == "__main__":
    main()

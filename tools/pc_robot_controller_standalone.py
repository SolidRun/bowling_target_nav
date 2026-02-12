#!/usr/bin/env python3
"""Standalone Robot Controller - No ROS2 Required"""
import socket
import time
import threading
import json
import tkinter as tk
from tkinter import ttk, scrolledtext, messagebox

class RobotController:
    def __init__(self, host="192.168.50.1", port=9090):
        self.host = host
        self.port = port
        self.socket = None
        self.connected = False
        self.on_data_callback = None
        self.running = False

    def connect(self):
        try:
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.socket.settimeout(5.0)
            self.socket.connect((self.host, self.port))
            self.socket.settimeout(0.1)
            self.connected = True
            self.running = True
            threading.Thread(target=self._read_loop, daemon=True).start()
            return True, f"Connected to {self.host}:{self.port}"
        except Exception as e:
            return False, str(e)

    def disconnect(self):
        self.running = False
        self.connected = False
        if self.socket:
            try: self.socket.close()
            except: pass
            self.socket = None

    def _read_loop(self):
        buffer = ""
        while self.running and self.socket:
            try:
                data = self.socket.recv(1024).decode('utf-8', errors='ignore')
                if data:
                    buffer += data
                    while '\n' in buffer:
                        line, buffer = buffer.split('\n', 1)
                        if line.strip() and self.on_data_callback:
                            self.on_data_callback(line.strip())
            except socket.timeout: pass
            except: break

    def send_velocity(self, lx, ly, az):
        if self.connected and self.socket:
            try:
                msg = json.dumps({"op": "publish", "topic": "/cmd_vel",
                    "msg": {"linear": {"x": lx, "y": ly, "z": 0}, "angular": {"x": 0, "y": 0, "z": az}}})
                self.socket.send(f"{msg}\n".encode())
                return True
            except: pass
        return False

class RobotGUI:
    def __init__(self):
        self.robot = RobotController()
        self.robot.on_data_callback = self.on_response
        self.root = tk.Tk()
        self.root.title("V2N Robot Controller (Standalone)")
        self.root.geometry("500x400")
        self._create_widgets()
        self._bind_keys()

    def _create_widgets(self):
        main = ttk.Frame(self.root, padding=10)
        main.pack(fill='both', expand=True)

        conn = ttk.LabelFrame(main, text="Connection", padding=10)
        conn.pack(fill='x', pady=5)
        ttk.Label(conn, text="V2N: 192.168.50.1:9090").pack(side='left')
        self.connect_btn = ttk.Button(conn, text="Connect", command=self.toggle_connection)
        self.connect_btn.pack(side='left', padx=20)
        self.status = ttk.Label(conn, text="Disconnected", foreground='red')
        self.status.pack(side='left')

        speed = ttk.LabelFrame(main, text="Speed", padding=5)
        speed.pack(fill='x', pady=5)
        ttk.Label(speed, text="Linear:").pack(side='left')
        self.linear_var = tk.DoubleVar(value=0.3)
        ttk.Scale(speed, from_=0.1, to=1.0, variable=self.linear_var, length=80).pack(side='left')
        ttk.Label(speed, text="Angular:").pack(side='left', padx=(10,0))
        self.angular_var = tk.DoubleVar(value=0.5)
        ttk.Scale(speed, from_=0.1, to=2.0, variable=self.angular_var, length=80).pack(side='left')

        ctrl = ttk.LabelFrame(main, text="Controls (WASD/QE)", padding=10)
        ctrl.pack(fill='x', pady=5)
        bf = ttk.Frame(ctrl)
        bf.pack()
        ttk.Button(bf, text="↑", command=self.fwd, width=5).grid(row=0, column=1)
        ttk.Button(bf, text="←", command=self.left, width=5).grid(row=1, column=0)
        ttk.Button(bf, text="■", command=self.stop, width=5).grid(row=1, column=1)
        ttk.Button(bf, text="→", command=self.right, width=5).grid(row=1, column=2)
        ttk.Button(bf, text="↓", command=self.bwd, width=5).grid(row=2, column=1)
        ttk.Button(bf, text="↺", command=self.rot_l, width=5).grid(row=1, column=3, padx=10)
        ttk.Button(bf, text="↻", command=self.rot_r, width=5).grid(row=1, column=4)

        self.log = scrolledtext.ScrolledText(main, height=6, state='disabled')
        self.log.pack(fill='both', expand=True, pady=5)

    def _bind_keys(self):
        for k in ['w','W']: self.root.bind(f'<{k}>', lambda e: self.fwd()); self.root.bind(f'<KeyRelease-{k}>', lambda e: self.stop())
        for k in ['s','S']: self.root.bind(f'<{k}>', lambda e: self.bwd()); self.root.bind(f'<KeyRelease-{k}>', lambda e: self.stop())
        for k in ['a','A']: self.root.bind(f'<{k}>', lambda e: self.left()); self.root.bind(f'<KeyRelease-{k}>', lambda e: self.stop())
        for k in ['d','D']: self.root.bind(f'<{k}>', lambda e: self.right()); self.root.bind(f'<KeyRelease-{k}>', lambda e: self.stop())
        self.root.bind('<q>', lambda e: self.rot_l()); self.root.bind('<e>', lambda e: self.rot_r())
        self.root.bind('<space>', lambda e: self.stop())

    def toggle_connection(self):
        if self.robot.connected:
            self.robot.disconnect()
            self.connect_btn.config(text="Connect")
            self.status.config(text="Disconnected", foreground='red')
        else:
            ok, msg = self.robot.connect()
            if ok:
                self.connect_btn.config(text="Disconnect")
                self.status.config(text="Connected", foreground='green')
                self._log(msg)
            else:
                messagebox.showerror("Error", f"Failed: {msg}\n\nRun on V2N:\nros2 launch rosbridge_server rosbridge_websocket_launch.xml")

    def on_response(self, data): self.root.after(0, lambda: self._log(f"← {data}"))
    def _log(self, msg):
        self.log.config(state='normal')
        self.log.insert('end', f"[{time.strftime('%H:%M:%S')}] {msg}\n")
        self.log.see('end')
        self.log.config(state='disabled')

    def fwd(self): self.robot.send_velocity(self.linear_var.get(), 0, 0)
    def bwd(self): self.robot.send_velocity(-self.linear_var.get(), 0, 0)
    def left(self): self.robot.send_velocity(0, self.linear_var.get(), 0)
    def right(self): self.robot.send_velocity(0, -self.linear_var.get(), 0)
    def rot_l(self): self.robot.send_velocity(0, 0, self.angular_var.get())
    def rot_r(self): self.robot.send_velocity(0, 0, -self.angular_var.get())
    def stop(self): self.robot.send_velocity(0, 0, 0)

    def run(self):
        self.root.protocol("WM_DELETE_WINDOW", lambda: (self.robot.disconnect(), self.root.destroy()))
        self.root.mainloop()

if __name__ == "__main__":
    RobotGUI().run()

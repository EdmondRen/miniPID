"""
PI + feedforward temperature control GUI (Tkinter)

Assumptions:
- You have an object named `controller` providing:
    controller.get_temperature() -> float   # temperature in your chosen units (e.g., K)
    controller.set_heater(power_uW: float)  # heater power in µW (or whatever you decide)

What you get:
- Start/Stop loop (updates each sample)
- Feedforward term (u_ff)
- PI control: u = u_ff + Kp*e + I
- Integral handling:
    - Reset integral on Start (optional)
    - Reset or keep integral when you change Kp/Ki while running
- Presets saved to a JSON file (save/load/delete)

Notes:
- Units matter. Kp should be (µW / temperature_unit).
- Ki is per second: (µW / temperature_unit / s).
- Sample time defaults to 30 s, but is editable.

Run:
    python temperature_controller.py
"""

import json
import os
import threading
import time
import traceback
import tkinter as tk
import requests
from collections import deque
from statistics import stdev
from tkinter import ttk, messagebox, simpledialog

PRESET_FILE = "presets.json"


# ----------------------------------------------------------------------
# Replace this with your real controller import / initialization
# ----------------------------------------------------------------------
class DummyController:
    """A simple simulated thermal plant for testing GUI without hardware."""
    def __init__(self):
        self._T = 0.050  # 50 mK (example)
        self._u = 0.0
        self._ambient = 0.040
        self._K = 0.002     # K per µW steady-state gain
        self._tau = 180.0   # seconds
        self._last = time.time()

    def get_temperature(self):
        # crude first-order response: dT/dt = -(T - (ambient + K*u))/tau
        now = time.time()
        dt = max(0.0, now - self._last)
        self._last = now
        target = self._ambient + self._K * self._u
        self._T += (-(self._T - target) / self._tau) * dt
        return self._T

    def set_heater(self, power_uW: float):
        self._u = float(power_uW)


class BlueforsController:
    def __init__(self, DEVICE_IP='localhost', TIMEOUT=10, CHANNEL_NUMBER_THERM = 0, CHANNEL_NUMBER_HEAT = 0):
        self.DEVICE_IP = DEVICE_IP
        self.TIMEOUT = TIMEOUT
        self.CHANNEL_NUMBER_THERM = CHANNEL_NUMBER_THERM
        self.CHANNEL_NUMBER_HEAT = CHANNEL_NUMBER_HEAT
        self._measurement_lock = threading.Lock()
        self._measurement_event = threading.Event()
        self._stop_event = threading.Event()
        self._poll_thread = None
        self._latest_by_channel = {}
        self._last_poll_error = None

    def connect(self):
        url = 'http://{}:5001/system'.format(self.DEVICE_IP)
        req = requests.get(url, timeout=self.TIMEOUT)
        req.raise_for_status()
        self._start_polling()
        return req.json()

    def close(self):
        self._stop_event.set()
        if self._poll_thread is not None and self._poll_thread.is_alive():
            self._poll_thread.join(timeout=1.0)
        self._poll_thread = None

    def _start_polling(self):
        if self._poll_thread is not None and self._poll_thread.is_alive():
            return
        self._stop_event.clear()
        self._poll_thread = threading.Thread(
            target=self._poll_measurements_loop,
            name="bluefors-poll",
            daemon=True,
        )
        self._poll_thread.start()

    def _poll_measurements_loop(self):
        url = 'http://{}:5001/channel/measurement/latest'.format(self.DEVICE_IP)
        poll_interval_s = 0.2

        while not self._stop_event.is_set():
            try:
                req = requests.get(url, timeout=self.TIMEOUT)
                req.raise_for_status()
                data = req.json()
                channel_number = int(data["channel_nr"])
                temperature = data["temperature"]  # Assuming the API returns temperature in the "temperature" field
                with self._measurement_lock:
                    self._latest_by_channel[channel_number] = {
                        "temperature": temperature,
                        "timestamp": time.time(),
                    }
                    self._last_poll_error = None
                self._measurement_event.set()
            except Exception as exc:
                with self._measurement_lock:
                    self._last_poll_error = exc
            self._stop_event.wait(poll_interval_s)

    def get_setting(self, channel_number: int = None):
        if channel_number is None:
            channel_number = self.CHANNEL_NUMBER_THERM
        url = 'http://{}:5001/channel'.format(self.DEVICE_IP)
        data = {
            'channel_nr': channel_number
        }
        req = requests.post(url, json=data, timeout=self.TIMEOUT)
        req.raise_for_status()
        return req.json()
    
    def get_temperature(self, channel_number: int = None):
        if channel_number is None:
            channel_number = self.CHANNEL_NUMBER_THERM
        with self._measurement_lock:
            measurement = self._latest_by_channel.get(channel_number)
            last_error = self._last_poll_error

        if measurement is not None:
            return measurement["temperature"]

        if self._poll_thread is None or not self._poll_thread.is_alive():
            raise RuntimeError("Bluefors polling thread is not running.")

        if last_error is not None:
            raise RuntimeError(f"Polling failed: {last_error}")

        raise LookupError(f"Waiting for measurement from channel {channel_number}.")

    def set_heater(self, power_uW: float, channel_number: int = None):
        if channel_number is None:
            channel_number = self.CHANNEL_NUMBER_HEAT
        url = 'http://{}:5001/heater/update'.format(self.DEVICE_IP)
        data = {
            'heater_nr': channel_number,
            'pid_mode' : 0, # 0 for manual power control, 1 for PID control
            'power': power_uW * 1e-6, # Applied manual power in Watts. The maximum power is 100 mA
            # 'power': '0.000001', # Applied manual power in Watts. The maximum power is 100 mA
        }
        # print("Sending command:", data)
        req = requests.post(url, json=data, timeout=self.TIMEOUT)
        req.raise_for_status()
        rtn = req.json()
        # print("Response:", rtn)

        return 

# ----------------------------------------------------------------------
# Core PI+FF controller logic (discrete time)
# ----------------------------------------------------------------------
class PIWithFeedforward:
    def __init__(self):
        self.integral_uW = 0.0  # integral contribution already in µW
        self.measurement_history = deque()

    def reset_integral(self):
        self.integral_uW = 0.0

    def reset_state(self):
        self.integral_uW = 0.0
        self.measurement_history.clear()

    def update(self, setpoint, measurement, kp, ki_per_s, kd, deriv_points, dt_s, u_ff,
               u_min, u_max, antiwindup=True):
        """
        Returns (u_cmd, error, p_term, i_term, d_term)

        Implementation:
          e = setpoint - measurement
          integral_uW += (ki * e * dt)
          d_term = -kd * d(measurement)/dt
          u = u_ff + kp*e + integral_uW + d_term

        Anti-windup:
          If saturated and integration would drive further into saturation,
          stop integrating (simple clamping).
        """
        e = setpoint - measurement
        p = kp * e
        window = max(2, int(deriv_points))
        self.measurement_history.append(float(measurement))
        while len(self.measurement_history) > window:
            self.measurement_history.popleft()

        if len(self.measurement_history) >= 2 and dt_s > 0:
            d_meas_dt = (
                self.measurement_history[-1] - self.measurement_history[0]
            ) / (dt_s * (len(self.measurement_history) - 1))
        else:
            d_meas_dt = 0.0
        d = -kd * d_meas_dt

        # Propose updated integral
        i_proposed = self.integral_uW + (ki_per_s * e * dt_s)
        u_unsat = u_ff + p + i_proposed + d

        # Saturate
        u = min(max(u_unsat, u_min), u_max)

        if antiwindup:
            # If not saturated, accept.
            if u == u_unsat:
                self.integral_uW = i_proposed
            else:
                # If saturated, only integrate if it helps move out of saturation
                if u == u_max:
                    # If error is negative, integrating would reduce u -> allow
                    if e < 0:
                        self.integral_uW = i_proposed
                    # else freeze integral
                elif u == u_min:
                    # If error is positive, integrating would increase u -> allow
                    if e > 0:
                        self.integral_uW = i_proposed
                # otherwise freeze
        else:
            self.integral_uW = i_proposed

        # Recompute terms with stored integral (in case we froze it)
        i = self.integral_uW
        u_cmd = min(max(u_ff + p + i + d, u_min), u_max)
        return u_cmd, e, p, i, d


# ----------------------------------------------------------------------
# Preset storage
# ----------------------------------------------------------------------
def load_presets():
    if not os.path.exists(PRESET_FILE):
        return {}
    try:
        with open(PRESET_FILE, "r", encoding="utf-8") as f:
            data = json.load(f)
        if not isinstance(data, dict):
            return {}
        return data
    except Exception:
        return {}


def save_presets(presets: dict):
    with open(PRESET_FILE, "w", encoding="utf-8") as f:
        json.dump(presets, f, indent=2, sort_keys=True)


# ----------------------------------------------------------------------
# GUI
# ----------------------------------------------------------------------
class App(tk.Tk):
    def __init__(self, controller_factory):
        super().__init__()
        self.title("PI+Feedforward Temp Ctrl")
        self.controller_factory = controller_factory
        self.controller = None

        # Control
        self.pi = PIWithFeedforward()
        self.running = False
        self.last_update_time = None

        # Presets
        self.presets = load_presets()
        self.temp_history = deque(maxlen=100)

        # Tk variables
        self.var_setpoint = tk.DoubleVar(value=0.050)     # e.g. K
        self.var_kp = tk.DoubleVar(value=0.01)            # µW / K
        self.var_ki = tk.DoubleVar(value=0.00001)         # µW / K / s
        self.var_kd = tk.DoubleVar(value=0.0)             # µW * s / K
        self.var_d_smooth_points = tk.IntVar(value=5)
        self.var_uff = tk.DoubleVar(value=3.0)            # µW
        self.var_dt = tk.DoubleVar(value=30.0)            # s
        self.var_u_min = tk.DoubleVar(value=0.0)          # µW
        self.var_u_max = tk.DoubleVar(value=20.0)         # µW

        self.var_reset_integral_on_start = tk.BooleanVar(value=True)
        self.var_keep_integral_on_tune = tk.BooleanVar(value=True)
        self.var_antiwindup = tk.BooleanVar(value=True)
        self.var_controller_ip = tk.StringVar(value="localhost")
        self.var_controller_timeout = tk.DoubleVar(value=10.0)
        self.var_temp_channel = tk.IntVar(value=1)
        self.var_heater_channel = tk.IntVar(value=1)
        self.var_connection_status = tk.StringVar(value="Disconnected")

        # Live readouts
        self.var_temp = tk.StringVar(value="—")
        self.var_temp_std = tk.StringVar(value="—")
        self.var_error = tk.StringVar(value="—")
        self.var_u_cmd = tk.StringVar(value="—")
        self.var_p_term = tk.StringVar(value="—")
        self.var_i_term = tk.StringVar(value="—")
        self.var_d_term = tk.StringVar(value="—")
        self.var_status = tk.StringVar(value="Stopped")

        self._build_ui()
        self._refresh_preset_list()

        # Trace changes to Kp/Ki to optionally reset integral while running
        self.var_kp.trace_add("write", self._on_tune_param_changed)
        self.var_ki.trace_add("write", self._on_tune_param_changed)
        self.var_kd.trace_add("write", self._on_tune_param_changed)
        self.var_uff.trace_add("write", self._on_tune_param_changed)
        self.var_d_smooth_points.trace_add("write", self._on_tune_param_changed)

        # Poll temperature display (even when stopped) at a gentle rate
        self.after(500, self._idle_poll_temperature)

    def _build_ui(self):
        pad = {"padx": 8, "pady": 6}

        controller_frame = ttk.LabelFrame(self, text="Controller")
        controller_frame.grid(row=0, column=0, sticky="ew", **pad)
        controller_frame.columnconfigure(1, weight=1)

        ttk.Label(controller_frame, text="IP address").grid(row=0, column=0, sticky="w")
        ttk.Entry(controller_frame, textvariable=self.var_controller_ip, width=18).grid(row=0, column=1, sticky="ew")
        ttk.Label(controller_frame, text="Timeout (s)").grid(row=1, column=0, sticky="w", pady=(6, 0))
        ttk.Entry(controller_frame, textvariable=self.var_controller_timeout, width=8).grid(row=1, column=1, sticky="w", pady=(6, 0))

        ttk.Label(controller_frame, text="Thermometer channel").grid(row=2, column=0, sticky="w", pady=(6, 0))
        ttk.Entry(controller_frame, textvariable=self.var_temp_channel, width=8).grid(row=2, column=1, sticky="w", pady=(6, 0))
        ttk.Label(controller_frame, text="Heater channel").grid(row=3, column=0, sticky="w", pady=(6, 0))
        ttk.Entry(controller_frame, textvariable=self.var_heater_channel, width=8).grid(row=3, column=1, sticky="w", pady=(6, 0))

        self.btn_connect = ttk.Button(controller_frame, text="Connect", command=self.connect_controller)
        self.btn_connect.grid(row=0, column=4, rowspan=2, sticky="w", padx=(12, 0))
        ttk.Button(controller_frame, text="Help", command=self._show_help).grid(row=3, column=4, rowspan=2, sticky="w", padx=(12, 0))

        self.connection_indicator = tk.Label(controller_frame, width=2, bg="#b91c1c", relief="groove")
        self.connection_indicator.grid(row=0, column=5, rowspan=2, sticky="w", padx=(12, 4))
        ttk.Label(controller_frame, textvariable=self.var_connection_status).grid(row=1, column=4, rowspan=4, sticky="w", padx=(16, 0))

        # Top: parameters
        frm = ttk.Frame(self)
        frm.grid(row=1, column=0, sticky="nsew", **pad)
        self.columnconfigure(0, weight=1)

        def add_row(r, label, var, unit="", width=12):
            ttk.Label(frm, text=label).grid(row=r, column=0, sticky="w")
            ent = ttk.Entry(frm, textvariable=var, width=width)
            ent.grid(row=r, column=1, sticky="w")
            ttk.Label(frm, text=unit).grid(row=r, column=2, sticky="w")
            return ent

        add_row(0, "Setpoint", self.var_setpoint, "K")
        add_row(1, "Kp", self.var_kp, "µW/K")
        add_row(2, "Ki", self.var_ki, "µW/K/s")
        add_row(3, "Kd", self.var_kd, "µW*s/K")
        add_row(4, "Derivative smoothing", self.var_d_smooth_points, "pts")
        add_row(5, "Feedforward u_ff", self.var_uff, "µW")
        add_row(6, "Sample time dt", self.var_dt, "s")
        add_row(7, "Heater min", self.var_u_min, "µW")
        add_row(8, "Heater max", self.var_u_max, "µW")

        opts = ttk.Frame(frm)
        opts.grid(row=9, column=0, columnspan=3, sticky="w", pady=(6, 0))
        ttk.Checkbutton(opts, text="Reset integral on Start",
                        variable=self.var_reset_integral_on_start).grid(row=0, column=0, sticky="w")
        ttk.Checkbutton(opts, text="Keep integral when tuning Kp/Ki/u_ff",
                        variable=self.var_keep_integral_on_tune).grid(row=1, column=0, sticky="w")
        ttk.Checkbutton(opts, text="Anti-windup (integrator clamping)",
                        variable=self.var_antiwindup).grid(row=2, column=0, sticky="w")

        # Controls
        ctrl = ttk.Frame(frm)
        ctrl.grid(row=10, column=0, columnspan=3, sticky="w", pady=(10, 0))
        self.btn_start = ttk.Button(ctrl, text="Start", command=self.start, state="disabled")
        self.btn_stop = ttk.Button(ctrl, text="Stop", command=self.stop, state="disabled")
        self.btn_start.grid(row=0, column=0, padx=(0, 8))
        self.btn_stop.grid(row=0, column=1, padx=(0, 8))
        ttk.Button(ctrl, text="Reset Integral Now", command=self._reset_integral_now).grid(row=0, column=2)

        # Live readouts
        live = ttk.LabelFrame(self, text="Live")
        live.grid(row=2, column=0, sticky="nsew", **pad)
        live.columnconfigure(1, weight=1)
        live.columnconfigure(2, weight=1)

        temp_value = ttk.Label(live, textvariable=self.var_temp, font=("TkDefaultFont", 18, "bold"))
        ttk.Label(live, text="Temperature").grid(row=0, column=0, sticky="w", padx=8, pady=3)
        temp_value.grid(row=0, column=1, sticky="w", padx=8, pady=3)

        def live_row(r, name, var):
            ttk.Label(live, text=name).grid(row=r, column=0, sticky="w", padx=8, pady=3)
            ttk.Label(live, textvariable=var).grid(row=r, column=1, sticky="w", padx=8, pady=3)

        live_row(1, "Std dev (last 100)", self.var_temp_std)
        live_row(2, "Error (SP - T)", self.var_error)
        live_row(3, "Heater power", self.var_u_cmd)
        live_row(4, "  P term", self.var_p_term)
        live_row(5, "  I term", self.var_i_term)
        live_row(6, "  D term", self.var_d_term)
        live_row(7, "Status", self.var_status)

        # Presets
        presets = ttk.LabelFrame(self, text="Presets")
        presets.grid(row=3, column=0, sticky="nsew", **pad)
        presets.columnconfigure(0, weight=1)

        self.lst_presets = tk.Listbox(presets, height=6)
        self.lst_presets.grid(row=0, column=0, sticky="nsew", padx=8, pady=6)
        self.lst_presets.bind("<<ListboxSelect>>", self._on_preset_selected)

        pbtn = ttk.Frame(presets)
        pbtn.grid(row=0, column=1, sticky="ns", padx=8, pady=6)
        ttk.Button(pbtn, text="Save Preset…", command=self._save_preset_dialog).grid(row=0, column=0, sticky="ew", pady=(0, 6))
        ttk.Button(pbtn, text="Load Preset", command=self._load_selected_preset).grid(row=1, column=0, sticky="ew", pady=(0, 6))
        ttk.Button(pbtn, text="Delete Preset", command=self._delete_selected_preset).grid(row=2, column=0, sticky="ew", pady=(0, 6))

        # Quit cleanly
        self.protocol("WM_DELETE_WINDOW", self._on_close)
        self._set_connection_status("Disconnected", "#b91c1c")

    def _set_connection_status(self, text, color):
        self.var_connection_status.set(text)
        self.connection_indicator.configure(bg=color)

    def _show_help(self):
        messagebox.showinfo(
            "Help",
            "Connect to the controller, verify the thermometer and heater channels, "
            "then start the PI loop. The live panel shows the latest temperature and "
            "the rolling standard deviation of the last 100 measurements.",
        )

    @staticmethod
    def _short_error_text(exc, max_len=48):
        msg = str(exc).strip() or exc.__class__.__name__
        if len(msg) <= max_len:
            return msg
        return msg[: max_len - 3] + "..."

    def _set_start_button_state(self):
        if self.running:
            self.btn_start.configure(state="disabled")
            self.btn_stop.configure(state="normal")
        elif self.controller is None:
            self.btn_start.configure(state="disabled")
            self.btn_stop.configure(state="disabled")
        else:
            self.btn_start.configure(state="normal")
            self.btn_stop.configure(state="disabled")

    def connect_controller(self):
        ip = self.var_controller_ip.get().strip()
        timeout = float(self.var_controller_timeout.get())
        previous_controller = self.controller

        if not ip:
            messagebox.showerror("Controller", "Enter a controller IP address.")
            return

        if self.running:
            self.stop()

        if previous_controller is not None and hasattr(previous_controller, "close"):
            previous_controller.close()
        self.controller = None

        self._set_connection_status("Connecting…", "#ca8a04")
        self.update_idletasks()

        try:
            try:
                controller = self.controller_factory(
                    DEVICE_IP=ip,
                    TIMEOUT=timeout,
                    CHANNEL_NUMBER_THERM=int(self.var_temp_channel.get()),
                    CHANNEL_NUMBER_HEAT=int(self.var_heater_channel.get()),
                )
            except TypeError:
                controller = self.controller_factory()

            if hasattr(controller, "connect"):
                controller.connect()

            self.controller = controller
        except Exception as exc:
            self.controller = None
            if 'controller' in locals() and hasattr(controller, "close"):
                controller.close()
            short_error = self._short_error_text(exc)
            self._set_connection_status(f"Connect failed: {short_error}", "#b91c1c")
            self._set_start_button_state()
            messagebox.showerror("Controller Connection Failed", str(exc))
            return

        self._set_connection_status(f"Connected to {ip}", "#15803d")
        self.var_status.set("Stopped")
        self._set_start_button_state()
        self._idle_poll_temperature_once()

    # ---------------- Control loop ----------------
    def start(self):
        if self.running:
            return
        if self.controller is None:
            messagebox.showinfo("Controller", "Connect to the controller first.")
            return
        if self.var_reset_integral_on_start.get():
            self.pi.reset_state()

        self.running = True
        self.last_update_time = None
        self.var_status.set("Running")
        self._set_start_button_state()
        self._control_tick()

    def stop(self):
        self.running = False
        self.var_status.set("Stopped")
        self._set_start_button_state()

    def _reset_integral_now(self):
        self.pi.reset_integral()
        self.var_i_term.set(f"{self.pi.integral_uW:.5f} µW")
        self.var_d_term.set("—")

    def _record_temperature(self, temperature):
        self.temp_history.append(float(temperature))
        if len(self.temp_history) >= 2:
            self.var_temp_std.set(f"{stdev(self.temp_history):.6f} K")
        else:
            self.var_temp_std.set("—")

    def _control_tick(self):
        if not self.running:
            return

        # Compute dt: use configured dt, but also guard if scheduling jitters
        dt_cfg = max(0.1, float(self.var_dt.get()))
        now = time.time()
        if self.last_update_time is None:
            dt = dt_cfg
        else:
            dt = max(0.0, now - self.last_update_time)
            # Keep dt from being crazy if GUI is stalled
            dt = min(max(dt, 0.5 * dt_cfg), 2.0 * dt_cfg)

        self.last_update_time = now

        # Read temperature
        try:
            T = float(self.controller.get_temperature(channel_number=int(self.var_temp_channel.get())))
        except LookupError:
            self.last_update_time = None
            self.var_status.set(f"Waiting for thermometer ch {int(self.var_temp_channel.get())}")
            self.after(250, self._control_tick)
            return
        except Exception as e:
            traceback.print_exc()
            self.var_status.set(f"Error reading temperature: {e}")
            self.stop()
            return

        sp = float(self.var_setpoint.get())
        kp = float(self.var_kp.get())
        ki = float(self.var_ki.get())
        kd = float(self.var_kd.get())
        d_smooth_points = int(self.var_d_smooth_points.get())
        u_ff = float(self.var_uff.get())
        u_min = float(self.var_u_min.get())
        u_max = float(self.var_u_max.get())
        antiwindup = bool(self.var_antiwindup.get())

        u_cmd, e, p_term, i_term, d_term = self.pi.update(
            setpoint=sp,
            measurement=T,
            kp=kp,
            ki_per_s=ki,
            kd=kd,
            deriv_points=d_smooth_points,
            dt_s=dt,
            u_ff=u_ff,
            u_min=u_min,
            u_max=u_max,
            antiwindup=antiwindup,
        )

        # Apply heater command
        try:
            self.controller.set_heater(u_cmd, channel_number=int(self.var_heater_channel.get()))
        except Exception as e2:
            traceback.print_exc()
            self.var_status.set(f"Error setting heater: {e2}")
            self.stop()
            return

        # Update readouts
        self._record_temperature(T)
        self.var_temp.set(f"{T:.5f} K")
        self.var_error.set(f"{e:.5f} K")
        self.var_u_cmd.set(f"{u_cmd:.5f} µW")
        self.var_p_term.set(f"{p_term:.5f} µW")
        self.var_i_term.set(f"{i_term:.5f} µW")
        self.var_d_term.set(f"{d_term:.5f} µW")

        # Schedule next tick aligned to dt_cfg
        self.after(int(dt_cfg * 1000), self._control_tick)

    def _idle_poll_temperature_once(self):
        if self.controller is None:
            return
        try:
            T = float(self.controller.get_temperature(channel_number=int(self.var_temp_channel.get())))
            # self._record_temperature(T)
            self.var_temp.set(f"{T:.5f} K")
        except LookupError:
            pass
        except Exception:
            pass

    def _idle_poll_temperature(self):
        # Keep showing temperature even when stopped
        self._idle_poll_temperature_once()
        self.after(1000, self._idle_poll_temperature)

    # ---------------- Presets ----------------
    def _current_settings_dict(self):
        return {
            "setpoint": float(self.var_setpoint.get()),
            "kp": float(self.var_kp.get()),
            "ki": float(self.var_ki.get()),
            "kd": float(self.var_kd.get()),
            "d_smooth_points": int(self.var_d_smooth_points.get()),
            "u_ff": float(self.var_uff.get()),
            "dt": float(self.var_dt.get()),
            "u_min": float(self.var_u_min.get()),
            "u_max": float(self.var_u_max.get()),
            "reset_integral_on_start": bool(self.var_reset_integral_on_start.get()),
            "keep_integral_on_tune": bool(self.var_keep_integral_on_tune.get()),
            "antiwindup": bool(self.var_antiwindup.get()),
            "controller_ip": self.var_controller_ip.get(),
            "controller_timeout": float(self.var_controller_timeout.get()),
            "temp_channel": int(self.var_temp_channel.get()),
            "heater_channel": int(self.var_heater_channel.get()),
        }

    def _apply_settings_dict(self, d):
        # Optionally reset integral when tuning while running
        # (the trace handlers also catch some changes, but loading a preset changes many at once)
        keep_int = bool(self.var_keep_integral_on_tune.get())
        was_running = self.running

        self.var_setpoint.set(d.get("setpoint", self.var_setpoint.get()))
        self.var_kp.set(d.get("kp", self.var_kp.get()))
        self.var_ki.set(d.get("ki", self.var_ki.get()))
        self.var_kd.set(d.get("kd", self.var_kd.get()))
        self.var_d_smooth_points.set(d.get("d_smooth_points", self.var_d_smooth_points.get()))
        self.var_uff.set(d.get("u_ff", self.var_uff.get()))
        self.var_dt.set(d.get("dt", self.var_dt.get()))
        self.var_u_min.set(d.get("u_min", self.var_u_min.get()))
        self.var_u_max.set(d.get("u_max", self.var_u_max.get()))

        self.var_reset_integral_on_start.set(d.get("reset_integral_on_start", self.var_reset_integral_on_start.get()))
        self.var_keep_integral_on_tune.set(d.get("keep_integral_on_tune", self.var_keep_integral_on_tune.get()))
        self.var_antiwindup.set(d.get("antiwindup", self.var_antiwindup.get()))
        self.var_controller_ip.set(d.get("controller_ip", self.var_controller_ip.get()))
        self.var_controller_timeout.set(d.get("controller_timeout", self.var_controller_timeout.get()))
        self.var_temp_channel.set(d.get("temp_channel", self.var_temp_channel.get()))
        self.var_heater_channel.set(d.get("heater_channel", self.var_heater_channel.get()))

        if was_running and (not keep_int):
            self.pi.reset_state()

    def _refresh_preset_list(self):
        names = sorted(self.presets.keys())
        self.lst_presets.delete(0, tk.END)
        for n in names:
            self.lst_presets.insert(tk.END, n)

    def _selected_preset_name(self):
        sel = self.lst_presets.curselection()
        if not sel:
            return None
        return self.lst_presets.get(sel[0])

    def _save_preset_dialog(self):
        name = simpledialog.askstring("Save Preset", "Preset name:")
        if not name:
            return
        self.presets[name] = self._current_settings_dict()
        try:
            save_presets(self.presets)
        except Exception as e:
            messagebox.showerror("Error", f"Failed to save presets: {e}")
            return
        self._refresh_preset_list()

    def _load_selected_preset(self):
        name = self._selected_preset_name()
        if not name:
            messagebox.showinfo("Presets", "Select a preset to load.")
            return
        self._apply_settings_dict(self.presets[name])

    def _delete_selected_preset(self):
        name = self._selected_preset_name()
        if not name:
            messagebox.showinfo("Presets", "Select a preset to delete.")
            return
        if not messagebox.askyesno("Delete Preset", f"Delete preset '{name}'?"):
            return
        self.presets.pop(name, None)
        try:
            save_presets(self.presets)
        except Exception as e:
            messagebox.showerror("Error", f"Failed to save presets: {e}")
            return
        self._refresh_preset_list()

    def _on_preset_selected(self, _evt):
        # no auto-load; selection just selects
        pass

    # ---------------- Integral-reset-on-tune behavior ----------------
    def _on_tune_param_changed(self, *_args):
        if not self.running:
            return
        if self.var_keep_integral_on_tune.get():
            return
        # If running and user changes Kp/Ki/u_ff, reset integrator
        self.pi.reset_state()
        self.var_i_term.set(f"{self.pi.integral_uW:.5f} µW")
        self.var_d_term.set("—")

    def _on_close(self):
        # Best-effort: stop loop and optionally set heater to feedforward or 0
        self.stop()
        if self.controller is not None:
            if hasattr(self.controller, "close"):
                self.controller.close()
        self.destroy()


def main():
    app = App(BlueforsController)
    app.mainloop()


if __name__ == "__main__":
    main()

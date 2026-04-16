# Temperature Controller

Tkinter GUI for closed-loop temperature control using PID with a feedforward term. The application is designed to talk to a controller object that can read temperature and set heater power, and the current implementation targets a Bluefors HTTP API.

## Features

- PID + feedforward control: `u = u_ff + Kp*e + Ki*sum(e*dt) - Kd*de/dt`
- Manual connection settings for controller IP, timeout, thermometer channel, and heater channel
- Start/stop control loop with configurable sample time
- Anti-windup integrator clamping
- Optional integral reset on start
- Optional integral reset when tuning while the loop is running
- Derivative smoothing based on a rolling linear-fit window
- Live readouts for temperature, error, heater output, and PID terms
- Rolling standard deviation over the last 100 temperature samples
- JSON-based preset save/load/delete support

## Requirements

- Python 3
- `requests`
- Tkinter available in your Python environment

Install the external dependency with:

```bash
pip install requests
```

## Running

From the repository root:

```bash
python temperature_controller.py
```

The application launches a desktop GUI titled `PID+Feedforward Temp Ctrl`.

## Controller Interface

The GUI expects a controller object with the following behavior:

```python
controller.get_temperature() -> float
controller.set_heater(power_uW: float)
```

In this repository:

- `BlueforsController` connects to a device over HTTP on port `5001`
- `DummyController` provides a simple simulated thermal system for local testing

The current `main()` function starts the app with `BlueforsController`.

## Bluefors Integration

`BlueforsController`:

- Connects to `http://<DEVICE_IP>:5001/system`
- Polls `http://<DEVICE_IP>:5001/channel/measurement/latest` in a background thread
- Reads channel settings via `POST /channel`
- Sends heater updates via `POST /heater/update`

Heater power is entered in microwatts in the GUI and converted to watts before sending to the Bluefors API.

## Main Controls

The GUI exposes:

- Setpoint
- `Kp`, `Ki`, `Kd`
- Derivative smoothing window
- Feedforward heater power `u_ff`
- Sample time `dt`
- Minimum and maximum heater output
- Integral handling and anti-windup options

Live status fields show:

- Current temperature
- Standard deviation of recent measurements
- Error (`setpoint - temperature`)
- Commanded heater power
- Individual P, I, and D contributions
- Loop status

## Presets

Presets are stored in [`presets.json`](/home/tomren/repo/tempcontrol/presets.json). Saved presets include tuning values, loop options, and controller connection settings.

## Notes on Units

- Temperature is displayed in kelvin in the current GUI labels
- `Kp` should be in `uW / K`
- `Ki` should be in `uW / K / s`
- `Kd` should be in `uW * s / K`
- Heater output is clamped between the configured minimum and maximum values

## Shutdown Behavior

Closing the window stops the control loop and closes the controller connection if the controller provides a `close()` method.

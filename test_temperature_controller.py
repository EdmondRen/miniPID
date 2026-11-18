import sys
import types
import unittest
from unittest import mock


try:
    import requests  # noqa: F401
except ModuleNotFoundError:
    sys.modules["requests"] = types.ModuleType("requests")

from temperature_controller import App


class FakeVar:
    def __init__(self, value):
        self.value = value
        self.write_count = 0

    def get(self):
        return self.value

    def set(self, value):
        self.value = value
        self.write_count += 1


class FakeButton:
    def __init__(self):
        self.state = None

    def configure(self, **kwargs):
        self.state = kwargs.get("state", self.state)


class FakePID:
    def update(self, **_kwargs):
        return 1.25, 0.01, 0.1, 0.2, 0.3


class FakeController:
    def __init__(self, read_results=None, write_results=None):
        self.read_results = list(read_results or [0.05])
        self.write_results = list(write_results or [None])
        self.heater_commands = []

    @staticmethod
    def _next_result(results):
        result = results.pop(0) if len(results) > 1 else results[0]
        if isinstance(result, Exception):
            raise result
        return result

    def get_temperature(self, channel_number=None):
        return self._next_result(self.read_results)

    def set_heater(self, power, channel_number=None):
        result = self._next_result(self.write_results)
        self.heater_commands.append((power, channel_number))
        return result


def make_app(controller):
    app = object.__new__(App)
    app.controller = controller
    app.pid = FakePID()
    app.running = True
    app.last_update_time = None
    app._control_after_id = None
    app._consecutive_control_failures = 0

    app.var_dt = FakeVar(1.0)
    app.var_temp_channel = FakeVar(6)
    app.var_heater_channel = FakeVar(4)
    app.var_setpoint = FakeVar(0.06)
    app.var_kp = FakeVar(1.0)
    app.var_ki = FakeVar(1.0)
    app.var_kd = FakeVar(1.0)
    app.var_d_smooth_points = FakeVar(5)
    app.var_uff = FakeVar(1.0)
    app.var_u_min = FakeVar(0.0)
    app.var_u_max = FakeVar(10.0)
    app.var_antiwindup = FakeVar(True)

    app.var_status = FakeVar("Running")
    app.var_temp = FakeVar("—")
    app.var_temp_std = FakeVar("—")
    app.var_error = FakeVar("—")
    app.var_u_cmd = FakeVar("—")
    app.var_p_term = FakeVar("—")
    app.var_i_term = FakeVar("—")
    app.var_d_term = FakeVar("—")
    app.temp_history = []

    app.btn_start = FakeButton()
    app.btn_stop = FakeButton()

    app._scheduled = {}
    app._cancelled = []
    app._next_after_id = 0

    def after(delay_ms, callback):
        app._next_after_id += 1
        after_id = f"after-{app._next_after_id}"
        app._scheduled[after_id] = (delay_ms, callback)
        return after_id

    def after_cancel(after_id):
        app._cancelled.append(after_id)
        app._scheduled.pop(after_id, None)

    app.after = after
    app.after_cancel = after_cancel
    return app


def run_scheduled_tick(app):
    after_id = app._control_after_id
    _delay_ms, callback = app._scheduled.pop(after_id)
    callback()


class ControlLoopTests(unittest.TestCase):
    @mock.patch("temperature_controller.traceback.print_exc")
    def test_transient_failures_recover_without_stopping(self, _print_exc):
        controller = FakeController(
            read_results=[RuntimeError("timeout"), RuntimeError("timeout"), 0.05]
        )
        app = make_app(controller)

        app._control_tick()
        self.assertTrue(app.running)
        self.assertEqual(app._consecutive_control_failures, 1)
        self.assertIn("retrying", app.var_status.get())

        run_scheduled_tick(app)
        self.assertTrue(app.running)
        self.assertEqual(app._consecutive_control_failures, 2)

        run_scheduled_tick(app)
        self.assertTrue(app.running)
        self.assertEqual(app._consecutive_control_failures, 0)
        self.assertEqual(app.var_status.get(), "Running")
        self.assertEqual(len(controller.heater_commands), 1)

    @mock.patch("temperature_controller.traceback.print_exc")
    def test_three_heater_failures_stop_and_preserve_reason(self, _print_exc):
        controller = FakeController(
            write_results=[
                RuntimeError("heater timeout"),
                RuntimeError("heater timeout"),
                RuntimeError("heater timeout"),
            ]
        )
        app = make_app(controller)

        app._control_tick()
        run_scheduled_tick(app)
        run_scheduled_tick(app)

        self.assertFalse(app.running)
        self.assertIsNone(app._control_after_id)
        self.assertEqual(app._scheduled, {})
        self.assertIn("Stopped — Heater update failed after 3 attempts", app.var_status.get())
        self.assertIn("heater timeout", app.var_status.get())

    def test_scheduling_replaces_and_stop_cancels_pending_tick(self):
        app = make_app(FakeController())

        app._schedule_control_tick(1000)
        first_id = app._control_after_id
        app._schedule_control_tick(2000)
        second_id = app._control_after_id

        self.assertNotEqual(first_id, second_id)
        self.assertIn(first_id, app._cancelled)
        self.assertEqual(list(app._scheduled), [second_id])

        app.stop()
        self.assertIn(second_id, app._cancelled)
        self.assertEqual(app._scheduled, {})
        self.assertEqual(app.var_status.get(), "Stopped")


class NumericCommitTests(unittest.TestCase):
    def setUp(self):
        self.app = object.__new__(App)

    def test_draft_does_not_change_active_value_until_commit(self):
        active = FakeVar(5.0)
        draft = FakeVar("100")

        self.assertEqual(active.get(), 5.0)
        self.app._commit_numeric_entry(draft, active, "Kp", integer=False)

        self.assertEqual(active.get(), 100.0)
        self.assertEqual(active.write_count, 1)

        self.app._commit_numeric_entry(draft, active, "Kp", integer=False)
        self.assertEqual(active.write_count, 1)

    @mock.patch("temperature_controller.messagebox.showerror")
    def test_invalid_draft_restores_last_committed_value(self, showerror):
        active = FakeVar(5.0)
        draft = FakeVar("-")

        self.app._commit_numeric_entry(draft, active, "Kp", integer=False)

        self.assertEqual(active.get(), 5.0)
        self.assertEqual(active.write_count, 0)
        self.assertEqual(draft.get(), "5.0")
        showerror.assert_called_once()

    def test_integer_commit_uses_integer_parser(self):
        active = FakeVar(5)
        draft = FakeVar("20")

        self.app._commit_numeric_entry(
            draft,
            active,
            "Derivative smoothing",
            integer=True,
        )

        self.assertEqual(active.get(), 20)
        self.assertIsInstance(active.get(), int)


if __name__ == "__main__":
    unittest.main()

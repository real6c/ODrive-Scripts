#!/usr/bin/env python3
"""
Live graph of ODrive telemetry (position, velocity, torque, current, voltage).

Expose `start_live_plotting(odrv)` so callers can reuse an existing connection and
display a rolling window of telemetry values similar to the printout emitted by
`motor_functions.trap_traj`.
"""

from collections import deque
from threading import Event, Thread
from typing import Deque, Optional, Tuple

import matplotlib.pyplot as plt
from odrive.enums import AxisState

# Prefer a dark appearance for all plots
plt.style.use("dark_background")

# Number of points to keep on the live plot
HISTORY_LENGTH = 100

_plot_thread: Optional[Thread] = None
_stop_event: Optional[Event] = None


def _init_buffers() -> Tuple[Deque[float], ...]:
    """Create empty deques for each metric we want to plot."""
    return tuple(deque(maxlen=HISTORY_LENGTH) for _ in range(6))


def _run_plotting(odrv) -> None:
    global _plot_thread, _stop_event

    plt.ion()
    axis = odrv.axis0

    pos_history, target_history, vel_history, torque_history, current_history, voltage_history = _init_buffers()

    fig, axes = plt.subplots(3, 2, figsize=(10, 8))
    fig.patch.set_facecolor("#101214")
    fig.suptitle("ODrive Live Telemetry", color="#FFFFFF")

    plots = {}
    labels = [
        ("Position (turns)", pos_history, target_history),
        ("Velocity (turns/s)", vel_history, None),
        ("Torque (Nm)", torque_history, None),
        ("Current (A)", current_history, None),
        ("Bus Voltage (V)", voltage_history, None),
    ]

    axes_flat = axes.flatten()
    for ax in axes_flat:
        ax.set_facecolor("#14171A")
        ax.spines["bottom"].set_color("#555555")
        ax.spines["top"].set_color("#555555")
        ax.spines["left"].set_color("#555555")
        ax.spines["right"].set_color("#555555")
        ax.tick_params(axis="x", colors="#DDDDDD")
        ax.tick_params(axis="y", colors="#DDDDDD")
        ax.grid(color="#233445", linestyle="--", linewidth=0.6, alpha=0.6)

    # Map each subplot so we can update it on every animation frame
    for ax, (title, primary_history, secondary_history) in zip(
        axes_flat[: len(labels)], labels
    ):
        ax.set_title(title, color="#FFFFFF")
        ax.set_xlim(0, HISTORY_LENGTH)
        line_primary, = ax.plot(
            [], [], label="POS" if "Position" in title else "Value", color="#4FB3F6"
        )
        plots[title] = {"axis": ax, "primary": line_primary}

        if secondary_history is not None:
            line_secondary, = ax.plot(
                [], [], label="TARGET", linestyle="--", color="#FFB74D"
            )
            plots[title]["secondary"] = line_secondary
            legend = ax.legend(loc="upper left", facecolor="#1C1F23", edgecolor="#46505A")
            for text in legend.get_texts():
                text.set_color("#FFFFFF")

    # Any remaining axes (typically just the last one) are hidden before reuse
    for ax in axes_flat[len(labels) : -1]:
        ax.axis("off")

    status_ax = axes_flat[-1]
    status_ax.axis("off")
    status_text = status_ax.text(
        0.5,
        0.5,
        "Motor DISARMED",
        color="#53E89B",
        ha="center",
        va="center",
        fontsize=18,
        fontweight="bold",
        transform=status_ax.transAxes,
    )

    def update() -> Tuple:
        current_pos = axis.pos_estimate
        target_pos = axis.controller.input_pos
        current_vel = axis.vel_estimate
        torque_estimate = axis.motor.torque_estimate
        bus_voltage = odrv.vbus_voltage
        motor_current = (
            axis.motor.electrical_power / bus_voltage if bus_voltage else 0.0
        )

        pos_history.append(current_pos)
        target_history.append(target_pos)
        vel_history.append(current_vel)
        torque_history.append(torque_estimate)
        current_history.append(motor_current)
        voltage_history.append(bus_voltage)

        def _set_line(history, line, ax_obj):
            if not history:
                return
            ax_obj.set_ylim(min(history) - 0.1, max(history) + 0.1)
            line.set_data(range(len(history)), list(history))

        _set_line(pos_history, plots["Position (turns)"]["primary"], plots["Position (turns)"]["axis"])
        _set_line(target_history, plots["Position (turns)"]["secondary"], plots["Position (turns)"]["axis"])
        _set_line(vel_history, plots["Velocity (turns/s)"]["primary"], plots["Velocity (turns/s)"]["axis"])
        _set_line(torque_history, plots["Torque (Nm)"]["primary"], plots["Torque (Nm)"]["axis"])
        _set_line(current_history, plots["Current (A)"]["primary"], plots["Current (A)"]["axis"])
        _set_line(voltage_history, plots["Bus Voltage (V)"]["primary"], plots["Bus Voltage (V)"]["axis"])

        motor_state = axis.current_state
        if motor_state == AxisState.CLOSED_LOOP_CONTROL:
            status_text.set_text("Motor ARMED")
            status_text.set_color("#FF6B6B")
        else:
            status_text.set_text("Motor DISARMED")
            status_text.set_color("#53E89B")

        return tuple(
            line
            for lines in plots.values()
            for key, line in lines.items()
            if key != "axis"
        )

    plt.tight_layout()
    fig.show()

    try:
        while _stop_event and not _stop_event.is_set():
            update()
            fig.canvas.draw_idle()
            plt.pause(0.05)
    finally:
        plt.ioff()
        plt.close(fig)
        _plot_thread = None
        _stop_event = None


def start_live_plotting(odrv, *, daemon: bool = True) -> Thread:
    """
    Launch the live plotter reusing the provided ODrive instance.

    Returns
    -------
    threading.Thread
        Thread object running the matplotlib event loop. Call ``join()`` if you need
        to wait for it to finish.
    """

    global _plot_thread, _stop_event

    if _plot_thread and _plot_thread.is_alive():
        return _plot_thread

    _stop_event = Event()
    thread = Thread(target=_run_plotting, args=(odrv,), daemon=daemon)
    _plot_thread = thread
    thread.start()
    return thread


def stop_live_plotting() -> None:
    """Request that the live plot window close and wait for the thread to exit."""
    global _plot_thread, _stop_event

    if _stop_event:
        _stop_event.set()

    if _plot_thread:
        _plot_thread.join(timeout=1.0)
        if _plot_thread and _plot_thread.is_alive():
            return
        _plot_thread = None


__all__ = ["start_live_plotting", "stop_live_plotting"]

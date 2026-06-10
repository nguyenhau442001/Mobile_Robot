#!/usr/bin/env python3
"""Real-time velocity plotter: cmd_vel setpoint vs odom actual."""
from collections import deque
import sys
import time

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import pyqtgraph as pg
from pyqtgraph.Qt import QtCore
from pyqtgraph.Qt import QtWidgets
import rclpy
from rclpy.node import Node

WINDOW_SEC = 10.0


class VelocityPlotter(Node):
    """ROS2 node that buffers cmd_vel and odom velocity for live plotting."""

    def __init__(self):
        """Initialise subscriptions and rolling time-series buffers."""
        super().__init__('velocity_plotter')
        self._t0 = time.time()

        self._cmd_t = deque()
        self._cmd_v = deque()
        self._odom_t = deque()
        self._odom_v = deque()

        self.create_subscription(
            Twist, '/cmd_vel',
            lambda msg: self._append(self._cmd_t, self._cmd_v, msg.linear.x), 10)

        self.create_subscription(
            Odometry, '/odom',
            lambda msg: self._append(self._odom_t, self._odom_v,
                                     msg.twist.twist.linear.x), 10)

    def _append(self, t_buf, v_buf, value):
        """Append a timestamped sample and evict points outside the window."""
        now = time.time() - self._t0
        t_buf.append(now)
        v_buf.append(value)

        while t_buf and (now - t_buf[0]) > WINDOW_SEC:
            t_buf.popleft()
            v_buf.popleft()


def main():
    """Entry point: spin the ROS2 node and drive the Qt event loop."""
    rclpy.init()
    node = VelocityPlotter()

    app = QtWidgets.QApplication(sys.argv)

    app.setStyle('Fusion')
    pg.setConfigOption('background', 'w')
    pg.setConfigOption('foreground', 'k')

    central = QtWidgets.QWidget()
    central.setStyleSheet('background-color: #f5f5f5;')
    layout = QtWidgets.QVBoxLayout(central)
    layout.setContentsMargins(12, 12, 12, 12)
    layout.setSpacing(8)

    win = pg.GraphicsLayoutWidget()
    layout.addWidget(win)

    plot = win.addPlot(title='cmd_vel vs odom')
    plot.setYRange(-6, 6)
    plot.setLabel('left', 'velocity', units='m/s')
    plot.setLabel('bottom', 'time', units='s')
    plot.addLegend(brush=pg.mkBrush(255, 255, 255, 200))
    plot.showGrid(x=True, y=True, alpha=0.3)

    line_cmd = plot.plot(
        pen=pg.mkPen('#e74c3c', width=2),
        name='cmd_vel (setpoint)')
    line_odom = plot.plot(
        pen=pg.mkPen('#2980b9', width=2),
        name='odom (actual)')

    pause_btn = QtWidgets.QPushButton('⏸ Pause')
    pause_btn.setCheckable(True)
    pause_btn.setFixedHeight(36)
    pause_btn.setStyleSheet("""
        QPushButton {
            background-color: #ffffff;
            border: 1px solid #bdc3c7;
            border-radius: 6px;
            font-size: 13px;
            padding: 0 16px;
        }
        QPushButton:hover {
            background-color: #ecf0f1;
        }
        QPushButton:checked {
            background-color: #2ecc71;
            color: white;
            border-color: #27ae60;
        }
    """)
    layout.addWidget(pause_btn)

    central.setWindowTitle('Velocity Tracker')
    central.resize(820, 460)
    central.show()

    def update():
        rclpy.spin_once(node, timeout_sec=0)

        if pause_btn.isChecked():
            return

        if node._cmd_t:
            line_cmd.setData(list(node._cmd_t), list(node._cmd_v))
        if node._odom_t:
            line_odom.setData(list(node._odom_t), list(node._odom_v))

        now = time.time() - node._t0
        plot.setXRange(max(0, now - WINDOW_SEC), now)

    def on_pause_toggled(checked: bool):
        pause_btn.setText('▶ Resume' if checked else '⏸ Pause')

    pause_btn.toggled.connect(on_pause_toggled)

    timer = QtCore.QTimer()
    timer.timeout.connect(update)
    timer.start(50)

    app.exec()

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

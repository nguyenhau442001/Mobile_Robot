"""Smoke test — import the velocity_plotter module so CI catches syntax errors
and missing stdlib imports without needing a live ROS graph or display."""
import sys
import types
import unittest.mock as mock


def _stub_ros_and_qt():
    """Install lightweight stubs for ROS2 and Qt dependencies."""
    # rclpy stubs
    rclpy_mod = types.ModuleType('rclpy')
    rclpy_mod.init = mock.MagicMock()
    rclpy_mod.shutdown = mock.MagicMock()
    rclpy_mod.spin_once = mock.MagicMock()
    node_mod = types.ModuleType('rclpy.node')
    node_mod.Node = object
    sys.modules.setdefault('rclpy', rclpy_mod)
    sys.modules.setdefault('rclpy.node', node_mod)

    # ROS message stubs
    for mod_name, attrs in [
        ('geometry_msgs', []),
        ('geometry_msgs.msg', ['Twist']),
        ('nav_msgs', []),
        ('nav_msgs.msg', ['Odometry']),
    ]:
        m = types.ModuleType(mod_name)
        for attr in attrs:
            setattr(m, attr, mock.MagicMock())
        sys.modules.setdefault(mod_name, m)

    # pyqtgraph / Qt stubs
    pg_mod = types.ModuleType('pyqtgraph')
    pg_mod.setConfigOption = mock.MagicMock()
    pg_mod.mkPen = mock.MagicMock()
    pg_mod.mkBrush = mock.MagicMock()
    pg_mod.GraphicsLayoutWidget = mock.MagicMock()
    sys.modules.setdefault('pyqtgraph', pg_mod)

    qt_core = types.ModuleType('pyqtgraph.Qt')
    sys.modules.setdefault('pyqtgraph.Qt', qt_core)

    qtcore_mod = types.ModuleType('pyqtgraph.Qt.QtCore')
    qtcore_mod.QTimer = mock.MagicMock()
    sys.modules.setdefault('pyqtgraph.Qt.QtCore', qtcore_mod)

    qtwidgets_mod = types.ModuleType('pyqtgraph.Qt.QtWidgets')
    qtwidgets_mod.QApplication = mock.MagicMock()
    qtwidgets_mod.QVBoxLayout = mock.MagicMock()
    qtwidgets_mod.QWidget = mock.MagicMock()
    qtwidgets_mod.QPushButton = mock.MagicMock()
    sys.modules.setdefault('pyqtgraph.Qt.QtWidgets', qtwidgets_mod)


_stub_ros_and_qt()


def test_import_velocity_plotter():
    from scripts import velocity_plotter
    assert callable(velocity_plotter.main)


def test_velocity_plotter_constants():
    from scripts import velocity_plotter
    assert velocity_plotter.WINDOW_SEC > 0

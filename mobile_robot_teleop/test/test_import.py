"""Smoke tests — import each entry-point module so CI catches syntax errors
and missing imports without needing the runtime ROS graph."""


def test_import_teleop_key():
    from mobile_robot_teleop import mobile_robot_teleop_key
    assert callable(mobile_robot_teleop_key.main)


def test_import_trapezoid_profile_controller():
    from mobile_robot_teleop import trapezoid_profile_controller
    assert callable(trapezoid_profile_controller.main)

def test_import():
    import pymoveit2

    assert hasattr(pymoveit2, "MoveIt2")
    assert hasattr(pymoveit2, "MoveIt2Gripper")
    assert hasattr(pymoveit2, "MoveIt2Servo")
    assert hasattr(pymoveit2, "GripperCommand")
    assert hasattr(pymoveit2, "GripperInterface")


def test_import_robots():
    from pymoveit2 import robots

    assert robots is not None

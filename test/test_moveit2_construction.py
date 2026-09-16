def test_fk_client_created_once(moveit2):
    moveit2.compute_fk_async(joint_state=[0.0] * 7)
    client_first = moveit2._MoveIt2__compute_fk_client
    moveit2.compute_fk_async(joint_state=[0.0] * 7)
    client_second = moveit2._MoveIt2__compute_fk_client
    assert client_first is client_second


def test_ik_client_created_once(moveit2):
    moveit2.compute_ik_async(position=(0.3, 0.0, 0.5), quat_xyzw=(0.0, 0.0, 0.0, 1.0))
    client_first = moveit2._MoveIt2__compute_ik_client
    moveit2.compute_ik_async(position=(0.3, 0.0, 0.5), quat_xyzw=(0.0, 0.0, 0.0, 1.0))
    client_second = moveit2._MoveIt2__compute_ik_client
    assert client_first is client_second


def test_all_public_properties_readable(moveit2):
    cls = type(moveit2)
    for name in dir(cls):
        if name.startswith("_"):
            continue
        attr = getattr(cls, name, None)
        if isinstance(attr, property):
            getattr(moveit2, name)


def test_cartesian_avoid_collisions_default_true(moveit2):
    assert moveit2.cartesian_avoid_collisions is True

cmake_minimum_required(VERSION 3.5)
project(pymoveit2)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# Install Python package
ament_python_install_package(pymoveit2)

# Install examples
set(EXAMPLES_DIR examples)
install(PROGRAMS
    ${EXAMPLES_DIR}/ex_allow_collisions.py
    ${EXAMPLES_DIR}/ex_clear_planning_scene.py
    ${EXAMPLES_DIR}/ex_collision_mesh.py
    ${EXAMPLES_DIR}/ex_collision_primitive.py
    ${EXAMPLES_DIR}/ex_fk.py
    ${EXAMPLES_DIR}/ex_gripper.py
    ${EXAMPLES_DIR}/ex_ik.py
    ${EXAMPLES_DIR}/ex_joint_goal.py
    ${EXAMPLES_DIR}/ex_orientation_path_constraint.py
    ${EXAMPLES_DIR}/ex_pose_goal.py
    ${EXAMPLES_DIR}/ex_servo.py
    DESTINATION lib/${PROJECT_NAME}
)

# Setup the project
ament_package()

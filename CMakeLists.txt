cmake_minimum_required(VERSION 3.10)
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
    ${EXAMPLES_DIR}/ex_doctor.py
    ${EXAMPLES_DIR}/ex_fk.py
    ${EXAMPLES_DIR}/ex_gripper.py
    ${EXAMPLES_DIR}/ex_ik.py
    ${EXAMPLES_DIR}/ex_joint_goal.py
    ${EXAMPLES_DIR}/ex_orientation_path_constraint.py
    ${EXAMPLES_DIR}/ex_pose_goal.py
    ${EXAMPLES_DIR}/ex_servo.py
    ${EXAMPLES_DIR}/ex_session.py
    DESTINATION lib/${PROJECT_NAME}
)

# Install example assets (e.g. default mesh for ex_collision_mesh.py)
install(DIRECTORY ${EXAMPLES_DIR}/assets
    DESTINATION lib/${PROJECT_NAME}
)

# Testing
if(BUILD_TESTING)
    find_package(ament_cmake_pytest REQUIRED)
    ament_add_pytest_test(pymoveit2_tests test
        WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
        ENV PYTEST_ADDOPTS=--ignore=${CMAKE_CURRENT_SOURCE_DIR}/test/integration
        TIMEOUT 300
    )
endif()

# Setup the project
ament_package()

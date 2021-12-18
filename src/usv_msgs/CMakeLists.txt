# All rights reserved.
cmake_minimum_required(VERSION 3.5)

### Export headers
project(usv_msgs)

# Generate messages
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/PlannerCostmap.idl"
  "action/PlanTrajectory.idl"
  "msg/BoundingBox.idl"
  "msg/BoundingBoxArray.idl"
  "msg/Complex32.idl"
  "msg/ControlDiagnostic.idl"
  "msg/DetectedObject.idl"
  "msg/DetectedObjectKinematics.idl"
  "msg/DetectedObjects.idl"
  "msg/DiagnosticHeader.idl"
  "msg/EnvEstimation.idl"
  "msg/HighLevelControlCommand.idl"
  "msg/MotorReport1.idl"
  "msg/MotorReport2.idl"
  "msg/ObjectClassification.idl"
  "msg/PointClusters.idl"
  "msg/PointXYZIF.idl"
  "msg/Quaternion32.idl"
  "msg/Shape.idl"
  "msg/Trajectory.idl"
  "msg/TrajectoryPoint.idl"
  "msg/VehicleControlCommand.idl"
  "msg/VehicleKinematicState.idl"
  DEPENDENCIES
    "builtin_interfaces"
    "action_msgs"
    "geometry_msgs"
    "sensor_msgs"
    "shape_msgs"
    "std_msgs"
  ADD_LINTER_TESTS
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package()

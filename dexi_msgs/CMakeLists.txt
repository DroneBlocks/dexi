cmake_minimum_required(VERSION 3.8)
project(dexi_msgs)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

# Build Messages and Services
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Prompt.msg"
  "msg/State.msg"
  "srv/GetTelemetry.srv"
  "srv/Load.srv"
  "srv/Navigate.srv"
  "srv/Run.srv"
  "srv/SetAltitude.srv"
  "srv/SetLedEffect.srv"
  "srv/SetPosition.srv"
  "srv/SetYaw.srv"
  "srv/Store.srv"
)

ament_package()



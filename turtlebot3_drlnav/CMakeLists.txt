cmake_minimum_required(VERSION 3.5)
project(awbudrl_ori)

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

################ FIND DEPENDENCIES  #################
find_package(ament_cmake REQUIRED)		# cpp (optional)
find_package(rclcpp REQUIRED)			# cpp (optional)
find_package(ament_cmake_python REQUIRED)   	# python (optional)
find_package(rclpy REQUIRED)                	# python (optional)
######################################################

# Install Python Module
ament_python_install_package(${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
  # Common
  common/graph.py
  common/logger.py
  common/ounoise.py
  common/replaybuffer.py
  common/settings.py
  common/storagemanager.py
  common/utilities.py
  common/visual.py
  # drl_environment
  drl_environment/drl_environment_real.py
  drl_environment/drl_environment.py
  # drl_gazebo
  drl_gazebo/drl_gazebo.py
  # drl_agent
  drl_agent/ddpg.py
  drl_agent/dqn.py
  drl_agent/drl_agent.py
  drl_agent/off_policy_agent.py
  drl_agent/sac.py
  drl_agent/td3.py
  DESTINATION lib/${PROJECT_NAME}
)


################ INSTALL LAUNCH, ETC #################
install(DIRECTORY
  # add directories here
  common
  drl_environment
  drl_gazebo
  drl_agent
  DESTINATION share/${PROJECT_NAME})

ament_package()

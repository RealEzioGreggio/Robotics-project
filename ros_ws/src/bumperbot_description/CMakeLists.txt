#Nel CMake dai istruzioni su come devono essere installati i file quando vai a buildare

cmake_minimum_required(VERSION 3.8)
project(bumperbot_description) # Questo è la variabile project name

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

# Volgio dirgli che deve installare altre cartelle
install(
  #Cartelle da insallare (del pkg) Tutte le cartelle extra che crei le metti qui
  DIRECTORY meshes urdf launch rviz models worlds photos
  #Devono essere installate dentro share dento bumperbot_description(PROJECT_NAME)
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

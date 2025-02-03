cmake_minimum_required(VERSION 3.8)
project(sine_wave)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(grayscale_image REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(ament_cmake_pytest REQUIRED)
find_package(ament_cmake_gtest REQUIRED)
find_package(GTest REQUIRED)
find_package(generate_parameter_library REQUIRED)

generate_parameter_library(
  publisher_params_cpp
  config/params.yaml
)

include_directories(
  ${rclcpp_INCLUDE_DIRS}  
  ${std_msgs_INCLUDE_DIRS} 
  ${OpenCV_INCLUDE_DIRS} 
  ${grayscale_image_INCLUDE_DIRS} 
  ${service_msgs_INCLUDE_DIRS}
  ${type_description_interfaces_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# Add executables for C++ sources
add_executable(sine_wave_publisher_cpp src/sine_wave_publisher_cpp.cpp)
ament_target_dependencies(sine_wave_publisher_cpp rclcpp std_msgs OpenCV grayscale_image)
target_link_libraries(sine_wave_publisher_cpp 
  ${rclcpp_LIBRARIES} 
  ${std_msgs_LIBRARIES} 
  ${OpenCV_LIBS} 
  ${grayscale_image_LIBRARIES}
  publisher_params_cpp
)

add_executable(sine_wave_subscriber_cpp src/sine_wave_subscriber_cpp.cpp)
ament_target_dependencies(sine_wave_subscriber_cpp rclcpp std_msgs grayscale_image)
target_link_libraries(sine_wave_subscriber_cpp 
  ${rclcpp_LIBRARIES} 
  ${std_msgs_LIBRARIES} 
  ${grayscale_image_LIBRARIES}
)

install(
  DIRECTORY include/sine_wave/
  DESTINATION include/${PROJECT_NAME}
)

install(DIRECTORY config image launch
  DESTINATION share/${PROJECT_NAME}/
)

install(TARGETS
  sine_wave_publisher_cpp
  sine_wave_subscriber_cpp
  publisher_params_cpp
  DESTINATION lib/${PROJECT_NAME}
)

# For Python files
ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  ${PROJECT_SOURCE_DIR}/sine_wave/generate_publisher_params.py
  ${PROJECT_SOURCE_DIR}/sine_wave/sine_wave_publisher.py
  ${PROJECT_SOURCE_DIR}/sine_wave/sine_wave_subscriber.py
  DESTINATION lib/${PROJECT_NAME}
)

enable_testing()

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

  # Add the C++ unit tests using Google Test
  find_package(GTest REQUIRED)

  # include_directories(
  #   ${rclcpp_INCLUDE_DIRS}  
  #   ${std_msgs_INCLUDE_DIRS} 
  #   ${OpenCV_INCLUDE_DIRS} 
  #   ${grayscale_image_INCLUDE_DIRS} 
  #   ${service_msgs_INCLUDE_DIRS}
  #   ${type_description_interfaces_INCLUDE_DIRS}
  #   ${CMAKE_CURRENT_SOURCE_DIR}/include
  # )

  # ament_add_gtest(test_publisher_cpp tests/test_publisher_cpp.cpp
  #   ${CMAKE_CURRENT_SOURCE_DIR}/config/params.yaml
  # )
  # target_include_directories(test_publisher_cpp PRIVATE 
  #   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  #   $<INSTALL_INTERFACE:include>
  #   ${rclcpp_INCLUDE_DIRS} 
  #   ${GTEST_INCLUDE_DIRS}
  #   ${CMAKE_BINARY_DIR}/${PROJECT_NAME}/include
  # )
  # target_link_libraries(test_publisher_cpp 
  #   ${rclcpp_LIBRARIES} 
  #   ${std_msgs_LIBRARIES} 
  #   ${OpenCV_LIBS} 
  #   ${grayscale_image_LIBRARIES}
  #   publisher_params_cpp
  #   gtest
  #   gtest_main
  # )

  # ament_add_gtest(test_subscriber_cpp tests/test_subscriber_cpp.cpp)
  # target_include_directories(test_subscriber_cpp PRIVATE
  #   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  #   $<INSTALL_INTERFACE:include>
  #   ${rclcpp_INCLUDE_DIRS}
  #   ${GTEST_INCLUDE_DIRS}
  #   ${CMAKE_BINARY_DIR}/${PROJECT_NAME}/include
  # )
  # target_link_libraries(test_subscriber_cpp 
  #   ${rclcpp_LIBRARIES} 
  #   ${std_msgs_LIBRARIES} 
  #   ${grayscale_image_LIBRARIES} 
  #   gtest
  #   gtest_main
  # )

  # Add the Python unit tests using pytest
  set(_pytest_tests
    tests/test_publisher.py
    tests/test_subscriber.py
  )
  foreach(_test_path ${_pytest_tests})
    get_filename_component(_test_name ${_test_path} NAME_WE)
    ament_add_pytest_test(${_test_name} ${_test_path}
      APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_BINARY_DIR}:${CMAKE_CURRENT_SOURCE_DIR}/${PROJECT_NAME}
      TIMEOUT 60
      WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
    )
  endforeach()
endif()

ament_package()

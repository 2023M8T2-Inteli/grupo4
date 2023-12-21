cmake_minimum_required(VERSION 3.8)
project(vallet_msgs)

# Variável de ambiente para garantir o encontro do caminho do NUMPY
# Caso necessário, set a variável de ambiente FNMPMAC com true
if($ENV{FNMPMAC})
    find_package(Python3 3.10.13 EXACT COMPONENTS Interpreter Development.NumPy)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Task.msg"
  DEPENDENCIES geometry_msgs
)

ament_package()
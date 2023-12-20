cmake_minimum_required(VERSION 3.8)
project(vallet_cpp)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

# FetchContent for managing dependencies
include(FetchContent)

FetchContent_Declare(json URL https://github.com/nlohmann/json/releases/download/v3.11.3/json.tar.xz)
FetchContent_MakeAvailable(json)

FetchContent_Declare(
        socketio_client_cpp
        GIT_REPOSITORY https://github.com/socketio/socket.io-client-cpp.git
        GIT_TAG master
        GIT_SUBMODULES_RECURSE ON
        # Omitting CONFIGURE_COMMAND, BUILD_COMMAND, INSTALL_COMMAND, and TEST_COMMAND
        # to let CMake use the default commands for the external project
)

FetchContent_GetProperties(socketio_client_cpp)
if (NOT socketio_client_cpp_POPULATED)
    FetchContent_Populate(socketio_client_cpp)
    set(BOOST_DATE_TIME_NO_LIB ON CACHE BOOL "" FORCE)
    set(BOOST_REGEX_NO_LIB ON CACHE BOOL "" FORCE)
    set(ASIO_STANDALONE ON CACHE BOOL "" FORCE)
    set(_WEBSOCKETPP_CPP11_STL_ ON CACHE BOOL "" FORCE)
    set(_WEBSOCKETPP_CPP11_FUNCTIONAL_ ON CACHE BOOL "" FORCE)
    # Agora, no diretório onde o conteúdo foi baixado, execute CMake e build.
    add_subdirectory(${socketio_client_cpp_SOURCE_DIR} ${socketio_client_cpp_BINARY_DIR})
endif ()

FetchContent_MakeAvailable(socketio_client_cpp)

include_directories(
        ${socketio_client_cpp_SOURCE_DIR}/lib/asio/asio/include
        ${socketio_client_cpp_SOURCE_DIR}/lib/websocketpp
        ${socketio_client_cpp_SOURCE_DIR}/lib/rapidjson/include
)

add_definitions(-DBOOST_DATE_TIME_NO_LIB)
add_definitions(-DBOOST_REGEX_NO_LIB)
add_definitions(-DASIO_STANDALONE)
add_definitions(-D_WEBSOCKETPP_CPP11_STL_)
add_definitions(-D_WEBSOCKETPP_CPP11_FUNCTIONAL_)

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(slam_toolbox REQUIRED)
find_package(vallet_msgs REQUIRED)

# -----------------> streamer <-----------------
add_executable(streamer src/streamer/main.cpp src/streamer/client/client.cpp)

target_include_directories(streamer PUBLIC
        ${socketio_client_cpp_SOURCE_DIR}/src
        ${socketio_client_cpp_SOURCE_DIR}/lib/asio/asio/include
        ${socketio_client_cpp_SOURCE_DIR}/lib/websocketpp
        ${socketio_client_cpp_SOURCE_DIR}/lib/rapidjson/include
)

target_sources(streamer PRIVATE
        ${socketio_client_cpp_SOURCE_DIR}/src/sio_client.cpp
        ${socketio_client_cpp_SOURCE_DIR}/src/sio_socket.cpp
        ${socketio_client_cpp_SOURCE_DIR}/src/internal/sio_client_impl.cpp
)

# Link the socketio_client_cpp library
target_link_libraries(streamer sioclient nlohmann_json::nlohmann_json)


ament_target_dependencies(streamer rclcpp geometry_msgs nav2_msgs tf2 tf2_geometry_msgs tf2_ros slam_toolbox)

install(TARGETS streamer DESTINATION lib/${PROJECT_NAME}/)
# -----------------> streamer <-----------------

# -----------------> logger <-----------------
add_executable(logger src/logger/main.cpp src/logger/text_saver/text_saver.cpp)

ament_target_dependencies(logger rclcpp geometry_msgs nav2_msgs tf2 tf2_geometry_msgs tf2_ros slam_toolbox vallet_msgs)

install(TARGETS logger DESTINATION lib/${PROJECT_NAME}/)
# -----------------> logger <-----------------

# -----------------> pose_queue <-----------------
add_executable(pose_queue src/pose_queue/main.cpp src/pose_queue/queue/queue.cpp src/streamer/client/client.cpp)

target_include_directories(pose_queue PUBLIC
        ${socketio_client_cpp_SOURCE_DIR}/src
        ${socketio_client_cpp_SOURCE_DIR}/lib/asio/asio/include
        ${socketio_client_cpp_SOURCE_DIR}/lib/websocketpp
        ${socketio_client_cpp_SOURCE_DIR}/lib/rapidjson/include
)

target_sources(pose_queue PRIVATE
        ${socketio_client_cpp_SOURCE_DIR}/src/sio_client.cpp
        ${socketio_client_cpp_SOURCE_DIR}/src/sio_socket.cpp
        ${socketio_client_cpp_SOURCE_DIR}/src/internal/sio_client_impl.cpp
)

target_link_libraries(pose_queue sioclient)

ament_target_dependencies(pose_queue rclcpp geometry_msgs nav2_msgs tf2 tf2_geometry_msgs tf2_ros slam_toolbox vallet_msgs std_msgs)

install(TARGETS pose_queue DESTINATION lib/${PROJECT_NAME}/)
# -----------------> pose_queue <-----------------

# -----------------> vallet <-----------------
add_executable(vallet src/vallet/main.cpp src/vallet/robot/robot.cpp)

ament_target_dependencies(vallet rclcpp geometry_msgs nav2_msgs tf2 tf2_geometry_msgs tf2_ros slam_toolbox vallet_msgs)

install(TARGETS vallet DESTINATION lib/${PROJECT_NAME}/)
# -----------------> vallet <-----------------

# -----------------> battery <-----------------
add_executable(battery src/battery/main.cpp src/battery/collector/collector.cpp src/streamer/client/client.cpp)

target_include_directories(battery PUBLIC
        ${socketio_client_cpp_SOURCE_DIR}/src
        ${socketio_client_cpp_SOURCE_DIR}/lib/asio/asio/include
        ${socketio_client_cpp_SOURCE_DIR}/lib/websocketpp
        ${socketio_client_cpp_SOURCE_DIR}/lib/rapidjson/include
)

target_sources(battery PRIVATE
        ${socketio_client_cpp_SOURCE_DIR}/src/sio_client.cpp
        ${socketio_client_cpp_SOURCE_DIR}/src/sio_socket.cpp
        ${socketio_client_cpp_SOURCE_DIR}/src/internal/sio_client_impl.cpp
)

# Link the socketio_client_cpp library
target_link_libraries(battery sioclient nlohmann_json::nlohmann_json)

ament_target_dependencies(battery rclcpp geometry_msgs nav2_msgs tf2
        tf2_geometry_msgs tf2_ros slam_toolbox vallet_msgs sensor_msgs)

install(TARGETS battery DESTINATION lib/${PROJECT_NAME}/)
# -----------------> battery <-----------------


if (BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    # the following line skips the linter which checks for copyrights
    # comment the line when a copyright and license is added to all source files
    set(ament_cmake_copyright_FOUND TRUE)
    # the following line skips cpplint (only works in a git repo)
    # comment the line when this package is in a git repo and when
    # a copyright and license is added to all source files
    set(ament_cmake_cpplint_FOUND TRUE)
    ament_lint_auto_find_test_dependencies()
endif ()

install(DIRECTORY
        launch
        DESTINATION share/${PROJECT_NAME}/
)

ament_package()

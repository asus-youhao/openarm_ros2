# cmake_minimum_required(VERSION 3.8)
# project(test_o6_touch)

# # Set C++ standard
# set(CMAKE_CXX_STANDARD 17)
# set(CMAKE_CXX_STANDARD_REQUIRED ON)

# # Find LinkerHandApi library
# find_library(LINKERHAND_LIB
#     NAMES linkerhand_cpp_sdk
#     PATHS /usr/local/lib /usr/lib
#     REQUIRED
# )

# # Add executable
# add_executable(test_o6_touch test_o6_touch.cpp)

# # Include directories
# target_include_directories(test_o6_touch PRIVATE
#     /usr/local/include/linkerhand-cpp-sdk
# )

# # Link libraries
# target_link_libraries(test_o6_touch
#     ${LINKERHAND_LIB}
#     pthread
# )

# # Install
# install(TARGETS test_o6_touch
#     DESTINATION bin
# )

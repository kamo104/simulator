cmake_minimum_required(VERSION 3.15) # Minimum version can vary; adjust as needed
project(simulator)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(JSON_BuildTests OFF CACHE INTERNAL "")

# Add subdirectory for json
add_subdirectory(submodules/json)

include_directories(${CMAKE_SOURCE_DIR}/Headers)

add_executable(simulator Sources/main.cpp)

# Link nlohmann_json library
target_link_libraries(simulator PRIVATE nlohmann_json::nlohmann_json)

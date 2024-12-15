cmake_minimum_required(VERSION 3.15) # Minimum version can vary; adjust as needed
project(simulator)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(JSON_BuildTests OFF CACHE INTERNAL "")

# Add subdirectory for json
add_subdirectory(submodules/json)

find_package(Boost REQUIRED COMPONENTS system filesystem thread regex)




include_directories(${CMAKE_SOURCE_DIR}/Headers)

add_executable(simulator main.cpp)

target_link_libraries(simulator PRIVATE 
    Boost::system
    Boost::filesystem
    Boost::thread
    Boost::regex
    nlohmann_json::nlohmann_json
)

# Include Boost
target_include_directories(simulator PRIVATE ${Boost_INCLUDE_DIRS})

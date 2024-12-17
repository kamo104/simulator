cmake_minimum_required(VERSION 3.15) # Minimum version can vary; adjust as needed
project(simulator)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(JSON_BuildTests OFF CACHE INTERNAL "")

# Add subdirectory for json
add_subdirectory(submodules/json)

set(Boost_USE_STATIC_LIBS ON)
# set(Boost_USE_STATIC_RUNTIME ON)

find_package(Boost REQUIRED COMPONENTS system filesystem thread regex url)

if(MSVC)
    add_compile_options(/bigobj)
endif()


include_directories(${CMAKE_SOURCE_DIR}/Headers)

add_executable(simulator main.cpp)

target_link_libraries(simulator PRIVATE 
    Boost::system
    Boost::filesystem
    Boost::thread
    Boost::regex
    Boost::url
    nlohmann_json::nlohmann_json
)

# Include Boost
target_include_directories(simulator PRIVATE ${Boost_INCLUDE_DIRS})

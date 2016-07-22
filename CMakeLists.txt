cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(test_texture)

set(CMAKE_BUILD_TYPE Release)
#SET (BOOST_ROOT "C:/Program Files/PCL 1.6.0/3rdParty/Boost")
#SET (BOOST_INCLUDEDIR "C:/Program Files/PCL 1.6.0/3rdParty/Boost/include")
#SET (BOOST_LIBRARYDIR "C:/Program Files/PCL 1.6.0/3rdParty/Boost/lib")

find_package(Boost REQUIRED)
find_package(PCL REQUIRED)

include_directories(${Boost_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})
link_directories(${Boost_LIBRARY_DIRS} ${PCL_LIBRARY_DIRS})
add_definitions(${Boost_DEFINITIONS} ${PCL_DEFINITIONS})

add_executable(test_texture mesh.h mesh.cpp)

target_link_libraries(test_texture ${Boost_LIBRARIES} ${PCL_LIBRARIES})
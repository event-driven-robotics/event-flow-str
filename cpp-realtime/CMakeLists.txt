# requires minimum cmake version
cmake_minimum_required(VERSION 3.5)

# produce the cmake var PROJECT_NAME
project(event-flow-str)

if(NOT CMAKE_BUILD_TYPE)
set_property(CACHE CMAKE_BUILD_TYPE PROPERTY VALUE "Release")
endif()

#include(GNUInstallDirs)

# mandatory use of these packages
find_package(YCM REQUIRED)
find_package(OpenCV REQUIRED)
find_package(YARP COMPONENTS os REQUIRED)
find_package(event-driven REQUIRED)
find_package(HDF5 REQUIRED COMPONENTS CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_executable(${PROJECT_NAME} yarp-app.cpp flow.h flow.cpp)
target_compile_features(${PROJECT_NAME} PRIVATE cxx_std_17)
#target_compile_options(sixdofobject PRIVATE -Werror -Wall -Wextra)
target_link_libraries(${PROJECT_NAME} PRIVATE  YARP::YARP_init
                                            YARP::YARP_os
                                            ev::event-driven
                                            ${OpenCV_LIBS})

add_executable(dsec-app dsec-app.cpp flow.h flow.cpp)
target_compile_features(dsec-app PRIVATE cxx_std_17)
target_include_directories(dsec-app PRIVATE ${HDF5_INCLUDE_DIRS})
target_link_libraries(dsec-app PRIVATE ${HDF5_CXX_LIBRARIES} ${HDF5_LIBRARIES} ${OpenCV_LIBS})

install(TARGETS ${PROJECT_NAME} dsec-app DESTINATION ${CMAKE_INSTALL_BINDIR})
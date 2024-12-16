cmake_minimum_required(VERSION 3.10)

project(dexhand021)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED True)
set(LIB_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../libs)

add_compile_options(-m64 -shared -fPIC)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../include)

ADD_EXECUTABLE(dexhand021 dexhand021.cpp)

target_link_libraries(dexhand021
        ${LIB_DIR}/libdexhand_sdk_cpp.so
        ${LIB_DIR}/libusb-1.0.so
        ${LIB_DIR}/libusbcanfd.so
)


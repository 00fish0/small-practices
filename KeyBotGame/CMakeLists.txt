cmake_minimum_required(VERSION 3.10)

project(KeyBotGame)

set(CMAKE_CXX_STANDARD 20)

include_directories(${PROJECT_SOURCE_DIR}/include)

aux_source_directory(${PROJECT_SOURCE_DIR}/src SRC_LIST)

add_executable(ketBotGame main.cpp ${SRC_LIST})

cmake_minimum_required(VERSION 3.10)
set(CMAKE_CPP_STANDARD 21)
set(CMAKE_CPP_STANDARD_REQUIRED True)


project(EvoRob VERSION 0.1)
set(SOURCES main.cpp src/utils.cpp src/controller.cpp src/map.cpp)
add_executable(EvoRob ${SOURCES})
target_include_directories(EvoRob PUBLIC "${PROJECT_BINARY_DIR}" ${CMAKE_CURRENT_SOURCE_DIR}/include)
configure_file(config.h.in config.h)


# Require matplotplusplus for visualization.
find_package(Matplot++ QUIET)
if(NOT Matplot++_FOUND)
    include(FetchContent)
    FetchContent_Declare(matplotplusplus
        GIT_REPOSITORY https://github.com/alandefreitas/matplotplusplus
        GIT_TAG v1.2.1)
    FetchContent_GetProperties(matplotplusplus)
    if(NOT matplotplusplus_POPULATED)
        FetchContent_Populate(matplotplusplus)
        add_subdirectory(${matplotplusplus_SOURCE_DIR} ${matplotplusplus_BINARY_DIR} EXCLUDE_FROM_ALL)
    endif()
endif()
target_link_libraries(EvoRob PUBLIC matplot)
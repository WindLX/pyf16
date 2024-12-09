cmake_minimum_required(VERSION 3.15)

project(f16_model)

set(CMAKE_C_STANDARD 17)
set(CMAKE_C_STANDARD_REQUIRED ON)

include_directories(include)

set(SOURCES "src/hifi_F16_AeroData.c"
            "src/lofi_F16_AeroData.c"
            "src/mexndinterp.c"
            # "src/leading_edge_flap.c"
            "src/model.c"
            # "src/hashmap.c"
            "src/utils.c")

if (WIN32)
    set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS on)
endif()

add_library(f16_model SHARED ${SOURCES})

if (UNIX)
    target_link_libraries(f16_model m)
endif()

set(CMAKE_INSTALL_PREFIX ${CMAKE_SOURCE_DIR}/install CACHE PATH "f16_model default install prefix" FORCE)

if (NOT EXISTS ${CMAKE_INSTALL_PREFIX})
    file(MAKE_DIRECTORY ${CMAKE_INSTALL_PREFIX})
endif()

install(TARGETS f16_model DESTINATION ${CMAKE_INSTALL_PREFIX}/f16_model)
install(FILES ${CMAKE_SOURCE_DIR}/info.toml DESTINATION ${CMAKE_INSTALL_PREFIX}/f16_model)
install(DIRECTORY ${CMAKE_SOURCE_DIR}/data/ DESTINATION ${CMAKE_INSTALL_PREFIX}/f16_model/data/)

set(F16_MODEL_DIR ${CMAKE_INSTALL_PREFIX}/f16_model)

# 测试目录
# enable_testing()
# add_subdirectory(tests)
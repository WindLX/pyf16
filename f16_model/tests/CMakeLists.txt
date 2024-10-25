function(add_test_target target_name target_source_files)
    add_executable(${target_name} ${target_source_files} "test_utils.c")

    get_target_property(target_binary_dir ${target_name} BINARY_DIR)

    add_custom_command(
        TARGET ${target_name}
        PRE_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy ${f16_library} ${target_binary_dir}
        COMMENT "Copying files from ${F16_MODEL_DIR} to ${target_binary_dir}"
    )

    target_include_directories(${target_name}
        PRIVATE ${CMAKE_SOURCE_DIR}/include
        PRIVATE ${CMAKE_SOURCE_DIR}/tests/include
    )
    
    target_link_libraries(${target_name} f16_model)

    add_test(${target_name} ${target_name})
endfunction()

enable_testing()

file(GLOB f16_library "${F16_MODEL_DIR}/*")

add_test_target(test_hifi_loader "test_hifi_loader.c")
add_test_target(test_model "test_model.c")

target_compile_definitions(${PROJECT_NAME} PRIVATE ENABLE_PROTECTIONS)
target_compile_definitions(${PROJECT_NAME} PRIVATE ENABLE_CHECKSUM)
target_compile_definitions(${PROJECT_NAME} PRIVATE ENABLE_SAFEWRITE)
target_compile_definitions(${PROJECT_NAME} PRIVATE ENABLE_WATCHDOG=20)

# Use PB0 for DFU-BOOT, pull-up, active-low
target_compile_definitions(${PROJECT_NAME} PRIVATE
    ENABLE_GPIO_DFU_BOOT
    GPIO_DFU_BOOT_PORT=1
    GPIO_DFU_BOOT_PIN=0
    GPIO_DFU_BOOT_PULL=1
    GPIO_DFU_BOOT_VAL=0
)

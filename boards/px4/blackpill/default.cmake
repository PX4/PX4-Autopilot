px4_add_board(
    PLATFORM nuttx
    VENDOR px4
    MODEL blackpill
    LABEL default
    TOOLCHAIN arm-none-eabi
    ARCHITECTURE stm32f411ceu6
    
    CONSTRAINED_FLASH
    
    DRIVERS
        barometer
        imu
        gps
        optical_flow/px4flow
        
    MODULES
        apps
        commander
        ekf2
        mavlink
        mc_att_control
        mc_pos_control
        navigator 
        
    SYSTEM_COMMANDS
        param
        perf
        reboot
        top
    )

px4_add_board(
    PLATFORM nuttx
    VENDOR px4
    MODEL blackpill
    LABEL default
    TOOLCHAIN arm-none-eabi
    ARCHITECTURE stm32f411ceu6
    
    # Жесткая экономия Flash памяти под 512 КБ
    CONSTRAINED_FLASH
    
    DRIVERS
        # Оставляем только драйвер вашего датчика MPU6050
        imu/mpu6050
        # Драйвер для приема данных оптического потока от OpenMV
        optical_flow/px4flow
        
    MODULES
        apps
        commander
        # Используем ekf2. Если сборка будет ругаться на нехватку RAM,
        # в будущем заменим на более легкий local_position_estimator
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

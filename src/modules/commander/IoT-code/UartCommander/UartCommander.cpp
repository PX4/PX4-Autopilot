

/* commander module headers */




/* PX4 headers */



#include <systemlib/mavlink_log.h>

#include <math.h>
#include <float.h>
#include <cstring>
#include <matrix/math.hpp>

#include <termios.h>

#include <commander/IoT-code/UartCommander/UartCommander.hpp>

#include <commander/IoT-code/Serial_port/serial_port.h>
// uart_commander.cpp


UartCommander::UartCommander() {
    // Constructor implementation
}

UartCommander::~UartCommander() {
    // Destructor implementation
}
int UartCommander::Uart_Rxtest(int expectedn){
	
   // Create an instance of the Serial_Port class
    Serial_Port serial_port("/dev/ttyS1", 115200); // Adjust port and baud rate as necessary
        
    
        // Open the serial port
        serial_port.start();
        uint8_t* data1 = new uint8_t[6];
        uint8_t* data2 = new uint8_t[6];
        
        int bytes_read1 = 0;
        int bytes_read2 = 0;
        bool split = false;

        while (true) {
            // Define a variable to store the received byte
            
            
            // Read from the serial port
            bytes_read1 = serial_port._read_port(*data1,20);
            if(bytes_read1+bytes_read2 < 6){
                bytes_read2 = bytes_read1;
                data2 = data1;
                split = true;
                continue;   
            }
            
            // Check if reading was successful
            if (bytes_read1 > 0) {
                // Process the received byte (optional)
                printf("Bytes recieved %d\n",bytes_read1+bytes_read2);

                if (split){
                    printf("Data: ");
                    for (int i = 0; i < bytes_read1; i++) {
                        printf("%u ", data1[i]); // Print each byte value
                    }

                    for (int i = 0; i < bytes_read2; i++) {
                        printf("%u ", data2[i]); // Print each byte value
                    }
                }else{
                    printf("Data: ");
                    for (int i = 0; i < bytes_read2; i++) {
                        printf("%u ", data2[i]); // Print each byte value
                    }
                    for (int i = 0; i < bytes_read1; i++) {
                        printf("%u ", data1[i]); // Print each byte value
                    }
                } 
                bytes_read1 = 0;
                bytes_read2 = 0;
                printf("\n ");

            
            }
        }
	serial_port.stop();
    

    return EXIT_SUCCESS;
}
int UartCommander::Uart_Txtest(){
	
   // Create an instance of the Serial_Port class
    Serial_Port serial_port("/dev/ttyS1", 115200); // Adjust port and baud rate as necessary

    
        // Open the serial port
        serial_port.start();
        char *msg = new char[6];
        std::strcpy(msg, "hello");

        
    

        while (true) {
            // Define a variable to store the received byte
            
            
            // Read from the serial port
            serial_port._write_port(msg,6);
            
            sleep(1);
            
            
            // Check if reading was successful
           

            
            
        }
	serial_port.stop();
    

    return EXIT_SUCCESS;
}
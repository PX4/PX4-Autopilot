#!/usr/bin/env python2

PKG = 'px4'

from pymavlink import mavutil

def main():
    # Start a connection listening to a UDP port
    the_connection = mavutil.mavlink_connection('udpin:localhost:14541')

    # Wait for the first heartbeat
    the_connection.wait_heartbeat()
    print("Heartbeat from system (system %u component %u)" % (the_connection.target_system, the_connection.target_system))
    collision = False

    while True:
        msg = the_connection.recv_match(type='HEARTBEAT',blocking=False)
        base_mode = the_connection.messages['HEARTBEAT'].base_mode
        custom_mode = the_connection.messages['HEARTBEAT'].custom_mode


        if (custom_mode == 67371008 and base_mode == 157):
            # Mission Flight Mode
            msg = the_connection.recv_match(type='COLLISION',blocking=False)
            if msg:
                print '\033[91m' + "!!!!!!!!!!!Error: The vehicle has crashed into an obstacle!!!!!!!!!!" + '\033[0m'
                collision = True



if __name__ == '__main__':
    main()

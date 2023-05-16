#!/usr/bin/env python3

"""
Converts IP address given in decimal dot notation to int32 to be used in UXRCE_DDS_CFG parameter
and vice-versa

@author: Beniamino Pozzan (beniamino.pozzan@phd.unipd.it)
"""


import argparse

parser = argparse.ArgumentParser(
    prog = 'convert_ip',
    description = 'converts IP to int32 and viceversa'
    )
parser.add_argument('input',
                    type=str,
                    help='IP address to convert')
parser.add_argument('-r','--reverse',
                    action='store_true',
                    help='converts from int32 to dot decimal')

args = parser.parse_args()

if( args.reverse == False ):

    try:
        ip_parts = [int(x) for x in args.input.split('.')]
    except:
        raise ValueError("Not a valid IP")
    if( len(ip_parts)!=4 ):
        raise ValueError("Not a valid IP")
    if( not all(x>=0 and x<255 for x in ip_parts) ):
        raise ValueError("Not a valid IP")

    ip = (ip_parts[0]<<24) + (ip_parts[1]<<16) + (ip_parts[2]<<8) + ip_parts[3]

    if(ip & 0x80000000):
        ip = -0x100000000 + ip

    print(ip)

else:
    try:
        ip = int(args.input)
    except:
        raise ValueError("Not a valid IP")
    if(ip < 0):
        ip = ip + 0x100000000
    print('{}.{}.{}.{}'.format(ip>>24, (ip>>16)&0xff, (ip>>8)&0xff, ip&0xff))

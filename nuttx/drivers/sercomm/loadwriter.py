#!/usr/bin/python
from socket import *
import time

SOCKET_NAME = '/tmp/osmocom_loader'

s = socket(AF_UNIX, SOCK_STREAM)
s.connect(SOCKET_NAME)

while 1:
  try:
    x = raw_input(">")
    y = len(x) + 1
    s.send(chr(y>>8) + chr(y&255) + x + "\n")
  except:
    print ''
    break

s.close()

import serial, time


port = serial.Serial('/dev/ttyACM0', baudrate=57600, timeout=2)

data = '01234567890123456789012345678901234567890123456789'
#data = 'hellohello'
outLine = 'echo %s\n' % data

port.write('\n\n\n')
port.write('free\n')
line = port.readline(80)
while line != '':
    print(line)
    line = port.readline(80)
    

i = 0
bytesOut = 0
bytesIn = 0

startTime = time.time()
lastPrint = startTime
while True:
    bytesOut += port.write(outLine)
    line = port.readline(80)
    bytesIn += len(line)
    # check command line echo
    if (data not in line):
        print('command error %d: %s' % (i,line))
        #break
    # read echo output
    line = port.readline(80)
    if (data not in line):
        print('echo output error %d: %s' % (i,line))
        #break
    bytesIn += len(line)
    #print('%d: %s' % (i,line))
    #print('%d: bytesOut: %d, bytesIn: %d' % (i, bytesOut, bytesIn))
    
    elapsedT = time.time() - lastPrint
    if (time.time() - lastPrint >= 5):
        outRate = bytesOut / elapsedT
        inRate = bytesIn / elapsedT
        usbRate = (bytesOut + bytesIn) / elapsedT
        lastPrint = time.time()
        print('elapsed time: %f' % (time.time() - startTime))
        print('data rates (bytes/sec): out: %f, in: %f, total: %f' % (outRate, inRate, usbRate))
        
        bytesOut = 0
        bytesIn = 0
        
    i += 1    
    #if (i > 2): break
    

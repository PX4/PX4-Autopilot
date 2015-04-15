import sys
print 'Arguments: file - ' + sys.argv[1] + ', length - ' + sys.argv[2]
file = open(sys.argv[1], 'w')
for i in range(int(sys.argv[2])):
	b = bytearray([i & 0xFF])
	file.write(b)
file.close()
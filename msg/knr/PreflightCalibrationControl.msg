# PreflightCalibration Control message
uint64 timestamp

uint8 ACTION_STOP = 0
uint8 ACTION_START = 1
uint8 ACTION_CAPTURE_FRONT = 2
uint8 ACTION_CAPTURE_UP = 3
uint8 ACTION_READ_FRONT = 4
uint8 ACTION_READ_UP = 5

# Stare Aliasy, nie chce mi sie poprawiac kodu bo juz napisalem, naprawie jak bede mial wiecej czasu po pracy
uint8 ACTION_SAVE = 2
uint8 ACTION_LOAD = 3

uint8 action # one of ACTION_*

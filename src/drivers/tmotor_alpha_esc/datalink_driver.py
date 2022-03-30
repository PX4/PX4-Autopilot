#!/home/eduard/.pyenv/versions/bc3100/bin/python
import serial
from time import sleep
from ctypes import c_int16


ser = serial.Serial ("/dev/ttyUSB0", 115200)    #Open port with baud rate
i=1
telem_header = int.from_bytes(b'\x9b', 'big')

n_poles = 28
pole_pairs = n_poles/2

temp_table = {
    241:0, 	    240:1, 	    239:2, 	    238:3, 	    237:4, 	    236:5, 	    235:6, 	    234:7, 	    233:8, 	    232:9,
    231:10, 	230:11, 	229:12, 	228:13, 	227:14, 	226:15, 	224:16, 	223:17, 	222:18, 	220:19,
    219:20, 	217:21, 	216:22, 	214:23, 	213:24, 	211:25, 	209:26, 	208:27, 	206:28, 	204:29,
    202:30, 	201:31, 	199:32, 	197:33, 	195:34, 	193:35, 	191:36, 	189:37, 	187:38, 	185:39,
    183:40, 	181:41, 	179:42, 	177:43, 	174:44, 	172:45, 	170:46, 	168:47, 	166:48, 	164:49,
    161:50, 	159:51, 	157:52, 	154:53, 	152:54, 	150:55, 	148:56, 	146:57, 	143:58, 	141:59,
    139:60, 	136:61, 	134:62, 	132:63, 	130:64, 	128:65, 	125:66, 	123:67, 	121:68, 	119:69,
    117:70, 	115:71, 	113:72, 	111:73, 	109:74, 	106:75, 	105:76, 	103:77, 	101:78, 	99:79,
    97:80, 	    95:81, 	    93:82, 	    91:83, 	    90:84, 	    88:85, 	    85:86, 	    84:87, 	    82:88, 	    81:89,
    79:90, 	    77:91, 	    76:92, 	    74:93, 	    73:94, 	    72:95, 	    69:96, 	    68:97, 	    66:98, 	    65:99,
    64:100, 	62:101, 	62:102, 	61:103, 	59:104, 	58:105, 	56:106, 	54:107, 	54:108, 	53:109,
    51:110, 	51:111, 	50:112, 	48:113, 	48:114, 	46:115, 	46:116, 	44:117, 	43:118, 	43:119,
    41:120, 	41:121, 	39:122, 	39:123, 	39:124, 	37:125, 	37:126, 	35:127, 	35:128, 	33:129,
}

def get_temp(ix):
    try:
        keys = temp_table.keys()
        if ix in keys:
            return temp_table[ix]
        x1 = min([val for val in keys if val > ix])
        x0 = max([val for val in keys if val < ix])
        y1 = temp_table[x1]
        y0 = temp_table[x0]
        yix = y0+ ((y1-y0)/(x1-x0)) * (ix-x0)
        return yix
    except:
        return -1
    


class EscTelemetryMessage:
    def __init__(self, data) -> None:

        self.motor_ix = data[0]
        self.channel_bag_number = int.from_bytes(data[1:3], 'big')
        self.rx_throttle = int.from_bytes(data[3:5], 'big')*(100/1024)
        self.actual_throttle = int.from_bytes(data[5:7], 'big')*(100/1024)
        self.electric_rpm = int(int.from_bytes(data[7:9], 'big')*(10/pole_pairs))
        self.bus_voltage = int.from_bytes(data[9:11], 'big')/10
        self.bus_current = round(c_int16(int.from_bytes(data[11:13], 'big')).value/64, 1)
        self.phase_current = round(c_int16(int.from_bytes(data[13:15], 'big')).value/64, 1)
        self.mos_temperature = get_temp(data[15])
        self.cap_temperature = get_temp(data[16])
        self.status_byte = data[17:]
        # status_bits = "{:08b}".format(int(status_byte.hex(),16))


    

class TMotorDataLinkMessage:
    def __init__(self, message_bytes) -> None:
        # basic checking if it's no good, crash.
        assert len(message_bytes)==160
        assert message_bytes[0]==155  # aka 0x9b'
        assert message_bytes[1]==158

        self.esc_messages = [None]*8

        for start_index in range(6,157, 19):
            esc_message = EscTelemetryMessage(message_bytes[start_index:start_index+18])
            self.esc_messages[esc_message.motor_ix-1] = esc_message
        
        crc_sum = sum(message_bytes[7:158])
        crc = int.from_bytes(message_bytes[158:], 'big')

        print(f'{crc_sum=} {crc=}') # this is clearly still wrong....



if __name__=="__main__":

    while True:
        received_data = ser.read()              #read serial port
        sleep(0.05)
        data_left = ser.in_waiting             #check for number of remaining bytes
        received_data += ser.read(data_left)

        try:
            dl_message = TMotorDataLinkMessage(received_data)
        except Exception as e:
            print(e)
            continue

        for esc_message in dl_message.esc_messages[:4]:
            print(esc_message.__dict__)
            

    

     

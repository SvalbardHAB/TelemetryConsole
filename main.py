# This Python file uses the following encoding: utf-8
import serial,sys,time,os

#const_default_serial_port = "/dev/tty.usbserial-A6009CSG"
#const_default_serial_port = "/dev/tty.usbserial-A6009CSG"
const_default_serial_port = "/dev/cu.usbmodem1556311"
const_default_serial_baud = 115200
const_data_bad_time = 3.0
const_serial_timeout = 0.01
const_lng_mult = 1

EFM_boards = [0]
cloud_boards = [0,1]
hygro_boards = [0]

def nameLogFile(base):
  file_counter = 0
  file_name = "data/"+base + str(file_counter) + ".CSV"
  while os.path.isfile(file_name):
    file_counter += 1
    file_name =  "data/"+base + str(file_counter) + ".CSV" ##make sure we don't overwrite anything.
  fx = open(file_name,'w')
  fx.close()
  return file_name

def open_serial_port(port_name):#tries to open a specified serial port
    serial_port.port = port_name
    try:
        print("opening",serial_port.port)
        serial_port.timeout = const_serial_timeout #set a timeout for the readline command
        serial_port.open()
        print("Successfully opened serial port.")
        return True
    except (OSError) as e:#if we get a SerialException, the port is
                                                  #probably open already.
        print("Bad serial port name. Try again.")
        print(e)
        return False

def check_serial():
    try:
        return serial_port.readline()
    except:
        return ""

def printData():
    #begin by clearing console
    os.system('cls' if os.name == 'nt' else 'clear')
    print("PACKET FROM BALLOON {0}. ".format(latest[3]) + ". CRC " + "fail!" if latest[0]==0 else "pass")
    #print("RX: "+rx)
    print("RSSI:{0}, SNR:{1}".format(latest[1],latest[2]))
    print("Altitude: {0} m".format(latest[7]))
    print("P: {0} hPa  |  T: {1} °C  |  Lat,long: {2}, {3}".format(latest[6],latest[8],latest[9],latest[10]))
    print("Photodiode counts: {0} | Average level: {1}".format(latest[11],latest[12]))
    if latest[3] in EFM_boards:
        print("EFM min: {0}  |  EFM max: {1}".format(latest[13],latest[14]))
    if latest[3] in cloud_boards:
        print("Cloud average: {0}/1024  |  SD: {1}".format(latest[15],latest[16]))
    if latest[3] in hygro_boards:
        print("Relative humidity: {0}%  |  Outside temp: {1} °C".format(latest[17],latest[18]))
    print("GPS {0}; has {1}got more than or three satellites; hdop {2} less than 20m.".format("valid" if last_status[3]  else "invalid","" if last_status[2]  else "not ", "is" if last_status[1]  else "is not"))
    print("SD init {0}successful. Promiscuous mode {1}".format("" if last_status[0]  else "un", "on." if last_status[4] else "off."))
    print("==========================")
    print("Waiting...")

def processdata(received):
    global latest,QNH
    rawlog = open(logfile_names[0],'a')
    rawlog.write(received)
    rawlog.write("\n")
    rawlog.close()
    proclog = 0 #open(logfile_names[1],'a')
    split = received.split(",")
    try:
        latest[0]=int(split[0],16) #CRC pass/fail
        latest[1] = int(split[1],16) #RSSI
        latest[2] = int(split[2],16) #SNR
        latest[3] = int(split[3],16) #device ID
        proclog = open(logfile_names[latest[3]+1],'a')
        latest[4] = int(split[4],16) #status byte
        latest[5] = int(split[5],16) #battery status—not yet implemented 2/9/16
        latest[6] = (int(split[6],16)<<24 | int(split[7],16)<<16 | int(split[8],16)<<8 | int(split[9],16))/100.0 #Pressure
        #calc altitude here...
        ht = (1-pow((latest[6]/QNH),0.190284))*145366.45 #ht in feet
        latest[7] = ht*0.3048 #convert to m


        latest[8] = int(split[10],16)-128 #temperature in excess 128 format

        latest[9] = ((int(split[11],16)<<24)|(int(split[12],16)<<16)|(int(split[13],16)<<8)|(int(split[14],16)))/10000000.0 #lat
        latest[10] = ((int(split[15],16)<<24)|(int(split[16],16)<<16)|(int(split[17],16)<<8)|(int(split[18],16)))/10000000.0 #long
        latest[10]*=const_lng_mult

        latest[11] = int(split[19],16)#photodiode count
        latest[12] = int(split[20],16)#photodiode avg level
        if latest[3] in EFM_boards:
            latest[13] = int(split[21],16) #EFM byte 1
            latest[14] = int(split[22],16) #EFM byte 2
        else:
            latest[13] = 0
            latest[14] = 0
        if latest[3] in cloud_boards:
            latest[15] = (int(split[23],16)<<8 | int(split[24],16))    #cloud detector avg
            latest[16] = int(split[25],16) #cloud detector standard deviation
        else:
            latest[15] = 0
            latest[16] = 0
        if latest[3] in hygro_boards:
            latest[17] = int(split[26],16) #relative humidity
            latest[18] = int(split[27],16) #external temperature
        else:
            latest[17] = 0
            latest[18] = 0


        last_status[4] = True if (int(split[4],16)&0x1) == 1 else False #promiscuous mode
        last_status[3] = True if (int(split[4],16)&0x2)>>1 == 1 else False #GPS valid
        last_status[2] = True if (int(split[4],16)&0x4)>>2 == 1 else False #GPS has >3 satellites
        last_status[1] = True if (int(split[4],16)&0x8)>>3 == 1 else False
        last_status[0] = True if (int(split[4],16)&0x10)>>4 == 1 else False #SD init ok

    except IndexError:
        print "Bad message."
        return

    for (n,i) in enumerate(latest):
        proclog.write(str(i))
        if n!=len(latest):
            proclog.write(",")
        else:
            proclog.write(",")
            proclog.write(time.time())
            proclog.write(",#")
            proclog.write("\n")
    proclog.close()
    printData()
    print(received)


serial_port = serial.Serial() #instantiate a new Serial port object
serial_port.baudrate = const_default_serial_baud
if not open_serial_port(const_default_serial_port):
    print("bad serial port.")
    sys.exit(0)
logfile_names = [nameLogFile("GND_RAW_"),nameLogFile("GND_PROCESSED_0_"),nameLogFile("GND_PROCESSED_1_"),nameLogFile("GND_PROCESSED_2")]
#to clear serial console os.system('cls' if os.name == 'nt' else 'clear')
QNH = input('Please input QNH in hPa')
latest = [0,   0,    0,   0,  0,      0,    0.0,   0.0,      0,           0.0,  0.0, 0,        0,      0,   0,   0,          0,           0,  0]
         #CRC, RSSI, SNR, ID, status, batt, press, altitude, temperature, lat, long, pd_count, pd_avg, EFM, EFM, Cloud mean, cloud stdev, rh, ext temp

last_status = [False,False,          False,     False,      False]
           #SD init, GPS HDOP <=20m, GPS>=3sat, GPS valid, promiscuous mode

pktcount = 0

while True:
    time.sleep(0.1)
    #rec = "#-100,13,0,1,82,4,95,0,5,0,2,0,7,A1,20,0,0,0,0,9"
    rec = check_serial()
    if rec!="" and rec[0]=="#":#if we've got something
        pktcount+=1
        rec = rec[1:]
        processdata(rec)
    elif rec!="":
        rawlog = open(logfile_names[0],'a')
        rawlog.write(rec)
        rawlog.write("\n")
        rawlog.close()

# potentiometer_plot.py
import datetime
import matplotlib.pyplot as plt
import serial
import numpy as np


# make sure the 'COM#' is set according the Windows Device Manager
ser = serial.Serial(port='/dev/tty.usbmodem14301', baudrate=115200, timeout=1)

roll = [] # 0
pitch = [] # 1 
yaw = [] # 2

rollProportional = [] # 3
rollIntegral = [] # 4
rollDerivative = [] # 5 
rollCorrection = [] # 6

pitchProportional = [] # 3
pitchIntegral = [] # 4
pitchDerivative = [] # 5 
pitchCorrection = [] # 6

throttle = [] # 7 
frontRight = [] # 8 
frontLeft = [] # 9
backRight = []
backLeft = []

graphRoll = True;
graphPitch = False; 


# skip first 5 
# break loop if abort == true

for i in range(405):
    line = ser.readline()   # read a byte string
    
    if line:
        string = line.decode()  # convert the byte string to a unicode string
        strList = string.split(" ")
        abort = int(strList[0])
        print(abort)
        
        if (abort == 1):
            break

        throt = int(strList[12])
        
        if (abort == 0 and throt > 220 ):
            graph = True; 
            print(strList)
            roll.append(strList[1])
            pitch.append(strList[2])
            yaw.append(strList[3])

            rollProportional.append(strList[4])
            rollIntegral.append(strList[5])
            rollDerivative.append(strList[6])
            rollCorrection.append(strList[7])

            pitchProportional.append(strList[8])
            pitchIntegral.append(strList[9])
            pitchDerivative.append(strList[10])
            pitchCorrection.append(strList[11])

            throttle.append(strList[12])
            frontRight.append(strList[13])
            frontLeft.append(strList[14])
            backRight.append(strList[15])
            backLeft.append(strList[16])


ser.close()



roll = [float(x) for x in roll]
pitch = [float(x) for x in pitch]
yaw = [float(x) for x in yaw]

rollProportional = [float(x) for x in rollProportional]
rollIntegral = [float(x) for x in rollIntegral]
rollDerivative = [float(x) for x in rollDerivative]
rollCorrection = [float(x) for x in rollCorrection]

pitchProportional = [float(x) for x in pitchProportional]
pitchIntegral = [float(x) for x in pitchIntegral]
pitchDerivative = [float(x) for x in pitchDerivative]
pitchCorrection = [float(x) for x in pitchCorrection]

throttle = [float(x) for x in throttle]
frontLeft = [float(x) for x in frontLeft]
frontRight = [float(x) for x in frontRight]
backLeft = [float(x) for x in backLeft]
backRight = [float(x) for x in backRight]

if(True):
    # USE SEABORN STYLE 
    plt.style.use('seaborn')

    fig, (ax1, ax2) = plt.subplots(2, 1)

    ax1.plot(roll) 
    ax1.axhline(0, color='black', ls='-')

    ax1.set_title('Roll vs Time')
    ax1.set_ylabel('roll')
    ax1.grid(True)

    ax2.plot(frontLeft,label='FrontLeft')
    ax2.plot(frontRight,label='FrontRight')
    ax2.plot(backLeft,label='BackLeft')
    ax2.plot(backRight,label='backRight')

    ax2.set_title('FrontLeft/FrontRight vs Time')
    ax2.set_ylabel('frontLeft and frontRight')
    ax2.grid(True)
    ax2.legend()

    plt.tight_layout()

    # get current time from datetime library
    now = datetime.datetime.now().time()
    timeStr = now.strftime("%H:%M:%S")

    name = "graphs/" + timeStr + ".pdf"

    plt.savefig(name, format="pdf")
    plt.close

    ############################## roll

if (graphRoll == True):     

    plt.style.use('seaborn')

    fig, (ax1, ax2) = plt.subplots(2, 1)

    ax1.plot(roll) # , t, rollCorrection
    ax1.axhline(0,color='black',ls='-')

    ax1.set_title('Roll vs Time')
    ax1.set_ylabel('roll')
    ax1.grid(True)

    ax2.plot(rollProportional, label='proportional')
    ax2.plot(rollIntegral, label='integral')
    ax2.plot(rollDerivative, label='derivative')
    ax2.plot(rollCorrection, label='roll correction', ls='--')
    ax2.axhline(0,color='black',ls='-')

    ax2.set_title('PID vs Time')
    ax2.set_ylabel('PID')
    ax2.grid(True)
    ax2.legend()

    # get current time from datetime library
    now = datetime.datetime.now().time()
    timeStr = now.strftime("%H:%M:%S")

    name = "graphs/rollPID" + timeStr + ".pdf"

    plt.tight_layout()
    plt.savefig(name, format="pdf")
    plt.close


    ########################## pitch
if (graphPitch == True):
    plt.style.use('seaborn')

    fig, (ax1, ax2) = plt.subplots(2, 1)

    ax1.plot(pitch) # , t, rollCorrection
    ax1.axhline(0,color='black',ls='-')

    ax1.set_title('Pitch vs Time')
    ax1.set_ylabel('pitch')
    ax1.grid(True)

    ax2.plot(pitchProportional, label='proportional')
    ax2.plot(pitchIntegral, label='integral')
    ax2.plot(pitchDerivative, label='derivative')
    ax2.plot(pitchCorrection, label='pitch correction', ls='--')
    ax2.axhline(0,color='black',ls='-')

    ax2.set_title('PID vs Time')
    ax2.set_ylabel('PID')
    ax2.grid(True)
    ax2.legend()

    # get current time from datetime library
    now = datetime.datetime.now().time()
    timeStr = now.strftime("%H:%M:%S")

    name = "graphs/pitchPID" + timeStr + ".pdf"

    plt.tight_layout()
    plt.savefig(name, format="pdf")
    plt.close



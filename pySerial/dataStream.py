# potentiometer_plot.py
import datetime
import matplotlib.pyplot as plt
import serial
import numpy as np


# make sure the 'COM#' is set according the Windows Device Manager
ser = serial.Serial(port='/dev/tty.usbmodem14301', baudrate=115200, timeout=1)

roll = [] # 0
pitch = [] # 1 
gyroYaw = [] # 2
proportional = [] # 3
integral = [] # 4
derivative = [] # 5 
rollCorrection = [] # 6
throttle = [] # 7 
frontRight = [] # 8 
frontLeft = [] # 9
backRight = []
backLeft = []

graph = False;


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

        throt = int(strList[8])
        
        if (abort == 0 and throt > 265 ):
            graph = True; 
            print(strList)
            roll.append(strList[1])
            pitch.append(strList[2])
            gyroYaw.append(strList[3])
            proportional.append(strList[4])
            integral.append(strList[5])
            derivative.append(strList[6])
            rollCorrection.append(strList[7])
            throttle.append(strList[8])
            frontRight.append(strList[9])
            frontLeft.append(strList[10])
            backRight.append(strList[11])
            backLeft.append(strList[12])
            if (throt > 315):
                print("THROTTLE AT 315 - WEIGHTLESSNESS ACHIEVED")

ser.close()



roll = [float(x) for x in roll]
proportional = [float(x) for x in proportional]
integral = [float(x) for x in integral]
derivative = [float(x) for x in derivative]
rollCorrection = [float(x) for x in rollCorrection]
throttle = [float(x) for x in throttle]
frontLeft = [float(x) for x in frontLeft]
frontRight = [float(x) for x in frontRight]
backLeft = [float(x) for x in backLeft]
backRight = [float(x) for x in backRight]

if(graph == True):
    # USE SEABORN STYLE 
    plt.style.use('seaborn')

    fig, (ax1, ax2) = plt.subplots(2, 1)

    ax1.plot(roll) # , t, rollCorrection
    ax1.axhline(0,color='black',ls='-')

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

    plt.style.use('seaborn')

    fig, (ax1, ax2) = plt.subplots(2, 1)

    ax1.plot(roll) # , t, rollCorrection
    ax1.axhline(0,color='black',ls='-')

    ax1.set_title('Roll vs Time')
    ax1.set_ylabel('roll')
    ax1.grid(True)

    ax2.plot(proportional, label='proportional')
    ax2.plot(integral, label='integral')
    ax2.plot(derivative, label='derivative')
    ax2.plot(rollCorrection, label='roll correction', ls='--')
    ax2.axhline(0,color='black',ls='-')

    ax2.set_title('PID vs Time')
    ax2.set_ylabel('PID')
    ax2.grid(True)
    ax2.legend()

    # get current time from datetime library
    now = datetime.datetime.now().time()
    timeStr = now.strftime("%H:%M:%S")

    name = "graphs/PID" + timeStr + ".pdf"

    plt.tight_layout()
    plt.savefig(name, format="pdf")
    plt.close



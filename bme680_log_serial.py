#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sun Oct  4 12:26:02 2020

@author: bernardo
https://makersportal.com/blog/2018/2/25/python-datalogger-reading-the-serial-output-from-arduino-to-analyze-data-using-pyserial

"""
import serial
import numpy as np
from matplotlib import pyplot as plt
import time
import csv

 #If you're not using Linux, you'll need to change this
#check the Arduino IDE to see what serial port it's attached to
#ser = serial.Serial('/dev/ttyACM0', 115200)
# For a MAC, eg.
ser = serial.Serial('/dev/cu.SLAB_USBtoUART', 115200)


while True:
    try:
        ser.reset_input_buffer()
        ser_bytes= ser.readline()
        # convert byte to string 
        data = ser_bytes[0:len(ser_bytes)-2].decode("utf-8")
        dataList = data.split(',')
        msgType = int(dataList[0])
#        data=np.array(data.split(','))
#       append PC time stamp       
        line_sv= str(int(time.time())) + ', ' + data
        print(line_sv)
        if(msgType == 1):
            """            
                            output += ", " + String(iaqSensor.rawTemperature);
                output += ", " + String(iaqSensor.pressure);
                output += ", " + String(iaqSensor.rawHumidity);
                output += ", " + String(iaqSensor.gasResistance);
                output += ", " + String(iaqSensor.iaq);
                output += ", " + String(iaqSensor.iaqAccuracy);
                output += ", " + String(iaqSensor.temperature);
                output += ", " + String(iaqSensor.    );
                output += ", " + String(iaqSensor.staticIaq);
                output += ", " + String(iaqSensor.co2Equivalent);
                output += ", " + String(iaqSensor.breathVocEquivalent);
            """
            uCtime =int(dataList[1])
            pressure =float(dataList[3])
            gasResistance = float(dataList[5])
            iaq =float(dataList[6])
            iaqAccuracy =int(dataList[7])
            temperature =float(dataList[8])
            humidity =float(dataList[9])
            co2Equivalent =float(dataList[11])
            breathVocEquivalent =float(dataList[12])
            #print(time.time() + ',' + line)
            with open("bme680_data.csv","a") as f:
                #f.write(line_sv + '\r\n')
                writer = csv.writer(f,delimiter=",")
                writer.writerow([int(time.time()),uCtime,pressure,gasResistance,iaq,iaqAccuracy,temperature,humidity,co2Equivalent,breathVocEquivalent])
    except (KeyboardInterrupt, SystemExit):
        print("Keyboard Interrupt")
        break
    except:
        pass
    
ser.close()  
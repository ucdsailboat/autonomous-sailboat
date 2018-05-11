import serial
import time
import csv

ser = serial.Serial('COM4',9600)         #Update serial port
ser.flushInput()

while True:
    try:
        #Reads and formats data from a single serial line into separate array elements
        ser_bytes = ser.readline()                                      
        decoded_bytes=ser_bytes[0:len(ser_bytes)-2].decode("utf-8")     
        decoded_bytes=decoded_bytes.split(',')                          
        
        #List of array elements expected in each line
        latitude=float(decoded_bytes[0])
        longitude=float(decoded_bytes[1])
        windSpeed=float(decoded_bytes[2])
        windDirection=flaot(decoded_bytes[3])
        
        #Writes to .csv file
        with open("boatTracker.csv","a",newline='') as f:
            writer = csv.writer(f,delimiter=",")
            writer.writerow([time.time(),latitude,longitude, windSpeed, windDirection])        
    except:
        print("Keyboard Interrupt")
        break

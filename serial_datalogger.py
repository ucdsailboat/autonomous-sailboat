import serial
import time
import csv

ser = serial.Serial('/dev/ttyACM0',9600)         #Update serial port
ser.flushInput()

while True:
    try:
        #Reads and formats data from a single serial line into separate array elements
        ser_bytes = ser.readline()                                      
        decoded_bytes=ser_bytes.decode("utf-8")     
        decoded_bytes=decoded_bytes.split(',')                          
        
        #List of array elements expected in each line
        windDirection=float(decoded_bytes[0])
        windSpeed=float(decoded_bytes[1])
        latitude=float(decoded_bytes[2])
        longitude=float(decoded_bytes[3])
        boatSpeed=float(decoded_bytes[4])

        
        #Writes to .csv file
        with open("boatTracker.csv","a",newline='') as f:
            writer = csv.writer(f,delimiter=",")
            writer.writerow([time.time(),windDirection,windSpeed,latitude,longitude, boatSpeed])        
    except:
        print("Keyboard Interrupt")
        break

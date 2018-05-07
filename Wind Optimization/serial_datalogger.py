import serial
import time
import csv

ser = serial.Serial('COM4',9600)
ser.flushInput()

while True:
    try:
        ser_bytes = ser.readline()
        decoded_bytes=ser_bytes[0:len(ser_bytes)-2].decode("utf-8")
        decoded_bytes=decoded_bytes.split(',')
        latitude=float(decoded_bytes[0])
        longitude=float(decoded_bytes[1])

        with open("boatTracker2.csv","a",newline='') as f:
            writer = csv.writer(f,delimiter=",")
            writer.writerow([time.time(),latitude,longitude])        
    except:
        print("Keyboard Interrupt")
        break

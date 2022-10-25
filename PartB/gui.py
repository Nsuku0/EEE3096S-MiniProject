import serial
import os
import csv

serialPort = serial.Serial(port = "COM3", baudrate=9600, bytesize=8, timeout=2, stopbits=serial.STOPBITS_ONE)

serialString = ""

menu = "LoT protocol Menu:\n\nEnter: \n 'X' to quit to program\n 'S' to begin LoT transmission\n  'A' Get and display ADC reading and add to log\n 'E' Echo Test\n"
input = ""
print(menu)

# Echo Test method to check that the serial comms is working
def echo():
    serialPort.write("Echo".encode())
    
    while(1):
        if(serialPort.in_waiting > 0):
            serialString = serialPort.readline()
            print(serialString.decode('Ascii'))
            break

# method to Do LoT Transmission
def transmit():
    serialPort.write("Transmit".encode())

    while(1):
        if(serialPort.in_waiting > 0):
            serialString = serialPort.readlines()

            for string in serialString:
                print(string.decode('Ascii'))
            break

# method to = Get and display ADC reading and add to log,
def log():
    print("ADC reading: ")
    try:
        file = open('Log.csv', 'a')
    except:
        file = open('Log.csv', 'x')

    writer = csv.writer(file, escapechar='\\')

    serialPort.write("ADC".encode())

    while(1):
        if(serialPort.in_waiting > 0):
            serialString = serialPort.readline()
            print(serialString.decode('Ascii'))
            break
        
    string = serialString.decode('Ascii')
    writer.writerow([string[-6:]])
    file.close()
    

while(1):
    input = input("Enter a command: ")

    match input:
        case "X":  #exit application
            break
        case "S": #S = Do LoT Transmission
            transmit()
        case "A":   # A = Get and display ADC reading and add to log
            log()
        case "E": # E = Echo Test, to check that the serial comms is working,
            echo()
 

import serial
import numpy as np
import matplotlib.pyplot as plt
import math
import kalman_filter as kal
import csv

import time as t

time = []
ROLL_deg_arr = []
road_taken_arr = []
i = 0

ser = serial.Serial(port='COM5', baudrate=9600, timeout=1)

myKalman = kal.KalmanFilter(0.3,0.01, 0.06)
start_time = t.time()


while True:
    try:
        if ser.in_waiting > 0:  
            data = ser.readline().decode('utf-8', errors='ignore').strip()
            if data:
                i += 1
                datas = data.split() 
                try:
                    roll = float(datas[1])
                    pitch = float(datas[3])
                    X_as_g = float(datas[5])
                    Y_as_g = float(datas[7])
                    Z_as_g = float(datas[9])
                    X = X_as_g * 9.81
                    Y = -Y_as_g * 9.81
                    Z = Z_as_g * 9.81
                    Y += math.sin(math.radians(roll)) * 9.81
                    #print(f"calculated: {-Y_as_g*9.81}, thing: {math.sin(math.radians(roll)) * 9.81}, sonuc: {Y}")
                    #print(f"deneme: {math.radians(roll)}")
                    #print(f"roll: {roll}, Y: {Y}")

                    road_taken = myKalman.update_filter(Y)
                    #print(road_taken)
                    
                    ROLL_deg_arr.append(roll)
                    road_taken_arr.append(road_taken)
                    current_time = t.time() - start_time
                    time.append(current_time)
                    #print(f"time is: {current_time}")

                except Exception as e:
                    print("Hata:", e)
    except KeyboardInterrupt:
        ser.close()
        with open("sensor_data.csv", mode="w", newline="") as file:
            writer = csv.writer(file)
            writer.writerow(["Time (s)", "Roll (deg)", "Road Taken"])
            for t, roll, road in zip(time, ROLL_deg_arr, road_taken_arr):
                writer.writerow([t, roll, road])

        print("Data saved to sensor_data.csv")

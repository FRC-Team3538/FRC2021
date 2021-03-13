#!/usr/bin/env python3
 
from pynput import keyboard
from serial import *
from tkinter import *
import time

serialPort = "COM5"
baudRate = 115200
ser = Serial(serialPort , baudRate, timeout=0, writeTimeout=0) #ensure non-blocking

#make a TkInter Window
root = Tk()
root.wm_title("RoboJackets FRC3538")

Label(root, width=40, text="FRC @ Home Timer").pack()

# Text box for Beam Status
BeamLatch = False
BeamStatustxt = StringVar()
BeamStatusLabel = Label(root, width=40, textvariable=BeamStatustxt)
BeamStatusLabel.pack()

# Text Box for Timer
TimerRunning = False
StartTime = time.time()
StopTime = time.time()
Timertxt = StringVar()
TimerLabel = Label(root, width=40, textvariable=Timertxt)
TimerLabel.pack()

# Enable Robot Hotkey
key1=False
key2=False
key3=False

def on_press(key):
    global key1, key2, key3
    global TimerRunning, StartTime

    try:
        if(key.char == '['):
            # print("Pressed  [")
            key1=True
        if(key.char == '\\'):
            # print("Pressed \\")
            key2=True
        if(key.char == ']'):
            # print("Pressed ]")
            key3=True
        
        if(key1 and key2 and key3):
            print("BOOM!")
            TimerRunning = True
            StartTime = time.time()

    except AttributeError:
        pass

def on_release(key):
    global key1
    global key2
    global key3

    try:
        if(key.char == '['):
            #print("Released  [")
            key1=False
        if(key.char == '\\'):
            #print("Released \\")
            key2=False
        if(key.char == ']'):
            #print("Released ]")
            key3=False
    except AttributeError:
        pass

# Monitor Serial port
def readSerial():
    global BeamLatch, TimerRunning, StopTime

    # Update Beam
    while ser.in_waiting:
        ln = ser.read()
        if len(ln) > 0:
            if(ln.decode() == '1' and BeamLatch == False):
                BeamLatch = True

                BeamStatustxt.set("BEAM BROKE")

                if TimerRunning:
                    TimerRunning = False
                    StopTime = time.time()

            if(ln.decode() == '0' and BeamLatch == True):
                BeamLatch = False

                BeamStatustxt.set("BEAM CLEAR")

    root.after(50, readSerial) # check serial again soon

    # Update Timer Display
    if(TimerRunning):
        Timertxt.set("%0.3f" % (time.time() - StartTime))
    else:
        Timertxt.set("%0.3f" % (StopTime - StartTime))


# Listen for keys non-blocking fashion:
listener = keyboard.Listener(
    on_press=on_press,
    on_release=on_release)
listener.start()

# after initializing serial, an arduino may need a bit of time to reset
root.after(100, readSerial)

root.mainloop()
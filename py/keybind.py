#!/usr/bin/env python3

from pynput import keyboard
import serial
import time

arduino = serial.Serial(port='/dev/ttyUSB0', baudrate=115200, timeout=.1)

key1=False
key2=False
key3=False

def on_press(key):
    global key1
    global key2
    global key3

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

# Collect events until released
with keyboard.Listener(
        on_press=on_press,
        on_release=on_release) as listener:
    listener.join()

# ...or, in a non-blocking fashion:
listener = keyboard.Listener(
    on_press=on_press,
    on_release=on_release)
listener.start()
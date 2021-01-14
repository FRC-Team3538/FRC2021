from rj.StatusFrameHolder import *
from rj.InitializeStatusFrame import *
import socket
import json
import time
import struct
import sys
import fire

DEFAULT_ROBOT_LOGGER_ADDRESS = ("10.83.32.2", 3538)
PLOTBUFFER_CONSUMER_ADDRESS = ("127.0.0.1", 9870)

def logger(robot_address=DEFAULT_ROBOT_LOGGER_ADDRESS, stream=False, save=False, stream_to=DEFAULT_PLOTBUFFER_CONSUMER_ADDRESS):
    if not (stream or save):
        print("Nothing to do! Exiting...")
        return

    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
        sock.settimeout(1.0)
        f = None
        try:
            sock.sendto(b"Hi", robot_address)
            data = sock.recv(4096)
            if stream:
                sock.sendto(data, stream_to)

            sfc = StatusFrameHolder.GetRootAsStatusFrameHolder(data, 4)
            if sfc.StatusFrameType() == 4:
                tab = sfc.StatusFrame()
                init = InitializeStatusFrame()
                init.Init(
                    tab.Bytes, tab.Pos
                )
            title = init.Title().decode('utf-8')[:-1].replace(' ', '.').replace(':', '')

            if save:
                f = open(f"logs/runtime.{title}.dat", "wb")
                print(f"Starting log: runtime.{title}.dat")
                f.write(data)
            else:
                f = None

            while True:
                data = sock.recv(4096)
                if stream:
                    sock.sendto(data, stream_to)
                if save:
                    f.write(data)

        except Exception as e:
            if f is not None:
                f.close()
            print(f"Received an Exception: {e}")
            raise e
    print("Exiting...")

if __name__ == "__main__":
    fire.Fire(logger)
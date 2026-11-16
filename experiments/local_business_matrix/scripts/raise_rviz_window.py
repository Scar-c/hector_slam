"""Raise only the RViz window opened with the requested configuration."""
import ctypes
import re
import subprocess
import sys

def raise_window(config):
    output=subprocess.run(['xwininfo','-root','-tree'],capture_output=True,text=True,check=True).stdout
    candidates=[line for line in output.splitlines() if config in line and '("rviz2" "rviz2")' in line]
    if not candidates:raise RuntimeError('RViz window not yet present')
    window=int(re.search(r'0x[0-9a-fA-F]+',candidates[0]).group(),16)
    lib=ctypes.CDLL('libX11.so.6')
    lib.XOpenDisplay.argtypes=[ctypes.c_char_p];lib.XOpenDisplay.restype=ctypes.c_void_p
    lib.XDefaultRootWindow.argtypes=[ctypes.c_void_p];lib.XDefaultRootWindow.restype=ctypes.c_ulong
    lib.XInternAtom.argtypes=[ctypes.c_void_p,ctypes.c_char_p,ctypes.c_int];lib.XInternAtom.restype=ctypes.c_ulong
    class Data(ctypes.Union):_fields_=[('l',ctypes.c_long*5),('b',ctypes.c_char*20)]
    class Client(ctypes.Structure):_fields_=[('type',ctypes.c_int),('serial',ctypes.c_ulong),('send_event',ctypes.c_int),('display',ctypes.c_void_p),('window',ctypes.c_ulong),('message_type',ctypes.c_ulong),('format',ctypes.c_int),('data',Data)]
    class Event(ctypes.Union):_fields_=[('client',Client),('pad',ctypes.c_long*24)]
    lib.XSendEvent.argtypes=[ctypes.c_void_p,ctypes.c_ulong,ctypes.c_int,ctypes.c_long,ctypes.POINTER(Event)]
    lib.XMapRaised.argtypes=[ctypes.c_void_p,ctypes.c_ulong];lib.XFlush.argtypes=[ctypes.c_void_p];lib.XCloseDisplay.argtypes=[ctypes.c_void_p]
    display=lib.XOpenDisplay(None)
    if not display:raise RuntimeError('Cannot open current desktop X display')
    event=Event();event.client.type=33;event.client.send_event=1;event.client.display=display;event.client.window=window
    event.client.message_type=lib.XInternAtom(display,b'_NET_ACTIVE_WINDOW',0);event.client.format=32;event.client.data.l[0]=2
    lib.XMapRaised(display,window);lib.XSendEvent(display,lib.XDefaultRootWindow(display),0,(1<<19)|(1<<20),ctypes.byref(event));lib.XFlush(display);lib.XCloseDisplay(display)
    return hex(window)
if __name__=='__main__':print(raise_window(sys.argv[1]))

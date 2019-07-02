import serial
from tkinter import *
import threading
from datetime import datetime

print('Connecting...')

ser = serial.Serial('COM14', 115200, timeout=1)

if ser.isOpen():
    print('Connected!')

loop_active = True
data_receive = True

temp = [255, 0, 0, 0, 0, 0, 0, 254]
pos = [255, 0, 0, 0, 0, 0, 0, 254]
factor = 1

win = Tk()
win.title("serial_test")
win.geometry("300x450")

lb0 = Label(win, text="Calibration Needed..")
lb1 = Label(win, text="0")
lb2 = Label(win, text="0")
lb3 = Label(win, text="0")
lb4 = Label(win, text="0")
lb5 = Label(win, text="0")
lb6 = Label(win, text="0")
lb7 = Label(win, text="0")
lb8 = Label(win, text="0")
lb9 = Label(win, text="0")
lb10 = Label(win, text="0")
lb11 = Label(win, text=factor)

vib_int = IntVar()
scale1 = Scale(win, variable=vib_int, from_=0, to=250, orient = HORIZONTAL)

def init_fsr():
    lb0.configure(text="Calibrating...")
    ser.write(bytearray([255,0,0,0,0,0,0,253]))       # start calibration

def ser_write():
    global temp, pos, factor
    for i in range(1, 6, 1):
        pos[i] = int(temp[i] / (250 * factor / 10) * 250)
        if pos[i] < 0:
            pos[i] = 0
        elif pos[i] > 250:
            pos[i] = 250
    lb6.configure(text=int(pos[1]))
    lb7.configure(text=int(pos[2]))
    lb8.configure(text=int(pos[3]))
    lb9.configure(text=int(pos[4]))
    lb10.configure(text=int(pos[5]))
    ser.write(bytearray(pos))

class Upd(threading.Thread):

    def __init__(self, tk_root):
        self.root = tk_root
        threading.Thread.__init__(self)
        self.start()

    def run(self):
        global loop_active, temp, vib_int
        while loop_active:
            if data_receive:
                x = ser.readline()
                if len(x) == 8:
                    if (x[0] == 0xff) & (x[6] == 0xfe):
                        lb1.configure(text=int(x[1]))
                        lb2.configure(text=int(x[2]))
                        lb3.configure(text=int(x[3]))
                        lb4.configure(text=int(x[4]))
                        lb5.configure(text=int(x[5]))
                        for i in range(1, 6, 1):
                            temp[i] = int(x[i])
                        pos[6] = int(vib_int.get())
                        ser_write()
                    elif (x[0] == 0xff) & (x[6] == 0xfd):       # receive when calibration terminated
                        lb0.configure(text='Calibrated!')

        ser.write(bytearray([255, 0, 0, 0, 0, 0, 0, 252]))     # stop motor
        ser.close()
        self.root.quit()
        self.root.update()

def high():
    global factor
    if factor < 30:
        factor += 1
        lb11.configure(text=factor)

def low():
    global factor
    if factor > 1:
        factor -= 1
        lb11.configure(text=factor)

def exitProgram():
    global loop_active
    loop_active = False

upd = Upd(win)

lb0.pack()
lb1.pack()
lb2.pack()
lb3.pack()
lb4.pack()
lb5.pack()
lb6.pack()
lb7.pack()
lb8.pack()
lb9.pack()
lb10.pack()

scale1.pack(anchor=CENTER)

bt3 = Button(win, text = '+', command = high)
bt4 = Button(win, text = '-', command = low)
bt5 = Button(win, text = 'exit', command = exitProgram)
bt6 = Button(win, text = 'calibration', command = init_fsr)

bt3.pack()
bt4.pack()
lb11.pack()
bt6.pack()
bt5.pack()

win.mainloop()
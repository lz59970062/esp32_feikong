import _thread
from tkinter import messagebox
import numpy
import serial  # 导入模块
import struct
import serial.tools.list_ports
import time
from tkinter import *
import pyqtgraph as pg
import numpy as np
import array
import sympy
read_gap=0
expect,measure=0,0
def drawgrp():
    global ser,expect,measure
    app = pg.mkQApp()  # 建立app
    pg.setConfigOption('background', 'w')
    win = pg.GraphicsWindow()  # 建立窗口
    win.setWindowTitle(u'串口绘图器')
    win.resize(800, 500)  # 小窗口大小

    data = array.array('d')  # 可动态改变数组的大小,double型数组
    historyLength = 100  # 横坐标长度
    p = win.addPlot()  # 把图p加入到窗口中
    p.showGrid(x=True, y=True)  # 把X和Y的表格打开

    p.setLabel(axis='left', text='y / V')  # 靠左
    p.setLabel(axis='bottom', text='x / point')
    p.setTitle('串口绘图器')  # 表格的名字
    curve1 = p.plot(pen='b')  # 绘制一个图形
    curve2=p.plot(pen='r')
    data1 = array.array('d')
    data2 = array.array('d')

    def plotData():
        global idx ,ser,expect,measure# 内部作用域想改变外部域变量

        if len(data) < historyLength:
            data1.append(expect)
            data2.append(measure)

        else:
            data1[:-1] = data1[1:]  # 前移
            data1[-1] = expect
            data2[:-1]=data2[1:]
            data2[-1]=measure

        curve1.setData(data1)
        curve2.setData(data2)

    timer = pg.QtCore.QTimer()
    timer.timeout.connect(plotData)  # 定时调用plotData函数
    timer.start(int(read_gap*1000))  # 多少ms调用一次

    app.exec_()


port_list = list(serial.tools.list_ports.comports())
print(port_list)
if len(port_list) == 0:
    print('无可用串口')
else:
    for i in range(0, len(port_list)):
        print(port_list[i])

ser = None


def do_connect(portx='COM6'):
    global ser
    try:
        # 端口，GNU / Linux上的/ dev / ttyUSB0 等 或 Windows上的 COM3 等

        # 波特率，标准值之一：50,75,110,134,150,200,300,600,1200,1800,2400,4800,9600,19200,38400,57600,115200
        bps = 115200
        # 超时设置,None：永远等待操作，0为立即返回请求结果，其他值为等待超时时间(单位为秒）
        timex = 5
        # 打开串口，并得到串口对象
        ser = serial.Serial(portx, bps,bytesize=8,parity='N',stopbits=1, timeout=timex)
        print('ok')
    except Exception as e:
        print("---异常---：", e)

print(str(port_list[0]).split()[0])
do_connect(portx=str(port_list[0]).split()[0])
# 设置tkinter窗口
root = Tk()
root.geometry('1200x300')#窗口大小
# 绘制两个label,grid（）确定行列
Label(root, text="一阶位置xKp").grid(row=0, column=3)
Label(root, text="一阶位置xKi").grid(row=0, column=5)
Label(root, text="一阶位置xKd").grid(row=0, column=7)
Label(root, text="一阶位置yKp").grid(row=0, column=9)
Label(root, text="一阶位置yKi").grid(row=0, column=11)
Label(root, text="一阶位置yKd").grid(row=0, column=13)
Label(root, text="一阶位置zKp").grid(row=0, column=15)
Label(root, text="一阶位置zKi").grid(row=0, column=17)
Label(root, text="一阶位置zKd").grid(row=0, column=19)
Label(root, text="二阶位置xKp").grid(row=1, column=3)
Label(root, text="二阶位置xKi").grid(row=1, column=5)
Label(root, text="二阶位置xKd").grid(row=1, column=7)
Label(root, text="二阶位置yKp").grid(row=1, column=9)
Label(root, text="二阶位置yKi").grid(row=1, column=11)
Label(root, text="二阶位置yKd").grid(row=1, column=13)
Label(root, text="二阶位置zKp").grid(row=1, column=15)
Label(root, text="二阶位置zKi").grid(row=1, column=17)
Label(root, text="二阶位置zKd").grid(row=1, column=19)
Label(root, text="一阶高度Kp").grid(row=2, column=3)
Label(root, text="一阶高度Ki").grid(row=2, column=6)
Label(root, text="一阶高度Kd").grid(row=2, column=9)
Label(root, text="二阶高度Kp").grid(row=3, column=3)
Label(root, text="二阶高度Ki").grid(row=3, column=6)
Label(root, text="二阶高度Kd").grid(row=3, column=9)
Label(root,text="三阶高度p").grid(row=4,column=6)
Label(root,text="3i").grid(row=4,column=8)
Label(root,text="3d").grid(row=4,column=10)

Label(root,text='预期vx').grid(row=4,column=1)
Label(root,text='预期vx').grid(row=5,column=1)
Label(root,text="期望角").grid(row=3,column=11)
Label(root,text="state").grid()

Label(root,text="电机启动").grid(row=8,column=3)
buffer=None
# 导入两个输入框
xpkp1 = Entry(root,width=6)
xpki1 = Entry(root,width=6)
xpkd1 = Entry(root,width=6)
ypkp1 = Entry(root,width=6)
ypki1 = Entry(root,width=6)
ypkd1 = Entry(root,width=6)
zpkp1 = Entry(root,width=6)
zpki1 = Entry(root,width=6)
zpkd1 = Entry(root,width=6)
xpkp2 = Entry(root,width=6)
xpki2 = Entry(root,width=6)
xpkd2 = Entry(root,width=6)
ypkp2 = Entry(root,width=6)
ypki2 = Entry(root,width=6)
ypkd2 = Entry(root,width=6)
zpkp2 = Entry(root,width=6)
zpki2 = Entry(root,width=6)
zpkd2 = Entry(root,width=6)
hkp1 = Entry(root,width=6)
hki1 = Entry(root,width=6)
hkd1 = Entry(root,width=6)
hkp2 = Entry(root,width=6)
hki2 = Entry(root,width=6)
hkd2 = Entry(root,width=6)
hkp3 = Entry(root,width=6)
hki3=Entry(root,width=6)
hkd3=Entry(root,width=6)
ex = Entry(root,width=6)
ey = Entry(root,width=6)
ez = Entry(root,width=6)
vx=Entry(root,width=5)
xp=Entry(root,width=5)
xi=Entry(root,width=5)
xd=Entry(root,width=5)

vy=Entry(root,width=6)
yp=Entry(root,width=5)
yi=Entry(root,width=5)
yd=Entry(root,width=5)
e_h=Entry(root,width=6)

evx=Entry(root,width=6)
evy=Entry(root ,width=6)
s0=Entry(root,width=6)
s1=Entry(root,width=6)
s2=Entry(root,width=6)
s3=Entry(root,width=6)

start=Entry(root,width=6)

# 设置输入框的位置
xpkp1.grid(row=0, column=4)
xpki1.grid(row=0, column=6)
xpkd1.grid(row=0, column=8)
ypkp1.grid(row=0, column=10)
ypki1.grid(row=0, column=12)
ypkd1.grid(row=0, column=14)
zpkp1.grid(row=0, column=16)
zpki1.grid(row=0, column=18)
zpkd1.grid(row=0, column=20)
xpkp2.grid(row=1, column=4)
xpki2.grid(row=1, column=6)
xpkd2.grid(row=1, column=8)
ypkp2.grid(row=1, column=10)
ypki2.grid(row=1, column=12)
ypkd2.grid(row=1, column=14)
zpkp2.grid(row=1, column=16)
zpki2.grid(row=1, column=18)
zpkd2.grid(row=1, column=20)
hkp1.grid(row=2, column=5)
hki1.grid(row=2, column=7)
hkd1.grid(row=2, column=10)
hkp2.grid(row=3, column=5)
hki2.grid(row=3, column=7)
hkd2.grid(row=3, column=10)
hkp3.grid(row=4,column=7)
hki3.grid(row=4,column=9)
hkd3.grid(row=4,column=11)
ex.grid(row=3, column=11)
ey.grid(row=3, column=12)
ez.grid(row=3, column=13)
e_h.grid(row=3,column=14)
evx.grid(row=4,column=2)
xp.grid(row=4,column=3)
xi.grid(row=4,column=4)
xd.grid(row=4,column=5)
evy.grid(row=5,column=2)
yp.grid(row=5,column=3)
yi.grid(row=5,column=4)
yd.grid(row=5,column=5)
s0.grid(row=6,column=1)
s1.grid(row=6,column=2)
s2.grid(row=6,column=3)
s3.grid(row=6,column=4)
start.grid(row=8,column=4)

var = StringVar()
Label(root, textvariable=var, bg='white', fg='black', font=('Arial', 12), width=100, height=200)


do_connect()

#def debug_curve(get):

def update():
    global ser,buffer
    try:
        buffer = struct.pack("fffffffffffffffffffffffffffffffffffffff????f",float(xpkp1.get()),float(ypkp1.get()),float(zpkp1.get()),float(xpki1.get()),float(ypki1.get()),float(zpki1.get()), float(xpkd1.get()),float(ypkd1.get()),float(zpkd1.get()),
                             float(xpkp2.get()),float(ypkp2.get()),float(zpkp2.get()),float(xpki2.get()),float(ypki2.get()),float(zpki2.get()), float(xpkd2.get()),float(ypkd2.get()),float(zpkd2.get()),float(hkp1.get()),
                             float(hkp2.get()),float(hkp3.get()),float(hki1.get()),float(hki2.get()),float(hki3.get()),float(hkd1.get()),float(hkd2.get()),float(hkd3.get()),float(ex.get()),float(ey.get()),float(ez.get()),float(e_h.get()),float(evx.get()),float(xp.get()),float(xi.get()),
                             float(xd.get()),float(evy.get()),float(yp.get()),float(yi.get()),float(yd.get()),int(s0.get()),int(s1.get()),int(s2.get()),int(s3.get()),float(start.get())
                             )
        result=ser.write(buffer)
        print(struct.unpack("fffffffffffffffffffffffffffffffffffffff????f", buffer))
    except Exception as f:
        messagebox.showinfo('error', f)


# 清除函数，清除输入框的内容
def dele():
    xpkp1.delete(0, END)
    xpki1.delete(0, END)
    xpkd1.delete(0, END)
    ypkp1.delete(0, END)
    ypki1.delete(0, END)
    ypkd1.delete(0, END)
    zpkp1.delete(0, END)
    zpki1.delete(0, END)
    zpkd1.delete(0, END)
    xpkp2.delete(0, END)
    xpki2.delete(0, END)
    xpkd2.delete(0, END)
    ypkp2.delete(0, END)
    ypki2.delete(0, END)
    ypkd2.delete(0, END)
    zpkp2.delete(0, END)
    zpki2.delete(0, END)
    zpkd2.delete(0, END)
    hkp1.delete(0, END)
    hki1.delete(0, END)
    hkd1.delete(0, END)
    hkp2.delete(0, END)
    hki2.delete(0, END)
    hkd2.delete(0, END)
    start.delete(0,END)


def serialread():
    global ser, var,read_gap,expect,measure
    nowti=time.time()
    if ser != None:
        while 1:
            try:
                read_gap=time.time()-nowti
                nowti=time.time()
                temp=ser.readline()
                try:
                    expect,measure=eval(temp)[0:2]
                except:
                    pass

                print(temp)
                #print(expect,measure)
                del temp
                ##var.set(ser.read_all())

            except:
                pass

def load():
    global  buffer
    temp=struct.unpack('fffffffffffffffffffffffffffff',buffer)
    temp=numpy.array(temp)
    xpkp1.insert(0,str(temp[0]))
    xpki1.insert(0,str(temp[1]))
    xpkd1.insert(0,str(temp[2]))
    ypkp1.insert(0,str(temp[3]))
    ypki1.insert(0,str(temp[4]))
    ypkd1.insert(0,str(temp[5]))
    zpkp1.insert(0,str(temp[6]))
    zpki1.insert(0,str(temp[7]))
    zpkd1.insert(0,str(temp[8]))
    xpkp2.insert(0,str(temp[9]))
    xpki2.insert(0,str(temp[10]))
    xpkd2.insert(0,str(temp[11]))
    ypkp2.insert(0,str(temp[12]))
    ypki2.insert(0,str(temp[13]))
    ypkd2.insert(0,str(temp[14]))
    zpkp2.insert(0,str(temp[15]))
    zpki2.insert(0,str(temp[16]))
    zpkd2.insert(0,str(temp[17]))
    hkp1.insert(0,str(temp[18]))
    hki1.insert(0,str(temp[19]))
    hkd1.insert(0,str(temp[20]))
    hkp2.insert(0,str(temp[21]))
    hki2.insert(0,str(temp[22]))
    hkd2.insert(0,str(temp[23]))
    start.insert(0,str(temp[24]))

def seropen():
    global ser
    try:
       ser.open()
    except:
       ser = serial.Serial(str(list(serial.tools.list_ports.comports())[0]).split()[0], 115200, bytesize=8, parity='N',
                        stopbits=1, timeout=2)
# 设置两个按钮，点击按钮执行命令 command= 命令函数
theButton1 = Button(root, text="上传", width=10, command=update)
theButton2 = Button(root, text="清除", width=10, command=dele)
theButton5= Button(root,text='加载',width=10,command=load)
if ser != None:
    theButton3 = Button(root, text="关闭串口", width=10, command=ser.close)

    theButton4 = Button(root,text="打开串口",width=10,command=seropen)

    theButton4.place(x=450, y=250)
    theButton3.place(x=350, y=250)
# 设置按钮的位置行列及大小
theButton1.place(x=150, y=250)
theButton2.place(x=250, y=250)
theButton5.place(x=550,y=250)

_thread.start_new_thread(serialread, ())
_thread.start_new_thread(drawgrp,())
mainloop()

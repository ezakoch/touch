# Live-plotting serial with matplotlib
# cd Dropbox/TOUCH/m2
# Eza Koch on 5/14/15

import serial
import signal
import sys
import numpy as np
from matplotlib import pyplot as plt

def signal_handler(signal, frame):
    m2.close()
    sys.exit(0)
signal.signal(signal.SIGINT, signal_handler)


m2 = serial.Serial('/dev/tty.usbmodem411', 9600)	# A new serial port is opened @ 9600 baud and named 'm2'
m2.flushInput()			# The input buffer is cleared, in case anything is leftover

while 1:

	#m2.write('1')	# Signal to the m2 that we want data!
	#print m2.read()

	user_input = raw_input('Write to device? \n(1) Duty Cycle 1\n(2) Temperature 2\nInput: ')
	if user_input == '1':
		m2.write('1')
		print m2.read()
	#elif user_input == '2':
	#	m2.write('2')
		#print m2.read()

#https://www.lebsanft.org/?p=48

plt.ion() # set plot to animated
ydata = [0] * 50 #the range of the x-axis
ax1=plt.axes()
line, = plt.plot(ydata)
plt.ylim([0,100]) # set the y-range to 0 to 100
# start data collection
while True:  
    data = ser.readline().rstrip() # read data from serial port and strip line endings
    if len(data.split(".")) == 2:
        ymin = float(min(ydata))-10
        ymax = float(max(ydata))+10
        plt.ylim([ymin,ymax])
        ydata.append(data)
        del ydata[0]
        line.set_xdata(np.arange(len(ydata)))
        line.set_ydata(ydata)  # update the data
        plt.draw() # update the plot
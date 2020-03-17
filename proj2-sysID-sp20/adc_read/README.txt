This is an instruction on how to use your launch-pad as a psuedo-oscilloscope with capability of showing voltages in range of 0 and 3.3V.


Prerequisites:
 - Install pyserial on you computers
 	pip install pyserial

Steps:
1. Upload the adc_read.ino to the launch pad.
2. Use P6.0 as your probe (You modify the file if you want to use another pin. Remember that you should always choose an analg pin).
3. 
	- getting help:
		python adc_read.py --help
	- Example run command:
		python adc_read.py -D /dev/tty.usbmodem31 -b 9600 -p

You can use these steps to tune the output of your mic-board or debug the circuits with less than 3.3V voltage domain.
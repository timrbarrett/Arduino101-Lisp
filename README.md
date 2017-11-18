# Arduino101-Lisp
BLE Based Lisp REPL

See separate bug list for things that fail tests

Procedure is to 
1) attach the genuino, arduino 101, or tiny tile via USB to the machine with the code.
2) Download and execute the code, monitoring the serial port. 
3) Check that Bluetooth is waiting for a connection, sd card is confirmed and IMU initialised.
4) Switch to the AI2 app. Scan, find yLisp, connect.
5) confirm serial ouput shows a connection, and that listpicker of ble advertiser has closed
6) press "(add 1 2)" button
7) confirm the answer 3 is the result, on serial output.
8) (assuming electrodes are in place, med fit 3 is wired up, levels are right) then when the big grey button is pressed, there should be a short delay, then the correct electrical signal is delivered to the shin.

If the fails at step 3 goto step 1
Failure at 4 goto step 1
Failure at 7 goto step 1

Implemented
- integer maths works 
  eg (add 1 2) => 3
  
- calling built in functions
  eg (dw 3 1) => () and pin 3 goes high
     (dw 3 0) => () pin 3 is set to low
     (ad5252 1 128) => () and set the wiper for channel 1 to 128
     (fet 1 0) => () and both fet digit pins aare written low

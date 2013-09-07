VPythonIMUGui
=============
The VPython IMU Gui is essentially a port of the FreeIMU Demo Cube and the FreeIMU YPR Demo from the 
FreeIMU Processing code developed by Fabio Versano. The code is based in part on the code developed by 
Jose Julio @2009. Changes made were quite extensive and include:

  1. Created a color coded rectange similar to that of the FreeIMU Demo Cube
   2. Created a downward pointing arrow to assist in direction visualization for
      z-axis
   3. Changed font sizes
   4. Added data display window to see the data stream
   5. Added routines to convert quatenions to Euler angle ported from the
      FreeIMU Demo Cube based in Processing

To work out of the box you will also need the FreeIMU_serial.ino sketch for the Arduino otherwise
you will need to modify the code to read and convert the serial stream from the Arduino.

A youtube video is available here (http://youtu.be/pylMfUNIyHA) to see this in action.

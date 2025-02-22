==================================
2429's Custom Camera code for 2024
==================================

There are three python files in here:
==================================
 grip.py - the GRIP pipeline
==================================
With a minor modification so we can change HSV values.   
It takes an image, blurs it, HSV thresholds it, gets contours from the threshold and filters the contours

======================================
 multiCameraServer.py - server config
======================================
starts the MJPEG server for our camera and feeds it images from the spartan_overlay.py

==================================
spartan_overlay.py - custom code
==================================
this is the custom one for us, and we can do whatever we want here.  
It takes the GRIP output and then does our calculations.  It's set up for an older game with the green lines, etc.  Think about how you would like to present aids to the driver and try to add them.  

I updated it to run in training mode as:
python spartan_overlay.py yellow  
- but that color option can be red, blue, green, yellow.  You can hold up objects in front of your camera and track and see the values (hopefully).  Try using it and see if you can figure out how it works and try your own overlays.
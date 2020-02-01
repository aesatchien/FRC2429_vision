# FRC2429_historical
Repo for the different years of vision processing code for 2429

Maintained by CJH

## Summary of code 
#### Note that each of these has been updated to work with 2020 java libraries
#### As of 2019, to install on the pi, just upload the /build/libs/SpartanVision_20XX-all.jar to the wpilib rpi webpage
#### images will be on cameraserver:1185 (unprocessed) and cameraserver:1186 (processed) for the gear/cube cam and 1187/1188 on shootercam
#### So e.g. point chrome to http://frcvision.local:1186/ or http://10.24.29.12:1186/ to view the stream

**jar**: Just the runable java jar files from each year that you can upload to the pi  

**2017**: Java- tracking the location of the gears and wiffle green target, info analyzed by opencv and sent to dashboard
			Both cameras in one program, so start it with a 0 for *gearcam* and 1 for *shootercam*.  They're also saved separately so you can call them individually and not have to worry about that.

**2018**: Java- *cubecam* looking at and tracking cubes (good starting point for 2020 balls), data sent to dashboard

**2019**: Java- mainly a simple overlay on camera because we did not track objects

**2020**: Plan to switch to python, break it into modules for 1) different targets and 2) posting to networktables

		TODO: add PIXY and other camera codes in addition to opencv

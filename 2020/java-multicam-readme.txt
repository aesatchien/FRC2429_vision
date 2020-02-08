2/1/2020 - Notes on getting the default multicamera jar to work with our stuff
Basically added the following:

Get the extra imports:
import edu.wpi.cscore.HttpCamera;  // CJH Addition
import edu.wpi.first.networktables.NetworkTableEntry;  // CJH Addition
import edu.wpi.first.networktables.NetworkTable;	//CJH Addition
import edu.wpi.cscore.HttpCamera.HttpCameraKind; // CJH Addition

Add some NT entries:
  static NetworkTableEntry targetsEntry;
  static NetworkTableEntry distanceEntry;
  static NetworkTableEntry rotationEntry;
  static NetworkTableEntry strafeEntry;
  static NetworkTable ballTable;
  
Add a second camera source from the processed mat:
 /	//****************  START CJH ADDITION  *************
    // This creates a CvSource to use. This will take in a Mat image that has had OpenCV operations
    // FPS of the MjpegServer is set here 
    int xResolution = 320;
    int yResolution = 256;
    int processedPort = 1182;

    CvSource imageSource = new CvSource("CV Image Source", VideoMode.PixelFormat.kMJPEG, xResolution, yResolution, 20);
    MjpegServer cvStream = new MjpegServer("CV Image Stream", processedPort);
    //cvStream.getProperty("compression").set(3);
    cvStream.setSource(imageSource);
    CameraServer inst2 = CameraServer.getInstance();
    inst2.addCamera(imageSource);

    //added this on 3/29/2019 to see if it would then show up in the NT - and it does!  Now I need to customize it.
	System.out.println("*** Starting 2429 BallCam Processed stream on " + processedPort);
    final HttpCamera camera = new HttpCamera("BallCam Processed", "http://10.24.29.12:1182/?action=stream", HttpCamera.HttpCameraKind.kMJPGStreamer);
    CameraServer.getInstance().addCamera(camera);
    //myShuffleboardTab.add(camera);
	
	// start NetworkTables
	
    NetworkTableInstance ntinst = NetworkTableInstance.getDefault();
	ballTable = ntinst.getTable("BallCam");
	targetsEntry = ballTable.getEntry("targets");
    distanceEntry = ballTable.getEntry("distance");
	rotationEntry = ballTable.getEntry("rotation");
	strafeEntry = ballTable.getEntry("strafe");
	//****************  END CJH ADDITION  *************
	
Add the custom pipeline:
// ************** CJH CUSTOM PIPELINE  ***********
    // start image processing on camera 0 if present
    if (cameras.size() >= 1) {
      VisionThread visionThread = new VisionThread(cameras.get(0),
              new GripPipelineQ() , pipeline -> {
        // do something with pipeline results
        int size = pipeline.filterContoursOutput().size();
        double distance = pipeline.getDistance();
        targetsEntry.setNumber(size);
        distanceEntry.setNumber(Math.round(100*distance)/100.0);
        ntinst.flush();
        imageSource.putFrame(pipeline.gripImage());
      });
// ************** END CJH CUSTOM PIPELINE  ***********
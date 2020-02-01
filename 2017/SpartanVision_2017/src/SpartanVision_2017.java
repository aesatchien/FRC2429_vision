import edu.wpi.first.wpilibj.networktables.*;
import edu.wpi.first.wpilibj.tables.*;
import edu.wpi.cscore.*;
import java.util.Iterator;
import java.util.ArrayList;
import java.util.List;
import java.util.Collections;
import java.util.Comparator;
import java.awt.Color;
import java.awt.color.*;
import java.lang.Double;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfInt;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;

/*  CHANGELOG
 * 
 *  2/01/2017 Got a very good fit for three distances for a fairly general angle formulation
 *  2/05/2017 Converted areas to convex hulls to deal with finger-poking; added joining of area 2 and 3 if split
 *  2/07/2017 Added more robust zero distance because when you drive too close you get tons of bogeys
 *  		 Specifically, more filtering on the contours (aspect ratio, must line up in y)
 *  2/09/2017 Started adding an if statement for the shooterCam vs gearCam, just covered names so far
 *  2/10/2017 Added the fisheye polynomial correction to get the strafe right and send the strafe by height, not target width
 *  2/11/2017 Have to archive the MS Lifecam settings for the gear - no way to make it work.  Ordered one with a larger FOV
 *  2/11/2017 Got the basic structure of the shooter logic done, sending back a rotation and a distance based on 4" tape 
 *  2/12/2017 Changing the approach for the strafe calculation to not use trig w/polynomial fix but the % FOV with a linear offset
 *  2/12/2017 Adding more cameras since I can't make up my mind yet, and now calibrated them all
 *  2/13/2017 Making the shootercam a bit more fun
 *  2/14/2017 Making the aspect ratio more strict for the gearcam because we get too much other stuff during delivery
 *  2/19/2017 Making things black on the top and bottom for easier reading of the output
 *  2/21/2017 Drawing clearer lines on the screen as an aide to the eye in case the green is not working (fail safe on the shooting) 
 *  2/22/2017 Added a moving distance bar on the right so the distance is meaningful and helps you center your shot
 *  3/02/2017 Made the bars thicker for steering/aligning in gearcam 
 *  3/05/2017 Switched the Shootercam distance calculation to one that depends on the y centroid of the top target (empirical, 2nd order polynomial fit) 
 *  3/10/2017 Getting rid of my old gearcam code that let you poke a target in half and now trying to grab the best two
 *  3/16/2017 Moved rotation offset down so it resets to zero every time; it was remembering the previous value
 *  3/16/2017 Added a circle to the gearcam so the driver can orient on the gear drop reflection and get the distance right
 *  3/25/2017 Changed the HSV values to allow more blue; that seems to happen when it saturates, so we can turn the camera higher
 *  		  Also, the competition LEDs may be just a tad bluer than my old ones I was testing with.  So move top from 89 to 96 on hue.
 *  		  I think this HSV change is the most important thing we can do to help with getting targets consistently.
 *  		  Perhaps moving Value from 90 to 110 will help when we are too bright and the targets bleed together, but I don't think that's our problem
 *  		  One other thing that may help is lowering the saturation from 255 - that may help when outside.
 *  
 */

public class SpartanVision_2017{
	
  static NetworkTable table;
	
  @SuppressWarnings("static-access")
public static void main(String[] args) {
    // Loads our OpenCV library. This MUST be included
    System.loadLibrary("opencv_java347");
    
    // Connect NetworkTables, and get access to the publishing table
	//table.setTeam(2429);  //this should connect to the RIO ...
    //table.setIPAddress("192.168.1.10");  //Practice at home, see the network table on the laptop
	//table.setIPAddress("roboRIO-2429-FRC.local");  // roboRIO should always answer here... but also setTeam does this
	
    //More robust way to grab anybody who is listening. Copied the setTeam method and added home -CJH
    int team = 2429;
    String[] addresses = new String[5];
    addresses[0] = "10." + (int)(team / 100) + "." + (int)(team % 100) + ".2";
    addresses[1] = "roboRIO-" + team + "-FRC.local";  //mDNS address of the roboRIO
    addresses[2] = "roboRIO-" + team + "-FRC.lan";
    addresses[3] = "192.168.1.30";  //at home
    addresses[4] = "172.22.11.2";  //apparently the roboRIO USB
    table.setIPAddress(addresses);
    table.setClientMode();
	
    //start making decisions on the type of target we're interested in    
	boolean bGearCam;
	String cameraName;
	int streamPort;
	int processedPort;
	int xResolution;
    int yResolution;
    int cameraShift = 0;  // for dealing with cameras with manufacturing flaws  
    double cameraFov;  
    double strafeCorrection = 1.0;
    double shooterCosCorrection = 1.0;
    int cameraChoice = 0;
    
    
    //Choose camera
	bGearCam = false;
    // Set the resolution for our camera, since this is over USB
    
    if (args.length > 0 ){
    	int cameraType = Integer.parseInt(args[0]);
    	if (cameraType == 0){
    		bGearCam = true;
    	}else{
    		bGearCam = false;
    	}
    }
    
    if (bGearCam){
   	 cameraName = "GearCam";
     // This is the network port you want to stream the raw received image to
     // By rules, this has to be between 1180 and 1190, so 1185 is a good choice
   	 streamPort = 1185;
   	 processedPort = 1186;
   	 cameraChoice = 2;
   	

   }else{
   	 cameraName = "ShooterCam";
   	 streamPort = 1187;
   	 processedPort = 1188;
   	 cameraChoice = 1;  //MS Lifecam in 320x240 seems best for the shooter cam
   	 shooterCosCorrection = 1.0;
   }
    
    System.out.println("Mode set to " + cameraName);
    table = NetworkTable.getTable(cameraName);
	 // This stores our reference to our mjpeg server for streaming the input image
    MjpegServer inputStream = new MjpegServer(cameraName, streamPort);
    
    //Set up the camera based on the choice made above
    switch (cameraChoice) {
    case 0: {
    	//MS Lifecam 416x240
    	cameraFov = 64.5;  //MS lifecam in 416/240 aspect ratio, measured by CJH
    	strafeCorrection = 0.55;  //Takes FoV into account to see how far off center we are 
      	xResolution = 416;
   		yResolution = 240;
    	break;
    	}
    case 1: {
    	//MS Lifecam 320x240 
    	cameraFov = 55;   //MS lifecam in 640/480 aspect ratio, measured by CJH
    	strafeCorrection = 0.464;
      	xResolution = 320;
   		yResolution = 240;
    	break;
    }
    case 2: {
    	//Genius 120 352x288 aspect ratio
    	cameraFov = 118.0;  //Genius 120 352/288 aspect ratio, measured by CJH
    	strafeCorrection = 1.08;
      	xResolution = 352;
   		yResolution = 288;
   		//cameraShift = 14;  //The practice bot camera had a shift of 14 pixels; it's on the practice bot
   		cameraShift = -8;  // The competition bot camera seems to not be as defective, but in the wrong direction
    	break;
    }
    case 3: {
    	//Logitech C270
    	cameraFov = 79.0;  //Logitech C290 432x240, measured by CJH
    	strafeCorrection = 0.703;
      	xResolution = 432;
   		yResolution = 240;
    	break;
    }
    case 4: {
    	//Logitech C270
    	cameraFov = 59;  //Logitech C290 432x240, measured by CJH
    	strafeCorrection = 0.519;
      	xResolution = 432;
   		yResolution = 240;
    	break;
    }
    case 5: {
    	//ELP  100 degree
    	cameraFov = 78;  //EPL 100 degree
    	strafeCorrection = 0.64;
      	xResolution = 352;
   		yResolution = 288;
    	break;
    }
    default: {
    	//If we somehow don't get what we want
    	cameraFov = 60.0;  
       	strafeCorrection = 1.0;
      	xResolution = 320;
   		yResolution = 240;
    }
    
    }//end switch on camera type

   
    /***********************************************/
    // USB Camera
    // Usually this will be on device 0, but there are other overloads that can be used
    UsbCamera camera = setUsbCamera(0, inputStream);
    //This is the one I have been using - CJH
 
    // These are good settings for the MS Lifecame in indoor lighting.  May need to be dimmer if bright lights around.
    //camera.setBrightness(40);
   // camera.setExposureManual(10);
    camera.setWhiteBalanceManual(3800);  //Good for indoors
    camera.setVideoMode(VideoMode.PixelFormat.kMJPEG, xResolution, yResolution, 20);
    // This creates a CvSink for us to use. This grabs images from our selected camera, 
    // and will allow us to use those images in opencv
    CvSink imageSink = new CvSink("CV Image Grabber");
    imageSink.setSource(camera);

    // This creates a CvSource to use. This will take in a Mat image that has had OpenCV operations
    // FPS of the MjpegServer is set here 
    CvSource imageSource = new CvSource("CV Image Source", VideoMode.PixelFormat.kMJPEG, xResolution, yResolution, 10);
    MjpegServer cvStream = new MjpegServer("CV Image Stream", processedPort);
    cvStream.setSource(imageSource);
    
    // All Mats and Lists should be stored outside the loop to avoid allocations
    // as they are expensive to create
    //Mat inputImage = new Mat();
    //Mat hsv = new Mat();

    
    /***********************************************/
    
   //  CJH start here
    
    //Details of the text overlay
	int targets = 0;
	long startTime;
	long endTime;
	
    //Details of the text overlay
	Point infoTextLocation= new Point((int)(0.03*xResolution),13);
    Point targetTextLocation= new Point((int)(0.7*xResolution),13);
    Point targetDistTextLocation= new Point((int)(0.03*xResolution),yResolution-20);
    Point targetAreaTextLocation= new Point((int)(0.03*xResolution),yResolution-5);
    //
    Scalar infoTextColor= new Scalar(0,255,255);
    Scalar targetTextColor= new Scalar(0,255,0);
    Scalar targetWarningColor= new Scalar(20,20,255);
    Scalar targetColor;
    double actualTargetWidth = 8.25;
    double actualTargetHeight = 5.3; 
    if (bGearCam){
    	//Yellow target outline
    	targetColor = new Scalar (0, 255, 255);
        actualTargetWidth = 8.25;
        actualTargetHeight = 5.5; //Seems to work better this way, not sure how to fix this
   }else{
	   //Red Target outline
	   	targetColor = new Scalar (0, 0, 255);
	   	targetTextColor = new Scalar (255, 255, 0);
	    actualTargetWidth = (0.85)*15.0;  // width of the top target, empirically fit with a modifier to the diameter
	    // Fit width to (10/12)*diameter
	    actualTargetHeight = 7.0; // centroid to centroid
   }

    
    //Make some Mats to work with
    Mat inputMat = new Mat();
    Mat processedMat = new Mat();
    
    
    //stuff I shouldn't have to reinitialize each time
	double imageTargetWidth =0;
	double imageTargetHeight =0;
	double imageOffsetFromZero =0;
	double areaRatio =0;
	
	double aspectRatio = (double)yResolution/(double)xResolution;
	int counter = 0;
	double averageRotation [] = {0,0,0,0};
	double averageAreaRatio [] = {0,0,0,0};
	
    
    // Infinitely process image
    while (true) {
      // Grab a frame. If it has a frame time of 0, there was an error.
      // Just skip and continue
    	
      counter++;
	  double targetAreas [] = {0.0,0.0};
	  double testAreas [] = {0.0,0.0};
	  double targetX [] = {0.0,0.0};
	  double targetY [] = {0.0,0.0};
	  double centroidY [] = {0.0,0.0};
	  double targetHeights [] = {0.0,0.0};
	  double targetWidths [] = {0.0,0.0};
	  double targetAspectRatio [] = {0.0,0.0};
	  double targetBleedover [] = {0.0,0.0};
	  //Rectangle rectangles [] = new Rectangle[2];
  	  double distanceToTarget = 0;
  	  double distanceToTargetByHeight = 0;
  	  double distanceToTargetByWidth = 0;
  	  double strafeToTarget = 0;
  	  double strafeToTargetByHeight = 0;
  	  double rotationOffset=0;

  	  double fovRadians = (Math.PI/180.0)*(cameraFov);
  	  double fisheyeCorrection =1.0;
  	  
	   
      long frameTime = imageSink.grabFrame(inputMat);
      if (frameTime == 0) continue;

      // Below is where you would do your OpenCV operations on the provided image
  
     	startTime=System.nanoTime();
        
        //Call the process tree
        
        //Blur
        int radius=2;
        int kernelSize = 2 * radius + 1;
		Imgproc.blur(inputMat, processedMat, new Size(kernelSize, kernelSize));
		
		//HSV
		double[] hsvThresholdHue = {53.0, 96.0};
		double[] hsvThresholdSaturation = {50.0, 255.0};
		double[] hsvThresholdValue = {90.0, 255.0};
		Imgproc.cvtColor(processedMat, processedMat, Imgproc.COLOR_BGR2HSV);
		Core.inRange(processedMat, new Scalar(hsvThresholdHue[0], hsvThresholdSaturation[0], hsvThresholdValue[0]),
				new Scalar(hsvThresholdHue[1], hsvThresholdSaturation[1], hsvThresholdValue[1]), processedMat);
		
		//Contours
		Mat hierarchy = new Mat();
		ArrayList<MatOfPoint> findContoursOutput = new ArrayList<MatOfPoint>();
		Imgproc.findContours(processedMat, findContoursOutput , hierarchy , Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
		
		//Filtered Contours
	    ArrayList<MatOfPoint> filterContoursOutput = new ArrayList<MatOfPoint>();
		ArrayList<MatOfPoint> filterContoursContours = findContoursOutput;
			
		double filterContoursMinArea = 50.0;
		double filterContoursMinPerimeter = 0;
		double[] filterContoursSolidity = {0, 100};
		double filterContoursMaxVertices = 1000000;
		double filterContoursMinVertices = 0;
		double filterContoursMinWidth;
		double filterContoursMaxWidth;
		double filterContoursMinHeight;
		double filterContoursMaxHeight;
		double filterContoursMinRatio;  //Ratio is width / height
		double filterContoursMaxRatio;
		
	    if (bGearCam){
			filterContoursMinWidth = 0.015 * xResolution;
			filterContoursMaxWidth = 0.3 * xResolution;
			filterContoursMinHeight = 0.02 * yResolution;
			filterContoursMaxHeight = 0.7 * yResolution;
			filterContoursMinRatio = 0.25;   // Gear target is 2/5 = 0.4
			filterContoursMaxRatio = 0.9;   //This kills my cool joining of contours
	    }else{
			filterContoursMinWidth = 0.015 * xResolution;
			filterContoursMaxWidth = 0.5 * xResolution;
			filterContoursMinHeight = 0.01 * yResolution;
			filterContoursMaxHeight = 0.3 * yResolution;
			filterContoursMinRatio = 1.25;  // Shooter target is probably 16" / 4" high, so expect 4 or higher
			filterContoursMaxRatio = 25;
	    }
		
		filterContours(filterContoursContours, filterContoursMinArea, filterContoursMinPerimeter, filterContoursMinWidth, filterContoursMaxWidth, filterContoursMinHeight, filterContoursMaxHeight, filterContoursSolidity, filterContoursMaxVertices, filterContoursMinVertices, filterContoursMinRatio, filterContoursMaxRatio, filterContoursOutput);

		/***********************************************/
		//Reject all but the largest two, then sort left to right if doing the gearCam

		int targetCount = filterContoursOutput.size();
		int minTargets = 1;
		minTargets = bGearCam ? 1 : 0;
		if (targetCount > 1) {
			//Sort based on area.  Remember the ordering, the forst time i did this i got the biggest one last and that's opposite of what I want
			Collections.sort(filterContoursOutput, new Comparator<MatOfPoint>() {
				  @Override public int compare(final MatOfPoint o1, final MatOfPoint o2) {
				    if (Imgproc.contourArea(o1) < Imgproc.contourArea(o2)) {
				      return 1;
				    } else if (Imgproc.contourArea(o1) > Imgproc.contourArea(o2)) {
				      return -1;
				    }  
				    return 0;
				  }
				});			
			
		//If the stick is cutting one side in half, join them
		//Although my stricter filtering of contours kills this - 2/14/2017
		/*	if (targetCount > 2) {
				//Get the centers of 2 and 3 in x
				Moments moments1 = Imgproc.moments(filterContoursOutput.get(1));
				Moments moments2 = Imgproc.moments(filterContoursOutput.get(2));
				double x1 = moments1.get_m10() / moments1.get_m00();
				double x2 = moments2.get_m10() / moments2.get_m00();
				double y1 = moments1.get_m01() / moments1.get_m00();
				double y2 = moments2.get_m01() / moments2.get_m00();
				double maxHeight = Imgproc.boundingRect(filterContoursOutput.get(0)).height;
				
				//See if 2 and three line up in x and aren't too far apart
				if ( (Math.abs(x1-x2)< 20 ) && ( Math.abs(y1-y2) < 0.8* maxHeight) ) {
					// Combine 2 and 3
					filterContoursOutput.get(1).push_back(filterContoursOutput.get(2));
				}
			}
			List<MatOfPoint> mainTargets = filterContoursOutput.subList(0, 2);
			*/
			
			// New code to only allow targets lined up in Y before we go into the loop
			// Find the ones that line up best in y and use them as the main two
			//List<MatOfPoint> mainTargets = filterContoursOutput.subList(0, 2);  //Not sure how to initialize the stupid thing
			ArrayList<MatOfPoint> mainTargets = new ArrayList<MatOfPoint>();
			
			if (targetCount > 2) {
				//Grab the two that are perfectly lined up
				double minTolerance = yResolution;
				int bestI = 0;
				int bestJ = 1;
				double yValues [] = new double[targetCount];
				for (int i=0; i< targetCount; i++){
					Moments moments1 = Imgproc.moments(filterContoursOutput.get(i));
					yValues[i] = moments1.get_m01() / moments1.get_m00();
					//System.out.println("Yval " + i + " " + yValues[i]);
				}				
				for (int i=0; i< targetCount-1; i++){
					for (int j=i+1; j < targetCount; j++) {
						if (Math.abs(yValues[i]-yValues[j]) < minTolerance){
							minTolerance = Math.abs(yValues[i]-yValues[j]);
							bestI = i;
							bestJ = j;
						}
					}
				}
				//System.out.println(bestI + " " + bestJ + " " + yValues[bestI] + " - " + yValues[bestJ] + " = " +  minTolerance);		
	
				mainTargets.add(filterContoursOutput.get(bestI));
				mainTargets.add(filterContoursOutput.get(bestJ));
			}
			else{
				//If there's no funny business, just grab the first two and that's what we analyze
				//Note: you can't clear this because then it erases the originals.  That was my original mistake
				mainTargets.add(filterContoursOutput.get(0));
				mainTargets.add(filterContoursOutput.get(1));
			}
			
			//On the gear sort left to right, on the boiler it's already sorted top to bottom
			  if (bGearCam){
					//Sort arrays left to right and write global variables based on the results of the target finding operation
					Collections.sort(mainTargets, new Comparator<MatOfPoint>() {
						  @Override public int compare(final MatOfPoint o1, final MatOfPoint o2) {
						    if (Imgproc.moments(o1).get_m10() / Imgproc.moments(o1).get_m00() > Imgproc.moments(o2).get_m10() / Imgproc.moments(o2).get_m00()) {
						      return 1;
						    } else if (Imgproc.moments(o1).get_m10() / Imgproc.moments(o1).get_m00() < Imgproc.moments(o2).get_m10() / Imgproc.moments(o2).get_m00()) {
						      return -1;
						    }  
						    return 0;
						  }
						});	
			    }

		
		/***********************************************/
		//Do all the post calculations

	    int count=0;
		int frameCount = filterContoursOutput.size();
		List<MatOfPoint> hulls  = new ArrayList<MatOfPoint>();
	    Iterator<MatOfPoint> each = mainTargets.iterator();
	    while (each.hasNext()) {
	        MatOfPoint wrapper = each.next();
	        Moments moments = Imgproc.moments(wrapper);
	        
	        Point centroid = new Point();
			centroid.x = -cameraShift + (moments.get_m10() / moments.get_m00());
			centroid.y = moments.get_m01() / moments.get_m00();
			Rect rectangle = Imgproc.boundingRect(wrapper);
			
			//Countour area is not robust to poking it with your finger
			//double area = Imgproc.contourArea(wrapper);
			//Convex Hull or Rectangle area is robust to poking it with your finger 
			final MatOfInt hull = new MatOfInt();
			Imgproc.convexHull(wrapper, hull);
			MatOfPoint mopHull = new MatOfPoint();
			mopHull.create((int) hull.size().height, 1, CvType.CV_32SC2);
			for (int j = 0; j < hull.size().height; j++) {
				int index = (int)hull.get(j, 0)[0];
				double[] point = new double[] { wrapper.get(index, 0)[0], wrapper.get(index, 0)[1]};
				mopHull.put(j, 0, point);
			}
			double area = Imgproc.contourArea(mopHull);
			//areas are a percentage of total image area
			targetAreas[count]= 100.0*area/(xResolution*yResolution);
			targetAspectRatio[count] = (double) rectangle.height / (double) rectangle.width;
			//X and Y are scaled from -1 to 1 in each direction, so distances in these coordinates
			//need to be divided by two to get percentages
			targetX[count]=(-1.0+ 2.0*centroid.x/xResolution);
			targetY[count]=(-1.0+ 2.0*centroid.y/yResolution);
			centroidY[count]=centroid.y;
			
			//The farther away you get, the worse the heights are
			targetBleedover[count]= rectangle.area()/area;
			//The height of the rectangle is in percentage of the y resolution
			targetHeights[count] = (rectangle.height)/((double)yResolution);
			targetWidths[count] = (rectangle.width);
			
			//Draw the rectangles, they are good to guide the eye
			Imgproc.rectangle(inputMat, rectangle.tl(), rectangle.br(), targetColor, 1);
			hulls.add(count,mopHull);
			count++;
	    }
	    
	    //Look in here for all the final calculations
		/***********************************************/
	 // Set aside a top bar for information
	 Imgproc.rectangle(inputMat, new Point(0,0), new Point(xResolution,0.06*yResolution), new Scalar(0,0,0), -1);  
   	 
   	 //This one is actually in scaled coordinates and gives you the distance of the midpoint from zero
   	 //Which is also the fraction of FOV/2 you are off center
   	 imageOffsetFromZero = (targetX[1]+targetX[0])/2.0;
     //I feel like i should average this but the lag is too high
   	 areaRatio = targetAreas[0]/targetAreas[1];
	    	    
	    if (bGearCam){
	    	imageTargetHeight = (targetHeights[1]+targetHeights[0])/2.0;  //average of the two
	    	imageTargetWidth = (targetX[1]-targetX[0])/2.0;
	    	distanceToTarget = actualTargetWidth / (2.0 * Math.tan(imageTargetWidth*fovRadians/2.0));
		    //This works for either one - the heights of the rectangles (avg for gear, sum for shooter)   	 
	     	distanceToTargetByHeight = actualTargetHeight / (2.0 * Math.tan((aspectRatio*imageTargetHeight*fovRadians/2.0)));
	     	 //This one is for distances on the MS Lifecam.  Other cameras need a different polynomial fit
	     	 //fisheyeCorrection = 2.4636*Math.pow(imageOffsetFromZero,4) - 3.123*Math.pow(Math.abs(imageOffsetFromZero),3) + 1.6596*Math.pow(imageOffsetFromZero,2) - 0.1444 * Math.abs(imageOffsetFromZero) + 1;  
	     	 //fisheyeCorrection = 1.0;
	     	 //strafeToTarget = (distanceToTarget/fisheyeCorrection) * 0.5*Math.tan(imageOffsetFromZero*fovRadians);
	     	 //strafeToTargetByHeight = (distanceToTargetByHeight/fisheyeCorrection) * 0.5*Math.tan(imageOffsetFromZero*fovRadians);
	     	strafeToTarget = distanceToTarget*strafeCorrection*imageOffsetFromZero;
	     	strafeToTargetByHeight = distanceToTargetByHeight*strafeCorrection*imageOffsetFromZero;
	     	//I added the negative so the angle we send to the robot is the angle it has to rotate to fix orientation - 02/03/2017 CJH
	    	rotationOffset = -(180.0/Math.PI)*Math.asin((distanceToTargetByHeight/4.12)*(Math.pow(areaRatio, 1.0/3.0)-1)/(Math.pow(areaRatio, 1.0/3.0)+1));
	    	averageRotation[counter%4] = rotationOffset*0.25;
	    	rotationOffset = 0;
	    	for (double element : averageRotation) {
	    		    rotationOffset+= element;
	    	}
	    	 
	    	//Check for bogus targets from saturation and reflections at impact
	    	 // Don't let the y-values of the targets differ by more than a few pixels
	    	if ( (Math.abs(targetY[0]-targetY[1]) > 0.4)
	    			 || (Math.abs(targetHeights[0]-targetHeights[1]) > 0.1*yResolution)
	    			 || (targetAspectRatio[0] < 1.0) || (targetAspectRatio[0] > 4.5) 
	    			 || (targetAspectRatio[1] < 1.0) || (targetAspectRatio[1] > 4.5)){
	    		 distanceToTarget = -1.0;
	    		 distanceToTargetByHeight = -1.0;
	    		 strafeToTarget = 0;
	    		 strafeToTargetByHeight = 0;
	    	} //endif on bogus targets
	    	
	    	 //Check for NaN from the angle being too high
	    	 if (Double.isNaN(rotationOffset)){
	    		 if (areaRatio<1){
	    			 rotationOffset=-45.0;
	    		 } 
	    		 else {
	    			 rotationOffset=45.0;
	    		 }
	    	 }
	    	
	    	//Write the information we need on the screen
	    	Imgproc.rectangle(inputMat, new Point(0,0.88*yResolution), new Point(xResolution,yResolution-1), new Scalar(0,0,0), -1);
	    	Imgproc.putText(inputMat, "Distance: " + String.format("%5.1f", distanceToTarget) + " |" + 
	    	String.format("%5.1f", distanceToTargetByHeight) + " Strafe " + String.format("%2.1f", strafeToTarget)+  "|" +
	    			String.format("%2.1f", strafeToTargetByHeight), targetDistTextLocation, 1 , 0.9, targetTextColor, 1);
	    	//Write the angles on the screen
	    	Imgproc.putText(inputMat, "Areas L/R: " + String.format("%.2f",targetAreas[0]) +
		    		"/"+ String.format("%.2f",targetAreas[1]) + " = " + 
		    		String.format("%.2f",areaRatio) +
		    		" Rot: " + String.format("%2.1f",rotationOffset) + "deg", targetAreaTextLocation,1 , 0.9, targetTextColor, 1);
	    	
	    	if(distanceToTarget > 10 ){
	    		Imgproc.putText(inputMat, "Targeted",
	    			targetTextLocation, 1 , 1.0, targetTextColor, 1);
	    	}
	    	else {
	    		Imgproc.putText(inputMat, "STOP!",
	        			targetTextLocation, 1 , 1.0, targetWarningColor, 1);
	    	}
	    	
	    	/*Imgproc.putText(inputMat, "Linear Guess: "+ String.format("%.1f",strafeCorrection*distanceToTarget*targetX[0]) + " / " 
	    	+ String.format("%.1f",strafeCorrection*distanceToTarget*targetX[1]) + " = " + String.format("%.1f",0.5*strafeCorrection*distanceToTarget*(targetX[0]+targetX[1])),
	    			new Point(0,yResolution/4), 1 , 1.0, targetWarningColor, 1);
	    	*/
	    	
	    	Imgproc.line(inputMat, new Point((int)(0.5*xResolution+cameraShift),0.80*yResolution), new Point((int)(0.5*xResolution+cameraShift),0.1*yResolution), new Scalar (200,200,200), 2);
	   }
	    //This is for shooter cam - still a work in progress to get better distances from circumference
	    else {
		    imageTargetHeight = (targetY[1]-targetY[0])/2.0;  // centroid to centroid, 7"
		    imageTargetWidth =  (targetWidths[0]/xResolution);  // 4" guy only for now, but width, not height
		    distanceToTarget = actualTargetWidth / (2.0 * Math.tan(imageTargetWidth*fovRadians/2.0));
	     	distanceToTargetByWidth = actualTargetWidth / (2.0 * Math.tan((imageTargetWidth*fovRadians/2.0)));
	     	//distanceToTargetByHeight = shooterCosCorrection*actualTargetHeight / (2.0 * Math.tan((aspectRatio*imageTargetHeight*fovRadians/2.0)));
	     	//New empirical fit to the y measurement to the top target
	     	distanceToTargetByHeight = 0.0007074*centroidY[0]*centroidY[0] + 0.1202*centroidY[0] + 20.72;
	     	 //I think we just see what fraction of FOV/2 the target is offset and rotate from there
	     	 rotationOffset = 1.0*imageOffsetFromZero*cameraFov/2.0;
	     	
	     	//Write the information we need on the screen
	     	Imgproc.rectangle(inputMat, new Point(0,0.86*yResolution), new Point(xResolution,yResolution-1), new Scalar(0,0,0), -1);
	     	 Imgproc.putText(inputMat, "Distance by target height: " + String.format("%5.1f", distanceToTargetByHeight), targetDistTextLocation,1 , 0.9, targetTextColor, 1);
	     	//Write the angles on the screen
	    	Imgproc.putText(inputMat,"Rotation: " + String.format("%2.1f",rotationOffset) + "deg", targetAreaTextLocation,1 , 0.9, targetTextColor, 1);
	    	
	    		    	
	    	//Scale the distance from 77" to 95" as a good shot; change as necessary
	    	double farShot = 58; double nearShot = 38;
	    	double scaled = Math.max(0, Math.min(100,(distanceToTargetByHeight-nearShot)*(100./(farShot-nearShot))));
	    	double textDistance = 0.01*(int)(yResolution * Math.min(Math.max(10, scaled),90));
	    	//draw a box on the right to put the distance in
	    	Imgproc.rectangle(inputMat, new Point(0.93*xResolution,textDistance-15), new Point(xResolution,textDistance+5), new Scalar(0,0,0), -1);
	    	Color hsvColor = Color.getHSBColor((float)(scaled/100),(float)1.0,(float)1.0);
   	        int rgb = Color.HSBtoRGB((float)(0.33-Math.abs((scaled-50)/150.)), 1, 1);
	        int red = (rgb >> 16) & 0xFF;
	        int green = (rgb >> 8) & 0xFF;
	        int blue = rgb & 0xFF;
	        Scalar distanceColor = new Scalar (rgb & 0xFF, (rgb >> 8) & 0xFF,(rgb >> 16) & 0xFF);
	        Imgproc.putText(inputMat, String.format("%2.0f", distanceToTargetByHeight),
	    			new Point (0.93*xResolution,textDistance), 1 , 1.0, distanceColor, 1);

	    	
	    	if(Math.abs(rotationOffset) < 2.0 && (distanceToTargetByHeight > nearShot) && (distanceToTargetByHeight < farShot) ){
	    		Imgproc.putText(inputMat, "Shoot!",
	    			targetTextLocation, 1 , 1.0, targetTextColor, 1);
	    	
	    			Imgproc.line(inputMat, new Point((int)(0.5*xResolution),0.8*yResolution), new Point((int)(0.5*xResolution),0.2*yResolution), targetTextColor, 1);
	    			Imgproc.rectangle(inputMat, new Point (xResolution/2 - targetWidths[0]/2,((targetY[0]+1))*yResolution/2-targetHeights[0]*yResolution/2), 
	    					new Point (xResolution/2+targetWidths[0]/2,((targetY[0]+1))*yResolution/2+targetHeights[0]*yResolution/2), targetTextColor, 1);
	    	
	    	}
	    	else {
	    		Imgproc.putText(inputMat, "No Shot",
	        			targetTextLocation, 1 , 1.0, targetWarningColor, 1);
	    		Imgproc.line(inputMat, new Point((int)(0.5*xResolution),0.6*yResolution), new Point((int)(0.5*xResolution),0.1*yResolution), targetWarningColor, 2);
	    	}
	    	
	   }
	    


 		/***********************************************/

        
		}// end if on targetcount > 1
		else{
			//Still want to draw some information on the screen even if no targets
			
		    if (bGearCam){
		    	// What do we need here?  Can't do any calculations...
		    	//Put a centerline, at least
		    	Imgproc.line(inputMat, new Point((int)(0.5*xResolution+cameraShift),0.80*yResolution), new Point((int)(0.5*xResolution+cameraShift),0.1*yResolution), new Scalar (0,255,255), 2);
				Imgproc.circle(inputMat, new Point (160 - cameraShift, 137), 14, new Scalar (180,0,180), 2);  
		    }
			    else {
			    	//Centerline
			    	Imgproc.line(inputMat, new Point((int)(0.5*xResolution),0.53*yResolution), new Point((int)(0.5*xResolution),0.37*yResolution), targetWarningColor, 2);
			    	//Imgproc.line(inputMat, new Point((int)(0.5*xResolution),(0.53+0.16)*yResolution), new Point((int)(0.5*xResolution),(0.37+0.12)*yResolution), targetWarningColor, 2);
			    	//Imgproc.line(inputMat, new Point((int)(0.5*xResolution),(0.53+0.16)*yResolution), new Point((int)(0.5*xResolution),0.37*yResolution), targetWarningColor, 2);
			    	// Probably want to put two boxes on this guy to show where it should be
			    	Imgproc.rectangle(inputMat, new Point(0.4*xResolution,0.42*yResolution), new Point(0.6*xResolution,0.50*yResolution), new Scalar(0,0,255), 1);
			    	Imgproc.rectangle(inputMat, new Point(0.4*xResolution,0.58*yResolution), new Point(0.6*xResolution,0.62*yResolution), new Scalar(0,0,255), 1);
			    }
		}

		// Clean up and write out the data to network tables - they are initialized to {0,0} each time so should update nicely now
		  
	    if (bGearCam){
		  table.putNumber("targets",filterContoursOutput.size());
		  table.putNumber("distance", distanceToTarget);
		  table.putNumber("distanceByHeight", distanceToTargetByHeight);
	    	if (Math.abs(rotationOffset)<15) {
	    		table.putNumber("strafe", strafeToTarget);
	    	} else{
	    		table.putNumber("strafe", strafeToTargetByHeight);
	    	}
		  table.putNumber("rotation", rotationOffset);
		  table.putBoolean("connected", true);
	    }
	    else {
	    	// Probably need some special calculations here that only pertain to boiler
	    	table.putNumber("targets",filterContoursOutput.size());
	    	table.putNumber("rotation", rotationOffset);
	    	table.putNumber("distance", distanceToTargetByHeight);
			table.putBoolean("connected", true);
			
	    }

		  
		  endTime=System.nanoTime();

		  Imgproc.putText(inputMat, String.format("FPS: %1.0f Bogeys:%2d",Math.pow(10.0,9)/(double)(endTime-startTime),
	    		filterContoursOutput.size()), infoTextLocation,1, 1.0, infoTextColor, 1);
		  
		  imageSource.putFrame(inputMat);
    
    } // end while loop
  } // end main
  
  //********************************************************************************************
  //Auxiliary functions
  // Had to put this down here.  It's ugly but very useful
	private static void filterContours(List<MatOfPoint> inputContours, double minArea,
			double minPerimeter, double minWidth, double maxWidth, double minHeight, double
			maxHeight, double[] solidity, double maxVertexCount, double minVertexCount, double
			minRatio, double maxRatio, List<MatOfPoint> output) {
			final MatOfInt hull = new MatOfInt();
			output.clear();
			//operation
			for (int i = 0; i < inputContours.size(); i++) {
				final MatOfPoint contour = inputContours.get(i);
				final Rect bb = Imgproc.boundingRect(contour);
				if (bb.width < minWidth || bb.width > maxWidth) continue;
				if (bb.height < minHeight || bb.height > maxHeight) continue;
				final double area = Imgproc.contourArea(contour);
				if (area < minArea) continue;
				if (Imgproc.arcLength(new MatOfPoint2f(contour.toArray()), true) < minPerimeter) continue;
				//I'm not checking on solidity -CJH
				/* 
				Imgproc.convexHull(contour, hull);
				MatOfPoint mopHull = new MatOfPoint();
				mopHull.create((int) hull.size().height, 1, CvType.CV_32SC2);
				for (int j = 0; j < hull.size().height; j++) {
					int index = (int)hull.get(j, 0)[0];
					double[] point = new double[] { contour.get(index, 0)[0], contour.get(index, 0)[1]};
					mopHull.put(j, 0, point);
				}
				final double solid = 100 * area / Imgproc.contourArea(mopHull);
				if (solid < solidity[0] || solid > solidity[1]) continue;
				*/
				if (contour.rows() < minVertexCount || contour.rows() > maxVertexCount)	continue;
				final double ratio = bb.width / (double)bb.height;
				if (ratio < minRatio || ratio > maxRatio) continue;
				output.add(contour);
			}
	}
  
    
  //********************************************************************************************
  
  private static HttpCamera setHttpCamera(String cameraName, MjpegServer server) {
    // Start by grabbing the camera from NetworkTables
    NetworkTable publishingTable = NetworkTable.getTable("CameraPublisher");
    // Wait for robot to connect. Allow this to be attempted indefinitely
    while (true) {
      try {
        if (publishingTable.getSubTables().size() > 0) {
          break;
        }
        Thread.sleep(500);
        } catch (Exception e) {
            // TODO Auto-generated catch block
            e.printStackTrace();
        }
    }


    HttpCamera camera = null;
    if (!publishingTable.containsSubTable(cameraName)) {
      return null;
    }
    ITable cameraTable = publishingTable.getSubTable(cameraName);
    String[] urls = cameraTable.getStringArray("streams", null);
    if (urls == null) {
      return null;
    }
    ArrayList<String> fixedUrls = new ArrayList<String>();
    for (String url : urls) {
      if (url.startsWith("mjpg")) {
        fixedUrls.add(url.split(":", 2)[1]);
      }
    }
    camera = new HttpCamera("CoprocessorCamera", fixedUrls.toArray(new String[0]));
    server.setSource(camera);
    return camera;
  }

  private static UsbCamera setUsbCamera(int cameraId, MjpegServer server) {
    // This gets the image from a USB camera 
    // Usually this will be on device 0, but there are other overloads
    // that can be used
    UsbCamera camera = new UsbCamera("CoprocessorCamera", cameraId);
    server.setSource(camera);
    return camera;
  }
  private static UsbCamera setUsbCamera(int cameraId) {
	    // This gets the image from a USB camera 
	    // Usually this will be on device 0, but there are other overloads
	    // that can be used
	    UsbCamera camera = new UsbCamera("CoprocessorCamera", cameraId);
	    return camera;
	  }
}

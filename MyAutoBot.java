import java.io.IOException;
import java.util.ArrayList;
import java.util.Collection;

import lejos.geom.Point;
import lejos.nxt.LCD;
import lejos.nxt.Motor;
import lejos.nxt.SensorPort;
import lejos.nxt.Sound;
import lejos.nxt.UltrasonicSensor;
import lejos.robotics.RegulatedMotor;
import lejos.robotics.localization.OdometryPoseProvider;
import lejos.robotics.navigation.ArcRotateMoveController;
import lejos.robotics.navigation.DifferentialPilot;
import lejos.robotics.navigation.Navigator;
import lejos.robotics.navigation.Pose;
import lejos.robotics.navigation.Waypoint;
import lejos.robotics.objectdetection.Feature;
import lejos.robotics.objectdetection.FeatureDetector;
import lejos.robotics.objectdetection.FeatureListener;
import lejos.robotics.objectdetection.RangeFeatureDetector;
import lejos.util.PilotProps;
import lejos.util.Stopwatch;

/**
 * Lejos autonomous robot vehicle that is capable of moving around a room
 * independently with the help of obstacle detection events.
 * Has various test routines for testing turns, sensors, paths, navigation, etc.
 * Utilizes mutli-threading for capturing sensor data while moving in an attempt
 * to make navigate around obstacles on its own.
 */
public class MyAutoBot implements FeatureListener
{
  private Navigator nav;
  private ArrayList<Waypoint> waypoints = new ArrayList<Waypoint>();
  private ArcRotateMoveController pilot;
  private OdometryPoseProvider poseProvider;
  private RangeFeatureDetector detector;
  private UltrasonicSensor ultra;
  private Waypoint homeWaypoint;
  private Pose homePose;
  //private RangeFinder rf;
  private Stopwatch sw = new Stopwatch();
  private boolean featureDetected = false;
  private boolean directionChange = false;
  private SensorRotator sensorThread;
  
  MyAutoBot (ArcRotateMoveController aPilot) 
  {    
    pilot = aPilot;
    nav = new Navigator(pilot);
    poseProvider = (OdometryPoseProvider) nav.getPoseProvider();
    //poseProvider = new OdometryPoseProvider(pilot);
    ultra = new UltrasonicSensor(SensorPort.S3);   
    detector = new RangeFeatureDetector(ultra, 25, 100);
    detector.enableDetection(false);
    detector.addListener(this);
    Motor.B.rotateTo(0);
    sensorThread = new SensorRotator();  
  }	

/**
 * Create pilot that will be used to move the robot around a room.
 * DifferentialPilot style will make the robot easier to control in most environments.
 * Wheel diameter and track width in cm.
 * @param args
 */
   public static void main(String[] args) throws IOException, InterruptedException
   {
       PilotProps props = new PilotProps();
       props.loadPersistentValues();
       //flat surface wheels
       //float wheelDiameter = Float.parseFloat(props.getProperty(PilotProps.KEY_WHEELDIAMETER, "43.2")); //CM or MM
       //off road wheels
       float wheelDiameter = Float.parseFloat(props.getProperty(PilotProps.KEY_WHEELDIAMETER, "68.7")); //CM or MM
       float trackWidth = Float.parseFloat(props.getProperty(PilotProps.KEY_TRACKWIDTH, "140")); //CM or MM
       RegulatedMotor leftMotor =  PilotProps.getMotor(props.getProperty(PilotProps.KEY_LEFTMOTOR, "A"));
       RegulatedMotor rightMotor = PilotProps.getMotor(props.getProperty(PilotProps.KEY_RIGHTMOTOR, "C"));
    	 
       DifferentialPilot pilot = new DifferentialPilot(wheelDiameter, trackWidth, leftMotor, rightMotor);
        
       MyAutoBot robot = new MyAutoBot(pilot);
       //robot.testSimpleSensors();
       //robot.testSensorTurns();
       //robot.testSensorThread();
       //robot.testOutAndBack();   
       //robot.testPilotArcTurn();
       //robot.testOnOffSensor();
       //robot.testWaypointPathHeading();
       robot.testRoam();   
   }

   public void testSimpleSensors() throws InterruptedException
   {
      LCD.drawString("Simple Sensor Test", 0, 0);
      //LCD.drawString("distance: " + ultra.getDistance(), 0, 2);
          
      while (true)
      {
 		 //Rotate LEFT
 		 Motor.B.rotateTo(90);
 		 int leftDistance = ultra.getDistance();
         int tachoCnt = Motor.B.getTachoCount();
 		 LCD.clear();
 	     LCD.drawString("leftDistance: ", 0, 0);
 	     LCD.drawString("tachoCnt: ", 0, 1);
 		 LCD.drawInt(leftDistance, 0, 2);
 		 LCD.drawInt(tachoCnt, 0, 3);
 		 
	     Thread.sleep(1000);
 		 
 		 //Rotate RIGHT
 		 Motor.B.rotateTo(-90);
 		 int rightDistance = ultra.getDistance();
         tachoCnt = Motor.B.getTachoCount();
 		 LCD.clear();
 	     LCD.drawString("rightDistance: ", 0, 0);
 	     LCD.drawString("tachoCnt: ", 0, 1);
 		 LCD.drawInt(rightDistance, 0, 2);
 		 LCD.drawInt(tachoCnt, 0, 3); 		 
 	     
	     Thread.sleep(1000);
      }
    	  
   }

   public void testSensorTurns() throws InterruptedException
   {
      LCD.drawString("Simple Sensor Test", 0, 0);
      //LCD.drawString("distance: " + ultra.getDistance(), 0, 2);
          
      pilot.setTravelSpeed(250);
      pilot.forward();
      Thread.sleep(3000);
      
      while (true)
      {
 		 //Rotate LEFT
 		 Motor.B.rotateTo(90);
 		 int leftDistance = ultra.getDistance();
         int tachoCnt = Motor.B.getTachoCount();
 		 LCD.clear();
 	     LCD.drawString("leftDistance: ", 0, 0);
 	     LCD.drawString("tachoCnt: ", 0, 1);
 		 LCD.drawInt(leftDistance, 0, 2);
 		 LCD.drawInt(tachoCnt, 0, 3);
 		 
	     if (tachoCnt > 10)
	     {   
    	   Motor.A.setSpeed(Motor.A.getSpeed() + 200);
    	   Motor.C.setSpeed(250);
	     }

         Thread.sleep(3000);
 		 
 		 //Rotate RIGHT
 		 Motor.B.rotateTo(-90);
 		 int rightDistance = ultra.getDistance();
         tachoCnt = Motor.B.getTachoCount();
 		 LCD.clear();
 	     LCD.drawString("rightDistance: ", 0, 0);
 	     LCD.drawString("tachoCnt: ", 0, 1);
 		 LCD.drawInt(rightDistance, 0, 2);
 		 LCD.drawInt(tachoCnt, 0, 3); 		 
 	     
	     if (tachoCnt < -10) {
	    	 Motor.C.setSpeed(Motor.C.getSpeed() + 200);
	    	 Motor.A.setSpeed(250);
		 }

	     Thread.sleep(3000);
      }
    	  
   }

   /**
    * Test object detection while scanning.
    * Logic to be used when pilot is moving to learn about surroundings.
    */
   public void testSensorThread() throws InterruptedException
   {
       LCD.drawString("Sensor Thread Test", 0, 0);          
       SensorRotator sensorThread = new SensorRotator();
       sensorThread.start();
	          
       while (true)
       {
           int distance = ultra.getDistance();
           int tachoCnt = Motor.B.getTachoCount();
           LCD.clear();
	       LCD.drawString("distance: " + ((int)distance), 0, 0);
	       LCD.drawString("tach: " + ((int)tachoCnt), 0, 1);
	       
	       // want to react if we sense an object on the side while
	       // the pilot is moving forward, but not stop the pilot (yet)
	       
	       // if we are scanning the side
	       if (tachoCnt > 10 || tachoCnt < -10) {
	    	   // side of bot is close to obstacle
		       if (distance < 25) {
		           LCD.drawString("side obstacle", 0, 2);
		       }
	       }
	       else {
	    	   // detected feature in front of bot, so stop now and assess
	           LCD.drawString("front obstacle", 0, 3);
	       }
       }
       
   }

   public void testOnOffSensor() throws InterruptedException
   {
       LCD.drawString("Sensor On/Off Test", 0, 0);          
       SensorRotator sensorThread = new SensorRotator();
       sensorThread.start();
       
       while (true) {
	       Thread.sleep(4000);
	       sensorThread.setRunning(false);
	       Thread.sleep(10000);
	       sensorThread.setRunning(true);       
       }
       
   }

   /**
    * Basic navigation test with Differential Pilot
    */
   public void testOutAndBack() throws InterruptedException
   {
      LCD.drawString("Out and Back test", 0, 0);
      pilot.setTravelSpeed(250);
      //pilot.travel(100, true);
      pilot.forward();
      
      //Thread.sleep(3000);
      
      while (true)
      {
    	  
    	  if (pilot.isMoving()) {
			  LCD.clear();
		      LCD.drawString("Motor.A.getSpeed(): " + (int)Motor.A.getSpeed(), 0, 1);
		      LCD.drawString("Motor.C.getSpeed(): " + (int)Motor.C.getSpeed(), 0, 2);
		      LCD.drawInt(Motor.A.getSpeed(), 0, 3);
		      LCD.drawInt(Motor.C.getSpeed(), 0, 4);
    	  }
    	  
    	  
	      // if it gets too close to something, backup
		  if (ultra.getDistance() <= 40) 
		  {
		      pilot.stop();
			  LCD.clear();
		      LCD.drawString("Obstacle detected at: ", 0, 1);
		      LCD.drawString(ultra.getDistance() + " cm ", 0, 2);
	          //Motor.A.stop();
	          //Motor.C.stop();
	
		      // rotate ultra sensor to each side
		      Motor.B.rotateTo(-100);
			  Motor.B.rotateTo(100);
			  Motor.B.rotateTo(0);

			  pilot.rotate(180);
		      Thread.sleep(1000);
	
		      pilot.travel(1000);
		      pilot.stop();
	
			  break;
		  }
      }
    	  
      LCD.drawString("Pilot STOP. ", 0, 3);
      pilot.stop();
   }
  
   public void testPilotArcTurn() throws InterruptedException
   {
      LCD.drawString("Navigate bot!", 0, 0);
      LCD.drawString("distance: " + ultra.getDistance(), 0, 2);
      pilot.setTravelSpeed(250);
      pilot.forward();
      Thread.sleep(3000);

      //LCD.drawString("side obstacle", 0, 2);
      //LCD.drawString("tacho" + (int)tachoCnt, 0, 3);
      //double radius = tachoCnt * (Math.PI / 180);
      //LCD.drawString("radius" + (double)radius, 0, 4);
      //pilot.arc(radius, tachoCnt, true);
      
      Thread.sleep(3000);

      LCD.drawString("Pilot STOP. ", 0, 3);
      pilot.stop();
   }

   /**
    * Use the Pose to figure out where the pilot is and was.
	* Pose:
	* Represents the location and heading(direction angle) of a robot.
	* All directions and angles are in degrees and use the standard convention in mathematics: 
	* direction 0 is parallel to the X axis, and direction +90 is parallel to the Y axis. 
    */
   public void testWaypointPathHeading() throws InterruptedException
   {
      pilot.setTravelSpeed(200);
      Thread.sleep(3000);
      pilot.forward();

      int ctr=0;
      // just drive forward and create some waypoints
      while (true)
      {
	      Pose pose = poseProvider.getPose();
	      float heading = pose.getHeading();
	      LCD.drawString("heading: " + heading, 0, ctr);
	      waypoints.add(new Waypoint(pose));
	      nav.addWaypoint(new Waypoint(pose));
	      Thread.sleep(1200);

	      // half way thru, make a left turn 
	      if (waypoints.size() == 6) {
	    	  //Motor.C.setSpeed(500);
	    	  pilot.stop();
	    	  pilot.rotate(90);
	    	  Thread.sleep(3000);
	    	  pilot.forward();
	      }

	      // stop driving when we have enough waypoints to create a path
	      if (waypoints.size() == 12) {
	    	  Sound.beep();
	    	  break;
	      }
	      ctr++;
      }

      pilot.stop();
      // allow time to look at the heading (direction angle) of each Pose (Waypoint)
      Thread.sleep(3000);

      ctr=0;
      LCD.clear();
      for (Waypoint pt :  waypoints) {
	      LCD.drawString("X: " +(int)pt.x + " " + "Y: " +(int)pt.y, 0, ctr);
	      ctr++;
      }

      // allow time to look at each x,y point along the path
      Thread.sleep(3000);
      
      // now, use the waypoints to go back to the start
      nav.followPath();
      nav.waitForStop();
      //goHome();
   }

   /**
    * Use the Pose and and last feature Pose and motor tacho counts to
    * see if the pilot is trying to move but is stuck.
    * Need to check if Pose.X is changing while the wheels are moving
	* Pose:
	* Represents the location and heading(direction angle) of a robot.
	* All directions and angles are in degrees and use the standard convention in mathematics: 
	* direction 0 is parallel to the X axis, and direction +90 is parallel to the Y axis. 
    */
   public void testPilotStuckDetection() throws InterruptedException
   {
      pilot.setTravelSpeed(150);
      Thread.sleep(3000);
      pilot.forward();

      int ctr=0;
      // just drive forward and create some waypoints
      while (true)
      {
	      Pose pose = poseProvider.getPose();
	      float heading = pose.getHeading();
	      LCD.drawString("heading: " + heading, 0, ctr);
	      waypoints.add(new Waypoint(pose));
	      Thread.sleep(1200);
	      
		  //TODO Check current waypoint vs previous waypoint and engine tach count
		  //to see if pilot has made progress since last waypoint, and is not stopped
      }

      pilot.stop();
      // allow time to look at the heading (direction angle) of each Pose (Waypoint)
      Thread.sleep(8000);
   }

   /*
    * Roam around a room with obstacle detection and setting waypoints.
	* This test will use a separate thread for object detection that will trigger
	* events that will stop the pilot so we can evaluate the path.
    */
   public void testRoam() throws InterruptedException
   {
	  final int PILOT_SPEED = 260;
      LCD.drawString("Roam Test:", 0, 1);
      pilot.setTravelSpeed(PILOT_SPEED);
      // distance * 10 to account for wheel diameter
      //pilot.travel(ultra.getDistance()*10, true);
	  Motor.B.rotateTo(0);
  	  Thread.sleep(3000);
      pilot.forward();
      detector.enableDetection(true);
      
      // Set starting (home) waypoint
      Pose pose = poseProvider.getPose();
      homePose = pose;
      homeWaypoint = new Waypoint(homePose);
      waypoints.add(homeWaypoint);
      nav.addWaypoint(homeWaypoint);

      // Start Thread to initiate sensor scanning
      sensorThread.start();

      // start traveling until an obstacle is detected
  	  boolean move = true;
      while (move)
      {
    	  int distance = ultra.getDistance();
    	  
    	  // we're coming up on a feature in front of us, so
    	  // start to slow down before we actually detect it and stop
    	  // stopping point is 25cm - slow down point is 50cm
    	  if (pilot.isMoving() && !featureDetected) 
    	  {
    		  // make sure the feature is in front of us
   	          int tachoCnt = Motor.B.getTachoCount();
    		  if (tachoCnt < 10 && tachoCnt > -10)
    		  {
	    		  if (distance < 50) {    			  
	    		      //pilot.setTravelSpeed(150);
	    		  } 
    		  }
    	  }
    	  
    	  // a direction change occurred because a feature was
    	  // detected in front of us - so now resume forward path
    	  if (directionChange) 
    	  {        
	          // reset normal travel speed
		      pilot.setTravelSpeed(PILOT_SPEED);
	          
	          pilot.forward();

	          directionChange = false;	          

		      // If we've encountered X number of features, go home
		      /*
        	  if (waypoints.size() == 10) { 
	            pilot.travel(ultra.getDistance()*10, true);
          		goHome();
        		//goHome(poseProvider.getPose());
        		move = false;
        	  }
        	  */
          }
      }
   }

    /* 
     * Change direction of pilot.
	 * If angle is positive, the robot turns to the left. The immediateReturn parameter works as in the Motor methods 
	 * –allowing the calling thread to do other work while the rotation task in progress. If another method is 
	 * called on the pilot while the rotation is in progress, the rotation will be terminated. 
	 * You must have accurate values for wheelDiameter and trackWidth for this method to produce accurate results.
     */
    private void computeNewDirection() throws InterruptedException
    {
    	// Need to reset sensor for accurate reading
    	// TODO: do we still need to do this?!
		ultra.reset();
		ultra.continuous();

		//Rotate LEFT
		Motor.B.rotateTo(90);
		Thread.sleep(500);
		int leftDistance = ultra.getDistance();

		//Rotate RIGHT
		Motor.B.rotateTo(-90);
		Thread.sleep(500);
		int rightDistance = ultra.getDistance();
		   
		LCD.clear();
	    LCD.drawString("leftDist: ", 0, 0);
	    LCD.drawString("rightDist: ", 0, 1);
	    LCD.drawInt(leftDistance, 0, 2);
	    LCD.drawInt(rightDistance, 0, 3);

	    //Thread.sleep(5000);
	    
		// TODO - find out exact distances to make better decision
	    // Positive angle rotate left (anti-clockwise), negative right.
	    // Check that we have an option to turn right or left,
	    // otherwise we'll just need to backup
		if (rightDistance > 20 || leftDistance > 20)
		{
			if (rightDistance > leftDistance) {
			    //Turn RIGHT
		        pilot.rotate(-90);
			}
			else if (leftDistance > rightDistance) {
				//Turn LEFT
		        pilot.rotate(90);
			}
			else {
			    //side distances are equal, doesn't matter
				//Turn LEFT
		        pilot.rotate(90);
		    }
		}
	    else {
			 // nowhere to turn, backup and reassess
	         pilot.travel(-250);
	         pilot.stop();
	 	     Motor.B.rotateTo(0, true);
	     	 leftDistance = 0;
	    	 rightDistance = 0;
	         //recompute
	         computeNewDirection();
		 }
   }
    
    /**
     * Stub method for testing distance sensor and obstacle detection
     */
    public void featureDetectedTEST(Feature feature, FeatureDetector sonar )
    {
    	Sound.beep();
    }
    
    /**
     * Feature listener implementation.
     * When feature is detected within 30cm (detector.getMaxDistance()), 
     * figure out why and stop/turn the pilot.
     * Find new angle and restart navigation.
     */
    public synchronized void featureDetected(Feature feature, FeatureDetector sonar)
    {
       Sound.beep();
       int tachoCnt = Motor.B.getTachoCount();
       
	   try 
	   {
	       featureDetected = true;

	       // pause sensor rotating thread
	       sensorThread.setRunning(false);
		   //Motor.B.rotateTo(0);
		   
	       //PilotProps.KEY_LEFTMOTOR,  "A" :  90 tacho
	       //PilotProps.KEY_RIGHTMOTOR, "C" : -90 tacho
	       LCD.clear();
           
	       // want to react if we sense an object on the side while
	       // the pilot is moving forward, but not stop the pilot (yet)
               // will need to check heading too - might need to turn away
               // from an obstacle, or could maintain direction if safe
	       if (tachoCnt > 30)
	       {   
	           //Increase LEFT motor speed
	    	   Motor.A.setSpeed(Motor.A.getSpeed() + 50);
	    	   Motor.C.setSpeed(Motor.C.getSpeed() - 25);
	    	   
	    	   Thread.sleep(1000);
	           pilot.forward();
	       }
	       else if (tachoCnt < -30) {
	           //Increase RIGHT motor speed
	    	   Motor.C.setSpeed(Motor.C.getSpeed() + 50);
	    	   Motor.A.setSpeed(Motor.A.getSpeed() - 25);
	    	   
	    	   Thread.sleep(1000);
	           pilot.forward();
	       }
	       else {
		       // detected feature in front of bot, so stop now
	  	       pilot.stop();
	  		   Thread.sleep(300);
	  	       // back up a bit
	  	       pilot.travel(-120);		       
	  		   Thread.sleep(300);

	  			// get coords, add waypoint
			   reportFeature(feature);    

			   // figure out how to turn around
			   computeNewDirection();
			 
			   // reset ultra sensor to 0 heading
			   Motor.B.rotateTo(0, true);
			   //Thread.sleep(1000);
			   
			   // tell main pilot loop to continue in new direction
			   directionChange = true;
	       }

	       // resume sensor rotating thread
	       sensorThread.setRunning(true);
	       featureDetected = false;
	   }
	   catch (Exception e) {
	      LCD.clear();
	      LCD.drawString("Exception! " + e.getMessage(), 0, 0);
	   }
   }

   /**
    * Calculate location of pilot and add as Waypoint.
    */
   private void reportFeature(Feature feature) throws InterruptedException
   {
       LCD.clear();
       float distance = feature.getRangeReading().getRange();
       float angle = feature.getRangeReading().getAngle();
       LCD.drawString("Feature Detected!", 0, 0);
       LCD.drawString("dist / angle: "+ distance+" "+ angle, 0, 1);
       Pose pose = poseProvider.getPose();
       LCD.drawString("Pose: "+ pose.getX()+" "+ pose.getY()+" "+ pose.getHeading(), 0, 2);
       Point obstacle = pose.pointAt(distance, angle+pose.getHeading());
       LCD.drawString("xy: "+ obstacle.x+" "+ obstacle.y, 0, 3);

       Waypoint featurePoint = new Waypoint(pose);
       nav.addWaypoint(featurePoint);
       waypoints.add(featurePoint);
   }
   
   /**
    * Go home via waypoint path
    */
   private void goHome()
   {
	   try {
	     Collection<Waypoint> route = waypoints;	     
	     LCD.clear();
	     int ctr = 0;
	     for(Waypoint wp: route) 
	     {
	       LCD.drawString("Go to (" + (int) wp.x + "," + (int) wp.y + ")", 0, ctr);
	       Pose pose = wp.getPose();
	       
	       // rotate the robot to the correct angle to start it's way home
	       pilot.rotate(wp.getHeading() - pose.getHeading());
	       
	       // (1) travel to waypoint
           //float distance = pose.distanceTo(wp.getPose().getLocation());
           //pilot.travel(distance);

	       // (2) travel to waypoint
           // equivalent to add(waypoint); followPath(); 
	       nav.goTo(wp);
	       ctr++;
	     }	     
	   } 
	   catch (Exception e) {
		   LCD.clear();
		   LCD.drawString("goHome()", 0, 0);
		   LCD.drawString("Exception:", 0, 1);
		   LCD.drawString(e.getMessage(), 0, 2);
	   }
	   
    }

   /**
    * Go home via destination point
    */
    private void goHome(Pose currentPose)
    {
	   // Go home via destination waypoint (angle to destination)
       float angle = currentPose.angleTo(homeWaypoint);
       pilot.rotate(angle - currentPose.getHeading());
       pilot.travel(currentPose.distanceTo(homeWaypoint)*10);
       LCD.clear();
       LCD.drawString("Go Home to (" + homeWaypoint.x + "," + homeWaypoint.y + ")", 0, 1);
	}
}

/**
 * Sensor Thread to control sensor scanning of pilot path.
 */
class SensorRotator extends Thread {
	
	private boolean running;
	public SensorRotator() {
		running = true;
	}
	public void setRunning(boolean running) {
		this.running = running;
	}
	
	@Override
	public void run() 
	{
		try {
			Motor.B.setSpeed(900);
			//while (!this.isInterrupted()) 
			while (true) 
			{
				// pauses thread operation
				if (!this.running) {
					continue;
				}
				
				Motor.B.rotateTo(-80);
				Thread.sleep(200);
				Motor.B.rotateTo(0);
				Thread.sleep(200);
				Motor.B.rotateTo(80);
				Thread.sleep(200);
				Motor.B.rotateTo(0);
				Thread.sleep(200);
			}
		} 
		catch (InterruptedException e) {
		    this.running = false;
		}

	}
}
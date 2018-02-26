/**
 * This class drives the robot around.
 * It is the only class that has access to the motors.
 * 
 */

package ca.mcgill.ecse211.lab5;

import java.util.Arrays;
import org.jfree.util.Rotation;
import org.opencv.core.RotatedRect;
import ca.mcgill.ecse211.odometer.Odometer;
import ca.mcgill.ecse211.odometer.OdometerExceptions;
import lejos.hardware.Sound;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.robotics.SampleProvider;

public class Navigation implements UltrasonicController{
 //Constants that are intrinsic to the robot.
  private static final int minDist = 10;        //Distance at which the robot will stand from the wall.
  private static final double SQUARESIDE = 30.48; //length of a square in cm. 
  private static final double DISTANCE_FROM_US_TO_CENTER = 10;
  private final int FILTER_OUT = 20;
  private static boolean robotIsNavigating = false;    //Boolean that will notify rest of code that the robot is moving.
  private Odometer odo;                         // Need odometer to get present location values.
  private static EV3LargeRegulatedMotor leftMotor;
  private static EV3LargeRegulatedMotor rightMotor;
  private static double wheelRadius;
  public static double track;
  public static int FORWARDSPEED;
  public static int ROTATIONSPEED;
  public static double wantedTheta;
  public static double deltaX;
  public static double deltaY;
  public static int distance = 255;
  private static double[][] centerOfBoard;
  private int[] last5Distances = new int[5];
  public static boolean navigationCorrectionEnable = false;
  public static boolean odometerCorrected;
  private static boolean robotIsRotating = false;
  private double CONSTANT_FLAG_CHECKING_DISTANCE = 12;
 
  
  /**
   * Constructor of Navigation.
   * @author Shawn Vosburg
   * @author Tianhang Wu
   * @param leftMotor 
   * @param rightMotor
   * @param wheelRadius Assumes both wheels have the same radius. This value must be in cm.
   * @param track       Distance between the center of the two wheels in cm.
   * @param forwardSpeed    Speed of straight motion in deg/s.   
   * @param rotationSpeed   Speed of rotation in deg/s
   * @param sizeOfBoard     Number of intersection on board on one side. Ex: A 3by3 board will have a sizeOfBoard value of 3.
   * @throws OdometerExceptions
   */
  public Navigation(EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor,
      double wheelRadius, double track, int forwardSpeed, int rotationSpeed, int sizeOfBoard) throws OdometerExceptions{
    this.odo = Odometer.getOdometer();              //give access to present odometer.
    
    // reset the motors
    for (EV3LargeRegulatedMotor motor : new EV3LargeRegulatedMotor[] {leftMotor, rightMotor}) {
      motor.stop();
      motor.setAcceleration(3000);
    }

    //Set the constants
    this.leftMotor = leftMotor;
    this.rightMotor = rightMotor;
    this.wheelRadius = wheelRadius;
    this.track = track;
    this.FORWARDSPEED = forwardSpeed;
    this.ROTATIONSPEED = rotationSpeed;
    
    //Determine where is the center of the board.
    this.centerOfBoard = new double[1][2];
    this.centerOfBoard[0][0]= this.centerOfBoard[0][1] = (sizeOfBoard-1)/2;
  }
  
  /**This function turns toward the inputed x and y coordinate and travels towards that location while checking of obstacles.
   * 
   * @param finalX  X-coordinate of the destination following laboratory convention.
   * @param finalY  Y-coordinate of the destination following laboratory convention.
   */
  public void travelTo (double finalX, double finalY) {
    //setting booleans to distinguish between the robot rotating and the robot navigating. Useful for odometry correction.
    robotIsNavigating = true;
    robotIsRotating = false;
    stopMotors(); 
    
    //Fetching constants
    double presentX = odo.getXYT()[0];
    double presentY = odo.getXYT()[1];
    
    //Calculating the theta required to turn to.
    deltaX = finalX * SQUARESIDE - presentX;
    deltaY = finalY * SQUARESIDE - presentY;
    
    //If deltaY is positive, then the range of theta is -90deg to 90 deg.
    if(deltaY >= 0) {
      wantedTheta = Math.atan(deltaX/deltaY)*180 / Math.PI;
    }
    //If deltaY is negative, then must add 180deg to result
    else {
      wantedTheta = Math.atan(deltaX/deltaY)*180 / Math.PI + 180;
    }
    
    //Turn robot to the desired Theta and set back the booleans.
    
    turnTo(wantedTheta,true,false);
    robotIsNavigating = true;
    robotIsRotating = false;
    
    //Move to the desired point.
    double distanceToTravel = Math.sqrt(Math.pow(deltaX, 2) + Math.pow(deltaY, 2));
    
    leftMotor.setSpeed(FORWARDSPEED);
    rightMotor.setSpeed(FORWARDSPEED);
             
    leftMotor.rotate(convertDistance(wheelRadius, distanceToTravel), true);
    rightMotor.rotate(convertDistance(wheelRadius, distanceToTravel), true);
    
    while(isMoving()) {
      
      //Verifies that the robot can re-adjust its trajectory and does so if it is allowed.
      if(odometerCorrected && navigationCorrectionEnable ) {
        odometerCorrected = false;
        stopMotors();
        travelTo(finalX, finalY);
        stopMotors();
        return;
      }      
    }
 
  }
  
  

  /**This function turns to the theta inputted, independently from the theta it is at currently. It always rotate with the minimal angle (The shortest rotation possible).
   * 
   * @param theta   Desired theta to be rotated at in degrees.
   * @param calledByAnotherNavigatingFunction   True is this function was called by another function that has set the boolean robotIsNavigating to true.
   * @param instantReturn      True if the user want the function to return instantly. False and the function will return once the rotation is complete.
   */
  public void turnTo(double theta, boolean calledByAnotherNavigatingFunction, boolean instantReturn) {
    //Booleans that determines if the robot is in rotating mode or moving forward mode.
    robotIsRotating = true;
    robotIsNavigating = false;
    
    stopMotors();
    
    //Go get what direction the robot is facing.
    double presentTheta = odo.getXYT()[2];
    
    //Calculate the difference in angle.
    double deltaTheta = theta - presentTheta;
   
    //make sure the angle is positive for following logic to work
    if(deltaTheta <0) deltaTheta = 360 + deltaTheta;
    
    //If the angle obtained is the minimal one, rotate in increasing theta direction. 
    if(deltaTheta <= 180) {
      leftMotor.setSpeed(ROTATIONSPEED);
      rightMotor.setSpeed(ROTATIONSPEED);
      
      leftMotor.rotate(convertAngle(wheelRadius, track, deltaTheta),true);
      rightMotor.rotate(-convertAngle(wheelRadius, track, deltaTheta), instantReturn);
      
    }
    //The angle obtain must therefore be the maximal angle. Turn the other way with the minimal angle.
    else {
      //Find minimal angle
      deltaTheta = 360 - deltaTheta;
      
      leftMotor.setSpeed(ROTATIONSPEED);
      rightMotor.setSpeed(ROTATIONSPEED);
      
      leftMotor.rotate(-convertAngle(wheelRadius, track, deltaTheta),true);
      rightMotor.rotate(convertAngle(wheelRadius, track, deltaTheta), instantReturn);
   
    }
  }
  
  /**Function that allows other classes access to the variable isNavigating.
   * 
   * @return True if the robot is navigating. False if it is not.
   */
  public static boolean isNavigating() {
    return (robotIsNavigating);
    
  }
  /**
   * Function that returns the boolean robotIsRotating. Useful for OdometryCorrection.  
   * @return True if the robot is presently rotating. False otherwise.
   */
  public static boolean isRotating() {
    return robotIsRotating;
  }
  
  /**Function that calculates how many degrees the wheels must rotate to cover a particular distance.
   * 
   * @param radius      Radius of the wheel in cm.
   * @param distance    Distance to be covered in cm.
   * @return            Returns the amount of degrees the wheel must turn to in order to cover the inputted distance.
   */
  private static int convertDistance(double radius, double distance) {
    return (int) ((180.0 * distance) / (Math.PI * radius));
  }

  /**Function that calculates the amount of degrees of rotation needed to turn a particular degree.
   * 
   * @param radius  Radius of the wheels in cm.
   * @param width   Largeness of the robot in cm. Also known as the Track value.
   * @param angle   Desired amount of angle to be turned in deg.
   * @return        Returns the amount of degree the wheels must turn in order for the robot to rotate the inputted amount of degrees.
   */
  private static int convertAngle(double radius, double width, double angle) {
    return convertDistance(radius, Math.PI * width * angle / 360.0);
  }

  /**
   * Function that is called by the Ultrasonic poller to update the distance the ultasonic sensor has detected.
   */
  @Override
  public void processUSData(int distance) {
    int filterControl=0;
    // rudimentary filter - toss out invalid samples corresponding to null
    // signal.
    // (n.b. this was not included in the Bang-bang controller, but easily
    // could have).
    //
    if (distance >= 255 && filterControl < FILTER_OUT) {
      // bad value, do not set the distance var, however do increment the
      // filter value
      filterControl++;
    } else if (distance >= 255) {
      // We have repeated large values, so there must actually be nothing
      // there: leave the distance alone
      this.distance = distance;
    } else {
      // distance went below 255: reset filter and leave
      // distance alone.
      filterControl = 0;
      this.distance = distance;
    }
    Navigation.distance = distance;
    
  }

  /**
   * Function that can be accesses by any class that returns the distance read by the ultrasonic sensor.
   */
  @Override
  public int readUSDistance() {
    // TODO Auto-generated method stub
    return Navigation.distance;
  }
  
  /**When a wall is detected, the robot enters obstacle avoidance mode. 
   * 
   * @param finalX      Final desired destination. This is the x-Coordinate of the desired point.
   * @param finalY      Final desired destination. This is the y-Coordinate of the desired point.
   * @param turnRight   Boolean that determines if the robot must evade the wall ahead by going right of it or left of it. Used primairly so that the robot does not fall off the platform.
   */
  private void obstacleAvoidance(double finalX, double finalY,boolean turnRight) {
    //The robot always moves in 90deg. Turing right by convention
    //means that the robot must increase theta by positive 90 degrees.
    //turning left means that the robot my decrease theta by negative 90 degrees.
    int deltaT = 90;
    if(!turnRight) deltaT= -90;
    
    //Turns, travels 10 cm and the checks if the block is still there. Repeat until no block is detected no more. Finish by turning back in the same direction it started with.
    //returns if when travelling the 10cm, it sees another block.
    if(turnMoveAndCheck(deltaT, 10, true)) return;
    
    //If there is no block ahead
    if(!isThereBlockAhead()) {
      
      //Move an extra 20cm just to be sure to clear the wall.
      //returns if when travelling the 20cm, it sees another block.
      if(turnMoveAndCheck(deltaT, 20, false)) return;
      
      //Move along the wall 30 cm and check if the wall is still there.
      //returns if when travelling the 30cm, it sees another block.
      if(turnMoveAndCheck(-deltaT, 30, false)) return;
      turnTo(odo.getXYT()[2]-deltaT,true,false);
     
      //If the wall is still there, move 10cm until it is not seen anymore.
      //returns if when travelling the 10cm, it sees another block.
      while(isThereBlockAhead()) {
        if(turnMoveAndCheck(deltaT, 10, true)) return;
      }
      
      //When no more wall is seen, it means that it is now clear. 
      //move 10 cm more to be sure and then travel to the wanted destination. 
      //returns if when travelling the 10cm, it sees another block.
      if(!isThereBlockAhead()) {
        if(turnMoveAndCheck(deltaT, 10, true)) return;
        travelTo(finalX,finalY);
      }
    }
  }
  
  /**Function that detects if an upcoming wall is too close.
   * 
   * @return True if a detected wall is under the minimalDistance. False if there is no wall that is too close to the robot. 
   * 
   */
  public boolean isThereBlockAhead() {
    if(distance < minDist) return true;
    else return false;
  }
  
  /**Function that turns the robot, moves a required distance, and optionally turns back to the original theta.
   * 
   * @param deltaT              The amount of degrees that the robot will turn with.
   * @param distanceToTravel    The amount of distance that the robot will move along in cm.
   * @param endWithTurnBack     True if the robot will return to the original theta.
   * @return                    Returns true if another wall is seen when travelling. False otherwise
   */
  public boolean turnMoveAndCheck(int deltaT,int distanceToTravel, boolean endWithTurnBack) {
    // Turns the amount of degrees inputted.
    turnTo(odo.getXYT()[2]+deltaT, true,false);
    
    //Covers the amount of distance inputted.
    leftMotor.setSpeed(FORWARDSPEED);
    rightMotor.setSpeed(FORWARDSPEED);
             
    leftMotor.rotate(convertDistance(wheelRadius, distanceToTravel), true);
    rightMotor.rotate(convertDistance(wheelRadius, distanceToTravel), true);
    
    //checks if upcoming wall when moving. Returns true if there is such a wall.
    while(leftMotor.isMoving() && rightMotor.isMoving()) {
      if(isThereBlockAhead()) return true;
    }
    
    //turns back if user wants the robot to do so.
    if(endWithTurnBack) turnTo(odo.getXYT()[2]-deltaT, true,false);
    
    //If no wall is seen during this operation, return false.
    return false;
    
    
  }
  
  /**This function calculates if the robot must turn towards the right when avoiding a wall or left. This function is mostly so that the robot does not fall off the platform.
   * 
   * @return True if the robot must evade towards the right. False if the robot must evade towards the left.
   */
  private boolean turnRightWhenAvoiding() {
    double presentX = odo.getXYT()[0];
    double presentY = odo.getXYT()[1];
    double presentT = odo.getXYT()[2];
    double deltaT;
    
    //Calculating the theta required to turn to.
    deltaX = centerOfBoard[0][0] * SQUARESIDE - presentX;
    deltaY = centerOfBoard[0][1] * SQUARESIDE - presentY;
    
    //If deltaY is positive, then the range of theta is -90deg to 90 deg.
    if(deltaY >= 0) {
      wantedTheta = Math.atan(deltaX/deltaY)*180 / Math.PI;
    }
    //If deltaY is negative, then must add 180deg to result
    else {
      wantedTheta = Math.atan(deltaX/deltaY)*180 / Math.PI + 180;
    }
    
    //Calculates the change in theta required.
    deltaT = wantedTheta - presentT;
    
    //If theta is negative, must make it positive. Ex: turns -90deg into 270deg. By convention to keep logic constant.
    if(deltaT <0) deltaT = deltaT +360;
    
    //Returns true if it is faster to turn right. 
    //returns false if it is faster to turn left.
    if(deltaT < 180) return true;
    else return false;
    
  }
  
  /**Stops both motors by setting their speed to 0.
   * 
   */
  public void stopMotors() {
    leftMotor.setSpeed(0);
    rightMotor.setSpeed(0);
  }
  
  /**This function rotates the robot a desired number of degrees.  
   * 
   * @param deltaTheta The amount of degrees that the robot must rotate by.
   * @param instantReturn   True if the function is to be returned immediately.
   * @param rotateClockwise     True if rotate clockwise. False if rotate counterclockwise
   */
  public static void rotateRobot(double deltaTheta, boolean instantReturn, boolean rotateClockwise) {
    double absoluteValueOfDeltaTheta = Math.abs(deltaTheta);
    robotIsRotating = true;
    robotIsNavigating = false;
    int rotateDirection = -1;
    if(rotateClockwise) rotateDirection = 1;
    
    leftMotor.setSpeed(ROTATIONSPEED);
    rightMotor.setSpeed(ROTATIONSPEED);
    
    leftMotor.rotate(rotateDirection*convertAngle(wheelRadius, track, absoluteValueOfDeltaTheta),true);
    rightMotor.rotate(-rotateDirection*convertAngle(wheelRadius, track, absoluteValueOfDeltaTheta), instantReturn);
    
  }
  
 /**Advances the robot a desired amount of cm.
  * 
  * @param distanceToTravel Distance to travel in cm.
  * @param instantReturn    True if the function is to be instantly returned. False if the function is to be returned after the travel is completed.
  */
  public static void advanceRobot(double distanceToTravel, boolean instantReturn) {
    robotIsNavigating = true;
    robotIsRotating = false;
    
    leftMotor.setSpeed(FORWARDSPEED);
    rightMotor.setSpeed(FORWARDSPEED);
             
    leftMotor.rotate(convertDistance(wheelRadius, distanceToTravel), true);
    rightMotor.rotate(convertDistance(wheelRadius, distanceToTravel), instantReturn);
    
  }
  /**Function that retrieves the tacho counts from both motors.
   * 
   * @return Array with left and right tacho counts. Convention is [0]=left tacho counts. [1] = right tacho counts.
   */
  public int[] getTachoCounts(){
    int[] tachoCounts = new int[2];

    tachoCounts[0] = leftMotor.getTachoCount();
    tachoCounts[1] = rightMotor.getTachoCount();
    
    return tachoCounts;
  }

  /**Updates the track value of the robot. Also known as wheel base.
   * 
   * @param idealTrack Track value of the robot in cm.
   */
  public void updateTrack(double idealTrack) {
    this.track = idealTrack;
    Odometer.TRACK = idealTrack;
    Odometer.ROTATION_RADIUS = idealTrack/2;
    Lab5.TRACK = idealTrack;
    
  }
  
  /** Determines if the robot is presently moving/
   * 
   * @return    True if one of the motors is presently moving.
   */
  public static boolean isMoving() {
    return leftMotor.isMoving() || rightMotor.isMoving();
  }
  
  /**Function that navigates around the serachZone. 
   * Rotates towards the search zone every 10cm and checks the distance.
   * 
   * @param maxDistanceToCheck  Distance in cm to verify if a block is present inside the searchZone. Usually the size of the searchZone/2.
   * @param searchDirection         Integer that determines in what direction that the robot must travel in. 0=+ve y, 1= +ve x, 2= -ve y, 3 = -ve x.
   * @return                    True if a block is detected and the robot must go check it out. False if no block has been detected.
   */
  public boolean navigateAroundSearchZone(int maxDistanceToCheck, int searchDirection) {
    double presentX, presentY;
    double nextX, nextY;   
    presentX = odo.getXYT()[0]/SQUARESIDE;
    presentY = odo.getXYT()[1]/SQUARESIDE;
    nextX = getNextDestinationPerimeterSearch(searchDirection, presentX, presentY)[0];
    nextY = getNextDestinationPerimeterSearch(searchDirection, presentX, presentY)[1];
    travelTo(nextX, nextY);
    
    turnTo(90 * (searchDirection+1),true,false);
    
    if(distance < maxDistanceToCheck) return true;
    turnTo(90*searchDirection, true, false);
    return false;
  }
  
  /**Robot travels to its believed location of the robot. Leaves ~9cm between the robot and the block.
   * Checks the block's color is any. Advances twice, each 1cm at a time, to detect the color of the block.
   * Stops after three unsuccessful attempts of checking the color.
   * 
   * @param maximumDetectableDistance Value of the maximum distance a robot will recognize a block in cm. Usually the searchZone's width/2.
   */
  public void goSeeBlock(int maximumDetectableDistance) {
    double beginningX = odo.getXYT()[0], beginningY = odo.getXYT()[1];
    double finalX, finalY, distanceCovered;
    
    //Block Detection must happen at 9cm
    int distanceToCheck = distance - 9;
    advanceRobot(1.5*distanceToCheck,true);    
    while(isMoving() && distance > 9 && distance < maximumDetectableDistance) {
      //do nothing
    }
    
    //check for the color of the flag.
    for(int i = 0;FlagDetection.colorDetected.equals("NO COLOR") && i < 4 ; i++) {
      advanceRobot(1,false);
      FlagDetection.findColorOfBlock(false);
      if(FlagDetection.beepingSequenceWhenDetectingBlock()) break;
    }
    //If a block is detected, back up so that when you will rotate, you won't hit the block.
    finalX = odo.getXYT()[0];
    finalY = odo.getXYT()[1];
    distanceCovered = Math.sqrt(Math.pow(finalX - beginningX, 2) + Math.pow(finalY - beginningY, 2));
    Navigation.advanceRobot(-distanceCovered, false);
    stopMotors();
    
  }
  /**This function calculates the next coordinate that the robot will go to for measuring if a flag is present.
   * 
   * @param searchDirection    Integer that determines in what direction that the robot must travel in. 0=+ve y, 1= +ve x, 2= -ve y, 3 = -ve x. 
   * @param presentX       X coordinate of robot.
   * @param presentY       Y coordinate of robot
   * @return               Double array of size 2 that contains the X, Y position of the next coordinate for sampling. Convention is [x,y].
   */
  public double[] getNextDestinationPerimeterSearch(int searchDirection, double presentX,double presentY) {
    double[] newCoords = new double[2];
    if(searchDirection == 0 ) { //Traveling in +ve y-direction.
     newCoords[0] = presentX;
     newCoords[1] = presentY + CONSTANT_FLAG_CHECKING_DISTANCE/SQUARESIDE;
    }
    else if(searchDirection == 1 ) { //Traveling in +ve x-direction.
      newCoords[0] = presentX + CONSTANT_FLAG_CHECKING_DISTANCE/SQUARESIDE;
      newCoords[1] = presentY;
    }
    else if(searchDirection == 2 ) { //Traveling in -ve y-direction.
      newCoords[0] = presentX;
      newCoords[1] = presentY - CONSTANT_FLAG_CHECKING_DISTANCE/SQUARESIDE;
    }
    else if(searchDirection == 3 ) { //Traveling in -ve x-direction.
      newCoords[0] = presentX - CONSTANT_FLAG_CHECKING_DISTANCE/SQUARESIDE;
      newCoords[1] = presentY;
    

    }
    return newCoords;
  }
  
  public double getContstantFlagCheckingDistance() {
    return CONSTANT_FLAG_CHECKING_DISTANCE/SQUARESIDE;
  }
  
  public void setForwardAndRotatingSpeed(int FORWARD_SPEED, int ROTATE_SPEED) {
    this.ROTATIONSPEED = ROTATE_SPEED;
    this.FORWARDSPEED = FORWARD_SPEED;
  }
}
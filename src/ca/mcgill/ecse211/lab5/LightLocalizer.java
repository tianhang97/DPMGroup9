/**
 * This class precisely localizes the robot with the use of the light sensor and the black lines on the ground.
 * @author Shawn Vosburg
 */

package ca.mcgill.ecse211.lab5;

import ca.mcgill.ecse211.odometer.Odometer;
import ca.mcgill.ecse211.odometer.OdometryCorrection;
import ca.mcgill.ecse211.odometer.OdometerExceptions;
import lejos.hardware.Button;
import lejos.hardware.Sound;
import lejos.hardware.sensor.EV3ColorSensor;

public class LightLocalizer {
  private static final double TRACK_ERROR = 0.108;
  private static double LS_OFFSET; //Distance from LS to center of rotation in cm. 
  private static double wheelRadius;
  private Navigation Navigator;
  private EV3ColorSensor LS;
  private Odometer odo;
  private static double SQUARESIDE;
  public static double tXminus, tXplus, tYminus, tYplus,deltaTy,deltaTx; //Variables that will record at what theta each lines were crossed at.
  private static int FORWARD_SPEED, ROTATE_SPEED;
  
  public LightLocalizer(Navigation Navigator, EV3ColorSensor lightSensor, double wheelRadius, double LS_TO_CENTER_OFFSET, double SQUARESIZE, int LOCALIZATION_SPEED, int forwardSpeed, int rotateSpeed) throws OdometerExceptions {
    this.Navigator = Navigator;
    this.LS = lightSensor;
    this.odo = Odometer.getOdometer();
    this.wheelRadius = wheelRadius;
    this.LS_OFFSET = LS_TO_CENTER_OFFSET;
    this.SQUARESIDE = SQUARESIZE;
    this.Navigator.setForwardAndRotatingSpeed(LOCALIZATION_SPEED, LOCALIZATION_SPEED);
    this.FORWARD_SPEED = forwardSpeed;
    this.ROTATE_SPEED = rotateSpeed;
  }
  /**
   * This function localizes the robot using the black lines. Assumes the robot is in the bottom left square. Also, assumes that a full rotation will
   * cause the light sensor to only cross 4 black lines.
   */
  
  public void Localize() {
    OdometryCorrection.startCorrection = false;
    
    //With the robot's position guaranteed to produce a 4 rotation with the LS crossing 4 lines, we can continue.
    //Now we fully rotate 360 and record when each of the black lines were crossed.
    Navigator.turnTo(45, false, false);
    odo.setTheta(0);
    
    stopAtNextBlackLine();
    tXminus = odo.getXYT()[2];
    
    stopAtNextBlackLine();
    tYplus = odo.getXYT()[2];
    
    stopAtNextBlackLine();
    tXplus = odo.getXYT()[2];
    
    stopAtNextBlackLine();
    tYminus = odo.getXYT()[2];
    
    Navigator.stopMotors();
    
    /*Now time for some maths.
    *From tutorial slides,  x= -d*cos(deltaTy/2)
    *                       y= -d*cos(deltaTx/2)
    *                       CorrectTheta = 270 + deltaY/2
    */
    deltaTx = tXplus - tXminus;
    deltaTy = tYminus - tYplus;
    
    if(deltaTx <0) deltaTx +=360;
    if(deltaTy <0) deltaTy +=360;
    
    odo.setX(-Math.abs(LS_OFFSET * Math.cos(deltaTy*Math.PI/360)));
    odo.setY(-Math.abs(LS_OFFSET * Math.cos(deltaTx*Math.PI/360)));
    
    odo.setTheta(273.5+deltaTy/2);      //Theoretically 270. In practice it is 274 .          
    
    //When this is done, travel to the origin and turn to Theta=0.
    Navigator.travelTo(0, 0);
    Navigator.turnTo(0, false, false);
    
    odo.setXYT(SQUARESIDE, SQUARESIDE, odo.getXYT()[2]);
    OdometryCorrection.startCorrection = true;
    OdometryCorrection.resetGyroSampler();
    
    //Update the running speeds in Navigation
    Navigator.setForwardAndRotatingSpeed(FORWARD_SPEED, ROTATE_SPEED);
  }
  /**
   * This function will rotate the robot clockwise and will stop the robot when a black line is met. 
   */
  private void stopAtNextBlackLine() {
    Navigator.rotateRobot(10, false, true);
    while(true) {
      Navigator.rotateRobot(360, true, true);
      while(Navigation.isRotating()) {
        if(LS.getColorID() == 13) {
          //When a black line is detected, stop the motors and beep.
          Navigator.stopMotors();
          Sound.beep();
          return;
        }
      }
      
    }

    
  }
  
  public double findIdealTrackValue() {
    stopAtNextBlackLine();
    
    int initialTachoLeft = Navigator.getTachoCounts()[0];
    int initialTachoRight = Navigator.getTachoCounts()[1];
    
    //Rotate a true 360deg
    for(int counter=0;counter<4;counter++) stopAtNextBlackLine();
    
    int finalTachoLeft = Navigator.getTachoCounts()[0];
    int finalTachoRight = Navigator.getTachoCounts()[1];
    
    int deltaTachoLeft = finalTachoLeft - initialTachoLeft;
    int deltaTachoRight = finalTachoRight - initialTachoRight;
    
    double diffInTachoInRads = Math.toRadians((double) Math.abs(deltaTachoRight - deltaTachoLeft));
    
    return wheelRadius*diffInTachoInRads/(2*Math.PI)+ TRACK_ERROR;
    
  
  }
  
  
}

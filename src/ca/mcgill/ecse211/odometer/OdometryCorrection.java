/** OdometryCorrection.java
 * This class corrects the odometer and ensures that the effective origin is located at the bottom left intersection.
 * It assumes that the robot performs perfect 90deg turns (Which is the whole purpose of the TEST TRACK state).
 * 
 * @author Shawn Vosburg
 * @author Tianhang Wu
 * 
 * Date: January 29th, 2018
 * McGill University, Montreal, Canada.
 */
package ca.mcgill.ecse211.odometer;

import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3GyroSensor;
import lejos.hardware.sensor.SensorMode;
import lejos.robotics.Color;
import lejos.robotics.SampleProvider;
import java.sql.SQLInvalidAuthorizationSpecException;
import ca.mcgill.ecse211.lab5.Lab5;
import ca.mcgill.ecse211.lab5.Navigation;
import lejos.hardware.Sound;

public class OdometryCorrection implements Runnable {
  private static final long CORRECTION_PERIOD = 10;
  private static double DISTANCE_CENTER_TO_LS;         //distance from LS to center of rotation of robot in cm.  
  private static final double SQUARESIDE = 30.48;                   //length of one square on the board in cm.
  private static final double DISTANCE_FROM_LINE_INTOLERANCE = 4;      //If a coordinate is within this distance in cm from a detected line, it will update
  private static final double ERROR_THETA_TOLERANCE = 0.5;
  private static final double ERROR_XY_TOLERANCE = 1;
  private Odometer odometer;
  public static EV3ColorSensor LS;
  private static EV3GyroSensor GyroSensor;
  private static SampleProvider GS_Sampler;
  public static float[] gyroSample = new float[1];
  public static boolean startCorrection = true;
  


  /**
   * This is the default class constructor. An existing instance of the odometer is used. This is to
   * ensure thread safety.
   * 
   * @throws OdometerExceptions
   */
  public OdometryCorrection(EV3ColorSensor lightSensor,double DISTANCE_CENTER_TO_LS, EV3GyroSensor GyroSensor) throws OdometerExceptions {

    this.odometer = Odometer.getOdometer();
    this.LS = lightSensor;
    this.DISTANCE_CENTER_TO_LS = DISTANCE_CENTER_TO_LS;
    this.GyroSensor = GyroSensor;
    this.GyroSensor.reset();
    this.GS_Sampler = this.GyroSensor.getAngleMode();
    //Set the current mode to ambient light level.
    LS.setCurrentMode(1);
   
  }

  /**
   * Here is where the odometer correction code should be run.
   * 
   * @throws OdometerExceptions
   */
  // run method (required for Thread)
  public void run() {
    long correctionStart, correctionEnd;
    
    double presentX, presentY, presentT, errorT;
    
    
    while (true) {
      if(Navigation.isNavigating() && startCorrection) {
        correctionStart = System.currentTimeMillis();
        
        GS_Sampler.fetchSample(gyroSample, 0);
        //A LINE GOT DETECTED
        if(LS.getColorID() == 13) {
          Sound.beep();
          //Get present values
          presentT = odometer.getXYT()[2];
          presentX = odometer.getXYT()[0] + DISTANCE_CENTER_TO_LS * Math.sin(Math.toRadians(presentT));
          presentY = odometer.getXYT()[1] + DISTANCE_CENTER_TO_LS * Math.cos(Math.toRadians(presentT));
          
          
          //Calculating the distance from a potential line.
          double errorX = presentX % SQUARESIDE;
          if(errorX > SQUARESIDE / 2) errorX = Math.abs(SQUARESIDE - errorX);
          
          double errorY = presentY % SQUARESIDE;
          if(errorY > SQUARESIDE / 2) errorY = Math.abs(SQUARESIDE - errorY);
          //We now have a positive error from the closest line.
          
          //If it is a vertical line that is cross, update only Xcoord. Check also if not too close from intersection.
          if(errorX <= DISTANCE_FROM_LINE_INTOLERANCE && errorY > DISTANCE_FROM_LINE_INTOLERANCE) {
            odometer.setX(Math.round(presentX/SQUARESIDE)*SQUARESIDE - DISTANCE_CENTER_TO_LS * Math.sin(Math.toRadians(presentT)));
            if(errorX > ERROR_XY_TOLERANCE ) Navigation.odometerCorrected = true;
          }
        
          //If it is a horizontal line that is cross, update only Ycoord. Check also if not too close from intersection.
          if(errorY <= DISTANCE_FROM_LINE_INTOLERANCE && errorX > DISTANCE_FROM_LINE_INTOLERANCE) {
            odometer.setY(Math.round(presentY/SQUARESIDE)*SQUARESIDE - DISTANCE_CENTER_TO_LS * Math.cos(Math.toRadians(presentT)));
            if(errorY > ERROR_XY_TOLERANCE)Navigation.odometerCorrected = true;
          }
          
          //Correcting for theta
          errorT = Math.abs(presentT + gyroSample[0]);
          if(errorT > ERROR_THETA_TOLERANCE)  {
            odometer.setTheta(-gyroSample[0]);
            Navigation.odometerCorrected = true;
          }
          
        }
        
  
        // this ensure the odometry correction occurs only once every period
        correctionEnd = System.currentTimeMillis();
        if (correctionEnd - correctionStart < CORRECTION_PERIOD) {
          try {
            Thread.sleep(CORRECTION_PERIOD - (correctionEnd - correctionStart));
          } catch (InterruptedException e) {
            // there is nothing to be done here
          }
        }
       }
      
    }
  }
  
  public static void resetGyroSampler() {
    GyroSensor.reset();
    GS_Sampler = GyroSensor.getAngleMode();
    
    // Sleep for 1 second
    try {
      Thread.sleep(1000);
    } catch (InterruptedException e) {
      // There is nothing to be done here
    }
  }
  
}
  



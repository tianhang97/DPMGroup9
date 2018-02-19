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
import lejos.hardware.sensor.SensorMode;
import lejos.robotics.Color;
import ca.mcgill.ecse211.lab5.Lab5;
import lejos.hardware.Sound;

public class OdometryCorrection implements Runnable {
  private static final long CORRECTION_PERIOD = 10;
  private static final double DISTANCE_CENTER_TO_LS = -15.9;         //distance from LS to center of rotation of robot in cm.  
  private static final double SQUARESIDE = 30.48;                   //length of one square on the board in cm.
  private static final double DISTANCE_FROM_LINE_ERROR = 0.05;      //If a coordinate is within this distance in cm from a detected line, it will update
  private Odometer odometer;
  public static EV3ColorSensor LS;
  
  //Variables that calculate the direction of the robot and what square it is in. 
  
  


  /**
   * This is the default class constructor. An existing instance of the odometer is used. This is to
   * ensure thread safety.
   * 
   * @throws OdometerExceptions
   */
  public OdometryCorrection(EV3ColorSensor lightSensor) throws OdometerExceptions {

    this.odometer = Odometer.getOdometer();
    this.LS = lightSensor;
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
    
    double presentX, presentY, presentT;
    
    while (true) {
      correctionStart = System.currentTimeMillis();
      
      
      //A LINE GOT DETECTED
      if(LS.getColorID() == 13) {
        //Sound.beep();
        //Get present values
        presentX = odometer.getXYT()[0];
        presentY = odometer.getXYT()[1];
        presentT = odometer.getXYT()[2];
        
        //If a line got detected and the x value of our robot is near a multiple of 30.48cm, then update it to that value.
        if(presentX % SQUARESIDE < DISTANCE_FROM_LINE_ERROR || presentX % SQUARESIDE > SQUARESIDE - DISTANCE_FROM_LINE_ERROR) {
         //Sound.beep();
          double integerMultipleX = Math.round(presentX / SQUARESIDE);
          
          odometer.setX(SQUARESIDE * integerMultipleX + DISTANCE_CENTER_TO_LS * Math.sin(presentT*Math.PI/180));       
        
          
        }
      
        //If a line got detected and the y value of our robot is near a multiple of 30.48cm, then update it to that value.
        //There is an else if because if our robot follows a black line, we only want one value to be updated.
        else if(presentY % SQUARESIDE < DISTANCE_FROM_LINE_ERROR || presentY % SQUARESIDE > SQUARESIDE - DISTANCE_FROM_LINE_ERROR) {
          //Sound.buzz();
          double integerMultipleY = Math.round(presentY / SQUARESIDE);
          
          odometer.setY(SQUARESIDE * integerMultipleY + DISTANCE_CENTER_TO_LS * Math.cos(presentT*Math.PI/180));       
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



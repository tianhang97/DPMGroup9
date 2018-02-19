/**
 * This class estimates the direction of the robot by scanning 2 perpendicular walls with an ultrasonic sensor.
 * @author Shawn Vosburg
 */

package ca.mcgill.ecse211.lab5;

import ca.mcgill.ecse211.odometer.Odometer;
import ca.mcgill.ecse211.odometer.OdometerExceptions;
import lejos.hardware.Button;
import lejos.hardware.Sound;

public class UltrasonicLocalizer{
  private Navigation Navigator = null;
  private boolean choicSelectedIsRisingEdge;
  private Odometer odo;                         // Need odometer to get present location values.
  private int d = 45;                           //Arbiturary distance in cm at which a wall is detected.
  private int k = 1;                            //+/- error in the d value above in cm.

  
  
  public UltrasonicLocalizer(Navigation Navigator, boolean choiceSelectedIsRisingEdge) throws OdometerExceptions {
    this.Navigator=Navigator;
    this.choicSelectedIsRisingEdge=choiceSelectedIsRisingEdge;  
    this.odo = Odometer.getOdometer();
  }
  
  /**
   * This function localizes the robot with either the rising or falling edge, as selected on the EV3.
   */
  public void Localize() {
    if(choicSelectedIsRisingEdge) LocalizeWithRisingEdge();
    else LocalizeWithFallingEdge();
  }
  
  /**
   * This function localizes the robot with 2 rising edges. Scans the left wall's rising edge than the back's wall rising edge.
   */
  
  private void LocalizeWithRisingEdge() {
    //Make a complete turn and a bit more (400degrees instead of 360) to be sure that the first rising edge will be captured
    Navigator.rotateRobot(720, true, true);
    
    //Figure out when the left wall stops.
    while(true) {
      if(Navigator.readUSDistance() < d+k) {        //This will let the robot know that it is facing a wall.
        break; 
      }
      
    }
    
    Sound.buzz();
    //Check to see when there is a rising edge.
    while(Navigator.readUSDistance() < d+k) {
      
    } //do nothing
    
   
    //When rising edge is detected, make the robot beep letting the user know. And set the odometer theta to 0deg.
    Navigator.stopMotors();
    odo.setTheta(360);
    Sound.beepSequenceUp();
    
    //Rotating Robot towards the other wall 45deg so the same rising edge is not counted twice.
    Navigator.rotateRobot(45, false, false);
    
    //tavel towards the next wall.
    Navigator.rotateRobot(720, true, false);
    
    //When next rising edge is discovered stop motors and adjust odometer.
    while(Navigator.readUSDistance() < d+k ) {
      
    }    //Do nothing
    Navigator.stopMotors();                     //When the rising edge is found, stop moving.
    odo.setTheta(227-odo.getXYT()[2]/2);        //The constant should be 225 in theory but in reality, it is 227
    Sound.beepSequenceUp();
    
    //Turn towards 0deg for grading by TA.
    Navigator.turnTo(0, false, false);
    
  }
  /**
   * This function localizes the robot with 2 falling edges. Scans the left wall's falling edge than the back's wall falling edge.
   */
  
  private void LocalizeWithFallingEdge() {
 
    //Make a complete turn and a bit more (400degrees instead of 360) to be sure that the first rising edge will be captured
    Navigator.rotateRobot(720, true, false);
    
    //Figure out when the robot is in the void..
    while(true) {
      if(Navigator.readUSDistance() > d-k) {        //This will let the robot know that it is facing a wall.
        break; 
      }
      
    }
    
    Sound.buzz();
    //Check to see when there is a rising edge.
    while(Navigator.readUSDistance() > d-k) {
      
    } //do nothing
    
   
    //When falling edge is detected, make the robot beep letting the user know. And set the odometer theta to 0deg.
    Navigator.stopMotors();
    odo.setTheta(0);
     Sound.beepSequenceUp();
     
    //Rotating Robot towards the other wall 45deg so the same rising edge is not counted twice.
    Navigator.rotateRobot(45, false, true);
    
    //travel towards the next wall.
    Navigator.rotateRobot(720, true, true);
    
    //When next falling edge is discovered stop motors and adjust odometer.
    while(Navigator.readUSDistance() > d-k ) {
      
    }    //Do nothing
    Navigator.stopMotors();                     //When the falling edge is found, stop moving.
    odo.setTheta( 43 + odo.getXYT()[2]/2 );         //In theory, the constant is 45 but in practice, it is 43 .
    Sound.beepSequenceUp();
  }
}

/**
 * This class is meant as a skeleton for the odometer class to be used.
 * 
 * @author Rodrigo Silva
 * @author Dirk Dubois
 * @author Derek Yu
 * @author Karim El-Baba
 * @author Michael Smith
 */

package ca.mcgill.ecse211.odometer;

import ca.mcgill.ecse211.lab5.Lab5;
import lejos.hardware.motor.EV3LargeRegulatedMotor;

public class Odometer extends OdometerData implements Runnable {

  private OdometerData odoData;
  private static Odometer odo = null; // Returned as singleton

  // Motors and related variables
  public int leftMotorTachoCount;
  public int rightMotorTachoCount;
  private EV3LargeRegulatedMotor leftMotor;
  private EV3LargeRegulatedMotor rightMotor;

  private static final double TRACK = Lab5.TRACK;
  private final double WHEEL_RAD;

  private double[] position;
  
  private int lastTachoL;
  private int lastTachoR;


  private static final long ODOMETER_PERIOD = 25; // odometer update period in ms
  private static final double ROTATION_RADIUS = TRACK/2; //the radius of rotation of our robot in cm
  /**
   * This is the default constructor of this class. It initiates all motors and variables once.It
   * cannot be accessed externally.
   * 
   * @param leftMotor
   * @param rightMotor
   * @throws OdometerExceptions
   */
  private Odometer(EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor,
      final double TRACK, final double WHEEL_RAD) throws OdometerExceptions {
    odoData = OdometerData.getOdometerData(); // Allows access to x,y,z
                                              // manipulation methods
    this.leftMotor = leftMotor;
    this.rightMotor = rightMotor;

    // Reset the values of x, y and z to 0
    odoData.setXYT(0, 0, 0);

    this.leftMotorTachoCount = 0;
    this.rightMotorTachoCount = 0;


    this.WHEEL_RAD = WHEEL_RAD;
    
    this.lastTachoL=0;
    this.lastTachoR=0;

  }

  /**
   * This method is meant to ensure only one instance of the odometer is used throughout the code.
   * 
   * @param leftMotor
   * @param rightMotor
   * @return new or existing Odometer Object
   * @throws OdometerExceptions
   */
  public synchronized static Odometer getOdometer(EV3LargeRegulatedMotor leftMotor,
      EV3LargeRegulatedMotor rightMotor, final double TRACK, final double WHEEL_RAD)
      throws OdometerExceptions {
    if (odo != null) { // Return existing object
      return odo;
    } else { // create object and return it
      odo = new Odometer(leftMotor, rightMotor, TRACK, WHEEL_RAD);
      return odo;
    }
  }

  /**
   * This class is meant to return the existing Odometer Object. It is meant to be used only if an
   * odometer object has been created
   * 
   * @return error if no previous odometer exists
   */
  public synchronized static Odometer getOdometer() throws OdometerExceptions {

    if (odo == null) {
      throw new OdometerExceptions("No previous Odometer exits.");

    }
    return odo;
  }

  /**
   * This method is where the logic for the odometer will run. Use the methods provided from the
   * OdometerData class to implement the odometer.
   */
  // run method (required for Thread)
  public void run() {
    long updateStart, updateEnd;

    while (true) {
      updateStart = System.currentTimeMillis();

      //Calculates how many degrees each wheel has rotated by. 
      leftMotorTachoCount = leftMotor.getTachoCount();
      rightMotorTachoCount = rightMotor.getTachoCount();

      //initialize changes & go get present theta value
      double dx, dy, dt;
      double presentTheta = odo.getXYT()[2];
      
      //These variables calculate what is the difference in degrees each wheel has seen
      double changeTachoL = leftMotorTachoCount - lastTachoL;
      double changeTachoR = rightMotorTachoCount - lastTachoR;
      
      //With the change variables, we can now find the common mode and difference between the two values.
      //the changes can now be defined as:
      // changeTachoL = commonTachoChange + diffTachoChange
      // changeTachoR = commonTachoChange - diffTachoChange
      double commonTachoChange = (changeTachoL + changeTachoR)/2 ;
      double diffTachoChange = (changeTachoL - changeTachoR)/2;
      
      //The commonTachoChange is the amount that contribute to pure forward movement while diffTachoChange contributes to pure rotational movement. 
      //the Catersian coordiate of Figure 3 in lab instructions is followed
        //Change from pure forward movement
      dy = convertTachoToArcLength(WHEEL_RAD, commonTachoChange) * Math.cos(Math.PI*presentTheta/180);
      dx = convertTachoToArcLength(WHEEL_RAD, commonTachoChange)* Math.sin(Math.PI*presentTheta/180);
      
        //Calculating change from rotation in radians, then converts into degrees. 
      dt =   convertTachoToArcLength(WHEEL_RAD, diffTachoChange)/ROTATION_RADIUS;
      dt = dt*180/Math.PI; 
      
      //Reset the LastTacho variables
      lastTachoL = leftMotorTachoCount;        
      lastTachoR = rightMotorTachoCount;
      
      
      
      
      odo.update(dx, dy, dt);

      // this ensures that the odometer only runs once every period
      updateEnd = System.currentTimeMillis();
      if (updateEnd - updateStart < ODOMETER_PERIOD) {
        try {
          Thread.sleep(ODOMETER_PERIOD - (updateEnd - updateStart));
        } catch (InterruptedException e) {
          // there is nothing to be done
        }
      }
    }
  }

  /** This function calculates the length traveled by the wheel using the formula: S=R*theta. 
   *  The function assumed perfect traction between the ground and the wheels.
   *  
   * @author Shawn Vosburg
   * @param radius The wheel radius in cm. 
   * @param deltaTacho The change in degrees of the wheel.
   * @return The arc length traveled by the wheel in cm. 
   */
  private static double convertTachoToArcLength(double radius, double deltaTacho) {
    return  (radius * deltaTacho)*(Math.PI/180) ;
  }

  
}

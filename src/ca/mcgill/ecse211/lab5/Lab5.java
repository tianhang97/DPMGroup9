/**This class contains the main function.
 * 
 * January 29th, 2018
 * McGill University, Montreal, Canada
 * @author Shawn Vosburg
 * @author TianHang Wu
 * 
 * 
 * This class contains the main method. It asks the user what type of test it wants to perform on
 * the EV3 autonomous robot. There is:
 *      1. Float the motors. This test unblocks the motors and allows to see if the odometer on the lcd display works properly.
 *      2. Uncorrected square driving. Tests the intrinsic accuracy of the odometer.
 *      3. Track Test. By placing the robot exactly on a corner, it can calculate the effective TRACK value
 *                      needed for perfect 90deg turns.
 *      4. Corrected Square Driving. Corrects the odometer so the origin is placed at the bottom left intersection. 
 */
package ca.mcgill.ecse211.lab5;

import org.jfree.util.Rotation;
import ca.mcgill.ecse211.lab5.UltrasonicPoller;
import ca.mcgill.ecse211.odometer.*;
import lejos.hardware.Button;
import lejos.hardware.Sound;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;

public class Lab5 {

  // Motor Objects, and Robot related parameters
  private static final EV3LargeRegulatedMotor leftMotor =
      new EV3LargeRegulatedMotor(LocalEV3.get().getPort("A"));
  private static final EV3LargeRegulatedMotor rightMotor =
      new EV3LargeRegulatedMotor(LocalEV3.get().getPort("B"));
  private static final TextLCD lcd = LocalEV3.get().getTextLCD();
  private static final EV3ColorSensor LS_BlackLines = 
      new EV3ColorSensor(LocalEV3.get().getPort("S1"));
  private static final EV3ColorSensor LS_BlockDetection = 
      new EV3ColorSensor(LocalEV3.get().getPort("S3"));
  
  static SensorModes usSensor = new EV3UltrasonicSensor(LocalEV3.get().getPort("S2")); // usSensor is the instance
  static SampleProvider usDistance = usSensor.getMode("Distance"); // usDistance provides samples from
                                                            // this instance
  static float[] usData = new float[usDistance.sampleSize()]; // usData is the buffer in which data are
                                                              // returned
  static float[] lsData = new float[3];
  
  //Constants that are intrinsic to the robot.
  public static final double WHEEL_RAD = 2.1;
  public static final double TRACK = 11.7 ;        //This value is extremely sensitive to the hardware and the battery level. Changes slightly but frequently.
  public static final int FORWARD_SPEED = 150;
  public static final int ROTATE_SPEED = 100;
  public static final double SQUARESIDE = 30.48;
  public static final int SIZEOFBOARD = 3;
  public static final double LS_TO_CENTER_OFFSET = -14.5;
  public static final String flagToBeDetected = "BLUE";
 
  public static void main(String[] args) throws OdometerExceptions {
    int buttonChoice;
    boolean choiceSelectedIsRisingEdge;
    
    // Odometer related objects
    Odometer odometer = Odometer.getOdometer(leftMotor, rightMotor, TRACK, WHEEL_RAD); // TODO Complete implementation
    OdometryCorrection odometryCorrection = new OdometryCorrection(LS_BlackLines); // TODO Complete
                                                                      // implementation
    Display odometryDisplay = new Display(lcd); // No need to change
   
    
 // Setup Ultrasonic Poller // This thread samples the US and invokes
    UltrasonicPoller usPoller = null; // the selected controller on each cycle

    //Setup LightSensor poller
    LightSensorPoller lsPoller = null;
    do {
      // clear the display
      lcd.clear();

      // ask the user whether the motors should drive in a square or float
      lcd.drawString("< Left | Right >", 0, 0);
      lcd.drawString("       |        ", 0, 1);
      lcd.drawString("Rising |Falling ", 0, 2);
      lcd.drawString(" Edge  |  Edge  ", 0, 3);
      lcd.drawString("       |        ", 0, 4);

      buttonChoice = Button.waitForAnyPress(); // Record choice (left or right press)
    } while (buttonChoice != Button.ID_LEFT && buttonChoice != Button.ID_RIGHT);

    //Select which routine to find theta.
    if (buttonChoice == Button.ID_LEFT)  choiceSelectedIsRisingEdge = true;
    else choiceSelectedIsRisingEdge=false;
         
    // Start odometer and display threads and correction Threads.
    Thread odoThread = new Thread(odometer);
    odoThread.start();
    Thread odoDisplayThread = new Thread(odometryDisplay);
    odoDisplayThread.start();
    
    //Create a navigator object for uspoller.
    final Navigation Navigator = new Navigation(leftMotor, rightMotor, WHEEL_RAD, TRACK, FORWARD_SPEED, ROTATE_SPEED,SIZEOFBOARD);
    
    //Odometer Correction
    Thread odoCorrectionThread = new Thread(odometryCorrection);
    //odoCorrectionThread.start();
  
    //Create ultrasonic and light localizer objects.
    final UltrasonicLocalizer USLocalizer = new UltrasonicLocalizer(Navigator,choiceSelectedIsRisingEdge);
    final LightLocalizer LSLocalizer = new LightLocalizer(Navigator,LS_BlackLines, WHEEL_RAD,LS_TO_CENTER_OFFSET);
    usPoller = new UltrasonicPoller(usDistance, usData,Navigator);; // the selected controller on each cycle
    usPoller.start();
    
    //Create lightsensor poller
    final FlagDetection FlagDetector = new FlagDetection();
    lsPoller = new LightSensorPoller(LS_BlockDetection,lsData,FlagDetector);
    lsPoller.start();
    //Set odometer to xyt = (0,0,0)
    odometer.setXYT(0, 0, 0);
    
    
    
    
    // Sleep for 2 seconds
    try {
      Thread.sleep(2000);
    } catch (InterruptedException e) {
      // There is nothing to be done here
    }
   
    /*Quick copy-paste for debugging the robot
     * 
     * USLocalizer.Localize();
        Button.waitForAnyPress();
        LSLocalizer.Localize();
     * 
     * for(int i=0;i<4;i++) {
              Navigator.rotateRobot(90, false, true);
            }
     */
    
    
    // spawn a new Thread to avoid SquareDriver.drive() from blocking
    (new Thread() {
      public void run() {
        
        FlagDetector.findFlag(flagToBeDetected);
        Button.waitForAnyPress();
        
        
        
      } 
    }).start();
    

    
    while (Button.waitForAnyPress() != Button.ID_ESCAPE);
    System.exit(0);
  }
}

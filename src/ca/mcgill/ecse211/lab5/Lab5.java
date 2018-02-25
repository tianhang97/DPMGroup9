/**This class contains the main function.
 * 
 * February 25th, 2018
 * McGill University, Montreal, Canada
 * @author Shawn Vosburg
 * @author TianHang Wu
 * @author Laurent Poulin
 * 
 * 
 * This class contains the main method. It demands the user to input a color block to search for and goes to search for it.
 *     
 */
package ca.mcgill.ecse211.lab5;

import ca.mcgill.ecse211.lab5.UltrasonicPoller;
import ca.mcgill.ecse211.odometer.*;
import lejos.hardware.Button;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3GyroSensor;
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
  private static final EV3GyroSensor GyroSensor = 
      new EV3GyroSensor(LocalEV3.get().getPort("S4"));
  
  static EV3UltrasonicSensor usSensor = new EV3UltrasonicSensor(LocalEV3.get().getPort("S2")); // usSensor is the instance
  static SampleProvider usDistance = usSensor.getMode("Distance"); // usDistance provides samples from
                                                            // this instance
  static float[] usData = new float[usDistance.sampleSize()]; // usData is the buffer in which data are
                                                              // returned
  static float[] lsData = new float[3];
  
  //Constants that are intrinsic to the robot.
  public static final double WHEEL_RAD = 2.1;
  public static double TRACK = 11.95 ;        //This value is extremely sensitive to the hardware and the battery level. Changes slightly but frequently.
  public static final int FORWARD_SPEED = 150;
  public static final int ROTATE_SPEED = 120;
  public static final double SQUARESIDE = 30.48;
  public static final int SIZEOFBOARD = 3;
  public static final double LS_TO_CENTER_OFFSET = -14.3;
   
  public static final double[] searchZone = {3,3,6,7}; // convention is {LL_X, LL_Y, UR_X, UR_Y}
  public static final int startCorner = 0;
  public static int blockColor = 0; // 1=red, 2= blue, 3=yellow, 4=white
      
  
  public static void main(String[] args) throws OdometerExceptions {
    int buttonChoice;
      
    // Setup Ultrasonic Poller // This thread samples the US and invokes
    UltrasonicPoller usPoller = null; // the selected controller on each cycle

    //Setup LightSensor poller
    LightSensorPoller lsPoller = null;
    
    //Asking the user for what color they want to search for
    do {
      // clear the display
      lcd.clear();

      // ask the user what color they want.
      lcd.drawString("< Left | Right >", 0, 0);
      lcd.drawString("       |        ", 0, 1);
      lcd.drawString("  RED  | YELLOW ", 0, 2);
      lcd.drawString(" BLUE  |  WHITE ", 0, 3);
      lcd.drawString("       |        ", 0, 4);

      buttonChoice = Button.waitForAnyPress(); // Record choice (left or right press)
    } while (buttonChoice != Button.ID_LEFT && buttonChoice != Button.ID_RIGHT);
    
    if( buttonChoice == Button.ID_LEFT) {
      do {
        // clear the display
        lcd.clear();

        // ask the user what color they want.
        lcd.drawString("< Left | Right >", 0, 0);
        lcd.drawString("       |        ", 0, 1);
        lcd.drawString("  RED  |  BLUE  ", 0, 2);
        lcd.drawString("       |        ", 0, 3);
        lcd.drawString("       |        ", 0, 4);

        buttonChoice = Button.waitForAnyPress(); // Record choice (left or right press)
      } while (buttonChoice != Button.ID_LEFT && buttonChoice != Button.ID_RIGHT);
      if(buttonChoice == Button.ID_LEFT) blockColor = 0;
      else blockColor = 1;
    }
    
    else {
      do {
        // clear the display
        lcd.clear();

        // ask the user what color they want.
        lcd.drawString("< Left | Right >", 0, 0);
        lcd.drawString("       |        ", 0, 1);
        lcd.drawString("YELLOW | WHITE  ", 0, 2);
        lcd.drawString("       |        ", 0, 3);
        lcd.drawString("       |        ", 0, 4);

        buttonChoice = Button.waitForAnyPress(); // Record choice (left or right press)
      } while (buttonChoice != Button.ID_LEFT && buttonChoice != Button.ID_RIGHT);
      if(buttonChoice == Button.ID_LEFT) blockColor = 2;
      else blockColor = 3;
    }
    
    // Odometer related objects
    final Odometer odometer = Odometer.getOdometer(leftMotor, rightMotor, TRACK, WHEEL_RAD); // TODO Complete implementation
    OdometryCorrection odometryCorrection = new OdometryCorrection(LS_BlackLines,LS_TO_CENTER_OFFSET,GyroSensor); // TODO Complete
                                                                      // implementation 
    Display odometryDisplay = new Display(lcd); // No need to change
    
         
    // Start odometer and display threads and correction Threads.
    Thread odoThread = new Thread(odometer);
    odoThread.start();
    Thread odoDisplayThread = new Thread(odometryDisplay);
    odoDisplayThread.start();
    
    //Create a navigator object for uspoller.
    final Navigation Navigator = new Navigation(leftMotor, rightMotor, WHEEL_RAD, TRACK, FORWARD_SPEED, ROTATE_SPEED,SIZEOFBOARD);
    
    //Odometer Correction
    Thread odoCorrectionThread = new Thread(odometryCorrection);
    odoCorrectionThread.start();

    
    //Create ultrasonic and light localizer objects.
    final UltrasonicLocalizer USLocalizer = new UltrasonicLocalizer(Navigator, false);
    final LightLocalizer LSLocalizer = new LightLocalizer(Navigator,LS_BlackLines, WHEEL_RAD,LS_TO_CENTER_OFFSET,SQUARESIDE);
    usPoller = new UltrasonicPoller(usDistance, usData,Navigator);; // the selected controller on each cycle
    usPoller.start();
    
    //Create lightsensor poller
    final FlagDetection FlagDetector = new FlagDetection(Navigator, SQUARESIDE, blockColor);
    lsPoller = new LightSensorPoller(LS_BlockDetection,lsData,FlagDetector);
    lsPoller.start();
    //Set odometer to xyt = (0,0,0)
    odometer.setXYT(SQUARESIDE, SQUARESIDE, 0);
    
    
    
    
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
        
        Navigator.rotateRobot(360, true, true);
        
      } 
    }).start();
    

    
    while (Button.waitForAnyPress() != Button.ID_ESCAPE);
    System.exit(0);
  }
}

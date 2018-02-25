/**Class that is responsible to search and detect the correct flag.
 * Navigates around the searchZone's perimeter and turns toward the searchZone to detect if a block is present.
 * @author Shawn Vosburg
 * 
 * February 25th, 2018
 * 
 */

package ca.mcgill.ecse211.lab5;

import ca.mcgill.ecse211.odometer.Odometer;
import ca.mcgill.ecse211.odometer.OdometerExceptions;
import lejos.hardware.Button;
import lejos.hardware.Sound;
import lejos.hardware.Sounds;

public class FlagDetection implements LightSensorController{
  //Class constants
  private static Navigation Navigator;
  private static double SQUARESIDE;
  private static Odometer odo;
  private static final double minimumDistanceToCheckBlock = 40;
  private static final int COUNTERMINIMUM = 5;
  private static final String[] FLAGCOLOR = {"RED", "BLUE", "YELLOW", "WHITE", "NO COLOR"};
  private static int colorDesired = 0;
  
  public static float R;
  public static float G;
  public static float B;
  private static double stdTolerance = 2;
  public static String colorDetected = "NO COLOR";
  
  //Conventions is [0] = red, [1]=blue, [2] = yellow, [3]=white
  private static double[] meanR = {1.8727,0.294,3.2733,3.1567};
  private static double[] meanG = {0.2646,0.7644,2.0489,3.0194};
  private static double[] meanB = {0.2058,1.1861,0.3724,2.6174};
  
  private static double[] stdR = {0.572427,0.080017,0.347269,0.377775};
  private static double[] stdG = {0.092971,0.230062,0.237717,0.39955};
  private static double[] stdB = {0.097454,0.267584,0.061981,0.317069};
  
  /**Class constructor. Sets up the proper constants and odometer.
   * 
   * @param Navigator   The class instance that control the movement of the robot.
   * @param SQUARESIDE  The length of one side of the board's square in cm.
   * @param blockColor  The desired flag color.
   * @throws OdometerExceptions
   */
  public FlagDetection(Navigation Navigator, double SQUARESIDE, int blockColor) throws OdometerExceptions {
    this.Navigator = Navigator;
    this.SQUARESIDE = SQUARESIDE;
    this.odo = Odometer.getOdometer();
    this.colorDesired = blockColor;
  }
  /**Takes in the data of the frontal light sensor.
   * @param colorValues Array of {R,G,B} obtained from EV3ColorSensor
   */
  public void processLSColorData(float[] colorValues) {
    R = 100*colorValues[0];
    G = 100*colorValues[1];
    B = 100*colorValues[2];
    
   
  }
  
  /** Implements the search Algorithm. Navigates around the perimeter of the searchZone and checks 
   * periodically to see if a block is present.
   * 
   * @param searchZone Coordinates of the searchZone. [LLx,LLy,URx,URy]
   */
  public void blockSearch(double[] searchZone) {
    //Disables the ongoing correction of the OdometryCorrection.
    Navigation.navigationCorrectionEnable = false;
    double[] searchLines = new double[4];
    double presentX, presentY;
    double deltaX=Math.abs(searchZone[0] - searchZone[2]), deltaY = Math.abs(searchZone[1] - searchZone[3]);
    
    //Determines the max distance to check in field.
    int maxDistanceToCheckX = (int) ((deltaX+1) * SQUARESIDE)/2;
    int maxDistanceToCheckY = (int) ((deltaY+1) * SQUARESIDE)/2;
   
    //searchLines contains the coordinates that the robot will travel along to detect block. It is purposefully
    //bigger than the searchZone.
    searchLines[0] = searchZone[0] - 0.5;
    searchLines[1] = searchZone[1] - 0.5;
    searchLines[2] = searchZone[2] + 0.5;
    searchLines[3] = searchZone[3] + 0.5;
    
    //Travels to the beginning of the searchLines.
    Navigator.travelTo(searchLines[0], searchLines[1]);
    Navigator.turnTo(0, false, false);
    
    //While the end of the searchZone is not reach, check for potential flags.
    while(odo.getXYT()[1] < searchZone[3]*SQUARESIDE) {
      while(!Navigator.navigateAroundSearchZone(maxDistanceToCheckX)) {
        //do nothing. This function will return when a block is seen.
      }
      //Record the position and go see block.
      presentX = odo.getXYT()[0]/SQUARESIDE;
      presentY = odo.getXYT()[1]/SQUARESIDE;
      Navigator.goSeeBlock();
      
      //After the block is seen and identified, come back to the position OUTSIDE the search zone.
      Navigator.travelTo(presentX, presentY);
      Navigator.turnTo(0, false, false);
      
      //Stop searching if the right color is detected.
      if(colorDetected.equals(FLAGCOLOR[colorDesired])) {
        return;
      }
      //reset the color String if the right color is not detected.
      else colorDetected = FLAGCOLOR[4];
      
    } 
    //Reset the correction String
    Navigation.navigationCorrectionEnable = true;
  }
 
  /**This function returns true if a flag is within detecting distance from the robot.
   * 
   * @param     maxDistance Maximum detecting distance as seen by the robot.
   * @return    True if the distance read is below the maximum required.
   */
  public boolean isThereFlag(double maxDistance) {
    if(Navigator.readUSDistance() < maxDistance) return true;
    else return false;
  
  }
 
  /**This function detects the color of a block using its frontal color sensor.
   * 
   * @param infiniteColorChecking   Set to true if you want the robot to detect color indeterminately. For debugging purposes only.
   *                                Set to false otherwise for normal use.
   */
  public static void findColorOfBlock(boolean infiniteColorChecking) {
    double errorR = 0, errorG=0, errorB=0;
    int counterR = 0, counterB=0, counterY=0, counterW=0, counterVoid = 0;
    int colorCounterMinimum = COUNTERMINIMUM;
    if(infiniteColorChecking) colorCounterMinimum = -1;
    
    //Will return after COUTNERMINIMUM of the same block detection has been made. Is allowed to perform 3*COUNTERMINIMUM 
    while(true) {
      //Test for red block     
      errorR = Math.abs(R-meanR[0]);
      errorG = Math.abs(G-meanG[0]);
      errorB = Math.abs(B-meanB[0]);
      
      //Determines if the stdError is within range for RED block
      if(errorR < stdTolerance*stdR[0] && errorG < stdTolerance*stdG[0] && errorB < stdTolerance*stdB[0]) {
        colorDetected = "RED";
        Sound.playTone(400, 100);
        counterR++;
        
        if(counterR == colorCounterMinimum) return;
        continue;
      }
      
      //Test for Blue block
      errorR = Math.abs(R-meanR[1]);
      errorG = Math.abs(G-meanG[1]);
      errorB = Math.abs(B-meanB[1]);
      
      //Determines if the stdError is within range for BLUE block
      if(errorR < stdTolerance*stdR[1] && errorG < stdTolerance*stdG[1] && errorB < stdTolerance*stdB[1]) {
        colorDetected = "BLUE";
        Sound.playTone(500, 100);
        counterB++;
             
        if(counterB == colorCounterMinimum) return;
        continue;
      }
      
      //Test for Yellow block
      errorR = Math.abs(R-meanR[2]);
      errorG = Math.abs(G-meanG[2]);
      errorB = Math.abs(B-meanB[2]);
      
      //Determines if the stdError is within range for Yellow block
      if(errorR < stdTolerance*stdR[2] && errorG < stdTolerance*stdG[2] && errorB < stdTolerance*stdB[2]) {
        colorDetected = "YELLOW";
        Sound.playTone(600, 100);
        counterY++;       
        
        if(counterY == colorCounterMinimum) return;
        continue;
      }
      
      //Test for white block
      errorR = Math.abs(R-meanR[3]);
      errorG = Math.abs(G-meanG[3]);
      errorB = Math.abs(B-meanB[3]);
      
      //Determines if the stdError is within range for WHITE block
      if(errorR < stdTolerance*stdR[3] && errorG < stdTolerance*stdG[3] && errorB < stdTolerance*stdB[3]) {
        colorDetected = "WHITE";
        Sound.playTone(700, 100);
        
        counterW++;
        
        if(counterW == colorCounterMinimum) return;
        continue;
      }
      
      //Reset the color back to null if no color has been detected.
      colorDetected = "NO COLOR";
      counterVoid++;
      if(counterVoid == 3*colorCounterMinimum) return;
      
    }
  }
  
  /**Travels to the beginning of the searchZone.
   * 
   * @param searchZone  Array of coords of the search zone. [LLx,LLy,URx,URy]
   */
  public void goToSearchZone(double[] searchZone) {
    Navigator.travelTo(searchZone[0], searchZone[1]);
  }
  
  /**Travels to the end of the searchZone
   * 
   * @param searchZone Array of coords of the search zone. [LLx,LLy,URx,URy]
   */
  public void exitSearchZone (double[] searchZone) {
    Navigator.travelTo(searchZone[2], searchZone[3]);
  }
  
  
}

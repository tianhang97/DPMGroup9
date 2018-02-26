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
import lejos.robotics.filter.MaximumFilter;

public class FlagDetection implements LightSensorController{
  //Class constants
  private static Navigation Navigator;
  private static double SQUARESIDE;
  private static Odometer odo;
  private static final int COUNTERMINIMUM = 5;
  private static final String[] FLAGCOLOR = {"RED", "BLUE", "YELLOW", "WHITE", "NO COLOR"};
  private static int colorDesired = 0;
  public static int searchDirection = 0;
  private static boolean correctFlagFound = false;
  private static final double TOLERANCE_ON_BLOCK_DETECTION = 1;
  private static int startCorner;
  private static double[] searchLines, searchZone;
  
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
  public FlagDetection(Navigation Navigator, double SQUARESIDE, int blockColor, int startCorner, double[] searchZone) throws OdometerExceptions {
    this.Navigator = Navigator;
    this.SQUARESIDE = SQUARESIDE;
    this.odo = Odometer.getOdometer();
    this.colorDesired = blockColor;
    this.startCorner = startCorner;
    this.searchZone = searchZone;
    
    //searchLines contains the coordinates that the robot will travel along to detect block. It is purposefully
    //bigger than the searchZone.
    this.searchLines = new double[4];
    searchLines[0] = searchZone[0] - 0.5;
    searchLines[1] = searchZone[1] - 0.5;
    searchLines[2] = searchZone[2] + 0.5;
    searchLines[3] = searchZone[3] + 0.5;
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
  public void blockSearch() {
    //Disables the ongoing correction of the OdometryCorrection.
    Navigation.navigationCorrectionEnable = false;
    double deltaX=Math.abs(searchZone[0] - searchZone[2]), deltaY = Math.abs(searchZone[1] - searchZone[3]);
    
    //Determines the max distance to check in field.
    int maxDistanceToCheckX = (int) ((deltaX+TOLERANCE_ON_BLOCK_DETECTION) * SQUARESIDE)/2;
    int maxDistanceToCheckY = (int) ((deltaY+TOLERANCE_ON_BLOCK_DETECTION) * SQUARESIDE)/2;
   
    
    for( searchDirection = 0; searchDirection < 4 && !correctFlagFound; searchDirection++) {
      //Travelling in...
      //... +ve y-direction.
      if(searchDirection == 0) {
        Navigator.travelTo(searchLines[0], searchLines[1]);
        while(odo.getXYT()[1] < (searchZone[3]- Navigator.getContstantFlagCheckingDistance()) * SQUARESIDE) {
          perimeterSearchRountine(searchDirection,maxDistanceToCheckX);
          //Stop searching if the right color is detected.
          if(correctFlagFound) break;
        }
      }
      
      //... +ve x-direction
      else if(searchDirection == 1) {
        Navigator.travelTo(searchLines[0], searchLines[3]);
        while(odo.getXYT()[0] < (searchZone[2]- Navigator.getContstantFlagCheckingDistance()) * SQUARESIDE) {
          perimeterSearchRountine(searchDirection,maxDistanceToCheckY);
          
          //Stop searching if the right color is detected.
          if(correctFlagFound) break;
        }
      }
      
      //... -ve y-direction
      else if(searchDirection == 2) {
        Navigator.travelTo(searchLines[2], searchLines[3]);
        while(odo.getXYT()[1] > (searchZone[1] + Navigator.getContstantFlagCheckingDistance()) * SQUARESIDE) {
          perimeterSearchRountine(searchDirection,maxDistanceToCheckX);
          
          //Stop searching if the right color is detected.
          if(correctFlagFound) break;
        }
      }
      
      //... -ve x-direction
      else {
        Navigator.travelTo(searchLines[2], searchLines[1]);
        while(odo.getXYT()[0] > (searchZone[0] + Navigator.getContstantFlagCheckingDistance()) * SQUARESIDE) {
          perimeterSearchRountine(searchDirection,maxDistanceToCheckY);
          
          //Stop searching if the right color is detected.
          if(correctFlagFound) break;
        }
      }
    
    }
    searchDirection--;
    //Reset the correction String
    exitSearchZone(searchZone,searchLines,searchDirection);
    Navigation.navigationCorrectionEnable = true;
  }
  
  /**This function calls the function that navigates around the side of the search zone. Also, checks to see if a block is present
   *    * 
   * @param searchDirection             Integer that determines in what direction that the robot must travel in. 0=+ve y, 1= +ve x, 2= -ve y, 3 = -ve x.
   * @param maxDistanceToCheck      Maximum distance read by USsensor that the robot will react to and go check the block.
   */
  private void perimeterSearchRountine(int searchDirection, int maxDistanceToCheck){
    if(Navigator.navigateAroundSearchZone(maxDistanceToCheck, searchDirection))  Navigator.goSeeBlock(maxDistanceToCheck);
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
        counterY++;       
        
        if(counterY == colorCounterMinimum) return;
        continue;
      }
      
      //Test for white block
      errorR = Math.abs(R-meanR[3]);
      errorG = Math.abs(G-meanG[3]);
      errorB = Math.abs(B-meanB[3]);
      
      //Determines if the stdError is within range for WHITE block
      if(errorR < (stdTolerance+1)*stdR[3] && errorG < (stdTolerance+1)*stdG[3] && errorB < (stdTolerance+1)*stdB[3]) {
        colorDetected = "WHITE";       
        counterW++;
        
        if(counterW == colorCounterMinimum) return;
        continue;
      }
     
      //Reset the color back to null if no color has been detected.
      colorDetected = "NO COLOR";
      counterVoid++;
      if(counterVoid == 25*colorCounterMinimum) return;
      
    }
    
  }
  
  /**Travels to the beginning of the searchZone.
   * 
   * @param searchZone  Array of coords of the search zone. [LLx,LLy,URx,URy]
   */
  public void goToSearchZone(double[] searchZone, int startCorner) {
    if(startCorner == 1 || startCorner == 2)    Navigator.travelTo(searchLines[2], searchLines[1]);
    
    Navigator.travelTo(searchZone[0], searchZone[1]);
    Sound.beep();
  }
  
  /**Travels to the end of the searchZone through the appropriate path.
   * 
   * @param searchZone Array of coords of the search zone. [LLx,LLy,URx,URy]
   */
  public void exitSearchZone (double[] searchZone, double[] searchLines ,int searchDirection) {
    if(searchDirection == 0) {
      Navigator.travelTo(searchLines[0], searchLines[3]);
      Navigator.travelTo(searchLines[2], searchLines[3]);
    }
    else if (searchDirection == 1) {
      Navigator.travelTo(searchLines[2], searchLines[3]);
    }
    
    else if (searchDirection == 2) {
      Navigator.travelTo(searchLines[2], searchLines[3]);
    }
    else if(searchDirection == 3) {
      Navigator.travelTo(searchLines[2], searchLines[1]);
      Navigator.travelTo(searchLines[2], searchLines[3]);
    }
    
    Navigator.travelTo(searchZone[2], searchZone[3]);
  }
  
  /**This function beeps the correct number of times when a block is detected.
   * @return Returns true if a block of color is detected. Returns false otherwise.
   */
  public static boolean beepingSequenceWhenDetectingBlock() {
    if(colorDetected.equals(FLAGCOLOR[colorDesired]))   {
      correctFlagFound = true;
      Sound.beep();
      return true;
    }
    else if (!colorDetected.equals(FLAGCOLOR[4])) {
      Sound.twoBeeps();
      colorDetected = FLAGCOLOR[4];
      return true;
    }
    else return false;
  }
}

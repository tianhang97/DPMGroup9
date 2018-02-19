package ca.mcgill.ecse211.lab5;

import lejos.hardware.Sound;

public class FlagDetection implements LightSensorController{
  public static float R;
  public static float G;
  public static float B;
  
  private static float minR = (float) 3;
  private static float minG = (float) 1;
  private static float minB = (float) 3;
  
  public void processLSColorData(float[] colorValues) {
    R = 1000*colorValues[0];
    G = 1000*colorValues[1];
    B = 1000*colorValues[2];
    
   
  }
  
  public void findFlag(String color) {
    while(true) {
      if(color.equals("RED") && R > minR && G <= minG && B <= minB) {
        Sound.beep();
      }
      else if(color.equals("BLUE") && R <= minR && G > minG && B > minB) {
        Sound.beep();
      }
      else if(color.equals("YELLOW") && R > minR && G > minG && B <= minB) {
        Sound.beep();
      }
      else if(color.equals("WHITE") && R > minR && G > minG && B > minB) {
        Sound.beep();
      }
    }
  }
  
}
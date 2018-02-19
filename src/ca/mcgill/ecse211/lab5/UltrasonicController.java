package ca.mcgill.ecse211.lab5;

/**
 * Class that initiates UltraSonic controller.
 */


public interface UltrasonicController {

  public void processUSData(int distance);

  public int readUSDistance();
}

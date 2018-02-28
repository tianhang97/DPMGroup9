/**Class that samples the LightSensor. Polls the RGB Values and return their respective intensities. 
 * @author Shawn Vosburg
 * 
 * February 25th, 2018
 * 
 */
package ca.mcgill.ecse211.lab5;

import lejos.hardware.sensor.EV3ColorSensor;
import lejos.robotics.SampleProvider;


public class LightSensorPoller extends Thread {
  private EV3ColorSensor ls;
  private LightSensorController cont;
  private float[] lsData;

  public LightSensorPoller(EV3ColorSensor ls, float[] lsData, LightSensorController cont) {
    this.ls = ls;
    this.cont = cont;
    this.lsData = lsData;
  }

  public void run() {
    while (true) {
      ls.getRGBMode().fetchSample(lsData, 0); // acquire data
      cont.processLSColorData(lsData); // now take action depending on value
      try {
        Thread.sleep(50);
      } catch (Exception e) {
      } // Poor man's timed sampling
    }
  }

}

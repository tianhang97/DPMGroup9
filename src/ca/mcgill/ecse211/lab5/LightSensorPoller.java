package ca.mcgill.ecse211.lab5;

import lejos.hardware.sensor.EV3ColorSensor;
import lejos.robotics.SampleProvider;

/**
 * Control of the wall follower is applied periodically by the UltrasonicPoller thread. The while
 * loop at the bottom executes in a loop. Assuming that the us.fetchSample, and cont.processUSData
 * methods operate in about 20mS, and that the thread sleeps for 50 mS at the end of each loop, then
 * one cycle through the loop is approximately 70 mS. This corresponds to a sampling rate of 1/70mS
 * or about 14 Hz.
 */
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

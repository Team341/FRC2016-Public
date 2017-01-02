package missdaisy;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Interface to laptop-based computer vision application.
 * 
 * No vision processing is actually done here; rather, this is just a thin interface to
 * SmartDashboard to pull values that are set by an offboard program.
 * 
 * @author Jared341
 */
public class Vision {
  private static Vision visionInstance = null;

  public static Vision getInstance() {
    if (visionInstance == null)
      visionInstance = new Vision();
    return visionInstance;
  }

  private Vision() {}

  public boolean seesTarget() {
    return SmartDashboard.getBoolean("found", false);
  }

  public double getRPM() {
    return 5000.0;
    // SmartDashboard.getNumber("rpms", 0.0);
  }

  public double getAzimuth() {
    return SmartDashboard.getNumber("azimuth", 0.0);
  }

  public double getRange() {
    return SmartDashboard.getNumber("range", 0.0);
  }
}

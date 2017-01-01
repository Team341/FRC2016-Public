package missdaisy.subsystems;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import missdaisy.Constants;

/**
 * The hanger, the subsystem that enables the robot to hang in the end game
 * 
 * @author Josh Sizer
 *
 */
public final class Hanger extends DaisySubsystem {
  private static Hanger hangerInstance = null;
  private Victor mHangerMotor;
  private Solenoid mHangerPiston;

  /**
   * Gets the instance of the hanger. Used in order to never have more than one hanger object, ever.
   * 
   * @return The one and only instance of the hanger
   */
  public static Hanger getInstance() {
    if (hangerInstance == null)
      hangerInstance = new Hanger();
    return hangerInstance;
  }

  private Hanger() {
    mHangerMotor = new Victor(Constants.PWMs.HANER_MOTOR);
    mHangerPiston = new Solenoid(Constants.Solenoids.HANGER);
  }

  public void setSpeed(double motorSpeed) {
    mHangerMotor.set(motorSpeed);
  }

  public void deploy() {
    mHangerPiston.set(false);
  }

  public void retract() {
    mHangerPiston.set(true);
  }

  public synchronized void reset() {
    setSpeed(0.0);
  }

  public void logToDashboard() {
    if (getCurrentController() != null)
      SmartDashboard.putString("HangerCurrentController", getCurrentController().toString());
    else
      SmartDashboard.putString("HangerCurrentController", "OpenLoop");
  }
}

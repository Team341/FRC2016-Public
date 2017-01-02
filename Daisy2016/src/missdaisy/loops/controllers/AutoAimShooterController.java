package missdaisy.loops.controllers;

import missdaisy.Constants;
import missdaisy.Vision;
import missdaisy.fileio.PropertySet;
import missdaisy.subsystems.Shooter;

/**
 * A controller that sets the RPM according the value supplied by the vision processing, ie by how
 * far away from the target the robot is.
 *
 */

public class AutoAimShooterController implements Controller {
  private static AutoAimShooterController autoAimShooterControllerInstance = null;
  private ShooterSpeedController mShooterController;
  private Shooter mShooter;

  /**
   * Gets the instance of the auto-aim shooter speed controller. Used in order to never have more
   * than one auto-aim shooter speed controller object, ever.
   * 
   * @return The one and only instance of the auto-aim shooter speed controller
   */
  public static AutoAimShooterController getInstance() {
    if (autoAimShooterControllerInstance == null)
      autoAimShooterControllerInstance = new AutoAimShooterController();
    return autoAimShooterControllerInstance;
  }

  private AutoAimShooterController() {
    mShooter = Shooter.getInstance();
    mShooterController = ShooterSpeedController.getInstance();
    loadProperties();
  }

  public synchronized void setGoal() {
    if (mShooter.isHoodBatterPosition()) {
      mShooterController.setGoal(Constants.Properties.SHOOTER_RPM_BATTER);
    } else if (mShooter.isHoodOuterworksPosition()) {
      mShooterController.setGoal(Constants.Properties.SHOOTER_RPM_OUTERWORKS);
    }
  }

  /**
   * If the robot sees the target, the shooter will ramp up to the specified RPM based on distance
   * from the target. Otherwise, it will ramp up to the default RPM
   */
  @Override
  public synchronized void run() {
    mShooterController.run();
  }

  /**
   * Sets the shooter motor off
   */
  @Override
  public synchronized void reset() {
    mShooterController.reset();
  }

  /**
   * Returns true if the shooter RPM is within an acceptable range of the goal RPM
   */
  @Override
  public synchronized boolean onTarget() {
    return mShooterController.onTarget();
  }

  @Override
  public void loadProperties() {}

  @Override
  public String toString() {
    return "AutoAimShooterController";
  }
}

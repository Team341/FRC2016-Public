package missdaisy.loops.controllers;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import missdaisy.Vision;
import missdaisy.loops.Navigation;

/**
 * Automatically tries to turn the robot towards the target. If the target is not in sight, then the
 * controller does not turn the robot
 * 
 */

public class AutoAimDriveController implements Controller {
  private static AutoAimDriveController autoAimDriveControllerInstance = null;
  private Vision mVision;
  private Navigation mNavigation;
  private DriveTurnController mDrive;

  /**
   * Gets the instance of the auto-aim drive turn controller. Used in order to never have more than
   * one auto aim-drive turn controller object, ever.
   * 
   * @return The one and only instance of the auto-aim drive turn controller
   */
  public static AutoAimDriveController getInstance() {
    if (autoAimDriveControllerInstance == null)
      autoAimDriveControllerInstance = new AutoAimDriveController();
    return autoAimDriveControllerInstance;
  }

  private AutoAimDriveController() {
    mVision = Vision.getInstance();
    mNavigation = Navigation.getInstance();
    mDrive = DriveTurnController.getInstance();
  }

  /**
   * Tries to turn the robot to the target if it identifies the target. If it cannot find the
   * target, the robot stays put
   */
  @Override
  public synchronized void run() {
    // if we can see the target, then we set the goal to the
    // angle that we want to turn to
    if (mVision.seesTarget()) {
      mDrive.setGoal(mVision.getAzimuth());
    } else {
      mDrive.setGoal(mNavigation.getApproxTargetAngle());
    }
    mDrive.run();
  }

  /**
   * Resets the controller's goal to be the current angle of the robot
   */
  @Override
  public synchronized void reset() {
    mDrive.reset();
  }

  /**
   * Returns true if the robot is facing the target correctly Returns false otherwise
   */
  @Override
  public synchronized boolean onTarget() {
    return mDrive.onTarget();
  }

  @Override
  public void loadProperties() {
    // No properties to load
  }

  @Override
  public String toString() {
    return "AutoAimDriveController";
  }
}

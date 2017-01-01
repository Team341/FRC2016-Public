package missdaisy.autonomous;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import missdaisy.loops.Navigation;
import missdaisy.loops.controllers.DriveOverDefenseController;
import missdaisy.subsystems.Drive;
import missdaisy.subsystems.Shooter;
import missdaisy.utilities.DaisyMath;

/**
 * This is a state that attempts to drive the robot over a defense. 
 * @author Joshua Sizer
 *
 */
public class DriveOverDefense extends State {
  private Drive mDrive;
  private Navigation mNavigation;
  private DriveOverDefenseController mDriveController;
  private double mDistanceToTarget;
  private double mMinDistance;
  private double mSpeed;
  
  /**
   * 
   * @param distanceToTarget The desired distance you want to end from the target
   * @param speed The speed to cross the defense with
   * @param minDistance The minimum distance to travel. This is to ensure that the robot attempts 
   *                    drive some distance over the defense.
   */
  public DriveOverDefense(double distanceToTarget, double speed, double minDistance) {
    super("DriveOverDefenseController");
    mDrive = Drive.getInstance();
    mNavigation = Navigation.getInstance();
    mDriveController = DriveOverDefenseController.getInstance();

    mDistanceToTarget = distanceToTarget;
    mSpeed = speed;
    mMinDistance = minDistance;
  }
  
  /**
   * Turns the robot's vision light on
   */
  public void enter() {
    Shooter.getInstance().setVisionLightState(true);
    mMinDistance = mNavigation.getAverageEncoderDistance() + mMinDistance;
    mDriveController.setGoal(mDistanceToTarget, mSpeed, mMinDistance);
    mDrive.setCurrentController(mDriveController);
    System.currentTimeMillis();
  }

  @Override
  public void running() {
    if (mDriveController.onTarget()) {
      mDrive.setSpeedTurn(0.0, 0.0);
    }
    SmartDashboard.putBoolean("DOD_OnTarget", mDriveController.onTarget());
  }

  @Override
  public boolean isDone() {

    return mDriveController.onTarget();// || ((System.currentTimeMillis() - startTime)/1000) > 3.0;
  }

  public void exit() {
    mDrive.setOpenLoop();
    mDrive.setSpeedTurn(0.0, 0.0);
    Shooter.getInstance().setVisionLightState(false);
  }
}

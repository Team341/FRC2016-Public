package missdaisy.autonomous;

import missdaisy.loops.controllers.DriveDistanceController;
import missdaisy.subsystems.Drive;

/**
 * Another class that simply drives a certain distance to the defense. 
 *
 */
public class DriveToDefense extends State {
  private Drive mDrive;
  private DriveDistanceController mDriveController;
  private double mSpeed;
  private double mDesiredDistance;
  private double mCurrentDistance;
  private double mDistanceToTravel;

  public DriveToDefense(double distanceFromTarget, double speed) {
    super("DriveToTarget");
    mDrive = Drive.getInstance();
    mDriveController = DriveDistanceController.getInstance();
    mDesiredDistance = distanceFromTarget;
    mSpeed = speed;
  }

  public void enter() {
    mDistanceToTravel = mCurrentDistance - mDesiredDistance;
    mDriveController.setGoal(mDistanceToTravel, mSpeed);
    mDrive.setCurrentController(mDriveController);
  }

  @Override
  public void running() {}

  public void exit() {
    mDrive.setOpenLoop();
  }

  @Override
  public boolean isDone() {
    return mDrive.seesDefense();
  }

}

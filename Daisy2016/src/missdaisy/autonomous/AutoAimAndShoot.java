package missdaisy.autonomous;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import missdaisy.Constants;
import missdaisy.Vision;
import missdaisy.loops.Navigation;
import missdaisy.loops.controllers.AutoAimDriveController;
import missdaisy.loops.controllers.ShooterSpeedController;
import missdaisy.subsystems.Drive;
import missdaisy.subsystems.Intake;
import missdaisy.subsystems.Shooter;
import missdaisy.utilities.DaisyMath;

/**
 * This state will attempt to aim at the target, and shoot when the robot is on target. In this
 * class is some logic that will move the robot forward/backward to ensure a correct distance from
 * the target. If the target is not visible to the robot, it will attempt a sweep left/right to find
 * the target.
 *
 * @author Adam N.
 */
public class AutoAimAndShoot extends State {
  private final Drive mDrive;
  private final Shooter mShooter;
  private final Intake mIntake;
  // The controllers that will attempt to aim and shoot
  private final AutoAimDriveController mDriveController;
  private final ShooterSpeedController mShooterController;
  private final Navigation mNavigation;
  private boolean mBallFired;
  private long startTime = -1;
  private long driveStartTime = 0;
  private long turnStartTime = 0;
  private long waitTime = 250;
  private int onTargetCounter;
  private double aimAngle;
  private double minRange = 230; // (112) for competition
  private double maxRange = 300; // (176) for competition
  private double initialTurnDirection = 0.0;

  /**
   * 
   * @param desiredAngle The angle we think the target will be at.
   */
  public AutoAimAndShoot(double desiredAngle) {
    super("AutoAimAndShoot");
    mDrive = Drive.getInstance();
    mShooter = Shooter.getInstance();
    mIntake = Intake.getInstance();
    mDriveController = AutoAimDriveController.getInstance();
    mShooterController = ShooterSpeedController.getInstance();
    mNavigation = Navigation.getInstance();
    mBallFired = false;
    aimAngle = desiredAngle;
  }

  public void enter() {
    // ensures all subsystems are in the correct state
    mShooter.setOuterworksPosition();
    mShooter.retractBarrier();
    mShooter.setVisionLightState(true);
    mShooterController.setGoal(4000);
    // mDrive.setCurrentController(mDriveController);
    mDrive.useAlphaFilter(true);
    mShooter.setCurrentController(mShooterController);
    startTime = -1;
    onTargetCounter = 0;
    driveStartTime = System.currentTimeMillis();
  }

  @Override
  public void running() {
    // sanity check
    mShooter.retractBarrier();
    
    // logging for debugging purposes
    SmartDashboard.putBoolean("AAS_ShooterOnTarget", mShooterController.onTarget());
    SmartDashboard.putBoolean("AAS_DriveOnTarget", mDriveController.onTarget());
    double range = Vision.getInstance().getRange();
    boolean rangeOnTarget = false;
    // we want to be controlling the drive in this autonomous state, not some other controller
    mDrive.setOpenLoop();
    if (System.currentTimeMillis() - driveStartTime >= 1000) {
      // driving back didn't help lets search left/right
      if (Vision.getInstance().seesTarget()) {
        rangeOnTarget = driveDistance();
        SmartDashboard.putString("AAS-state:", "Timedout: within Angle");
      } else {
        // this block will do a sweep, ie a left/right turn in order to search for the target  
        double currAngle = mNavigation.getHeadingInDegrees();
        double angleDelta = DaisyMath.boundAngleNeg180to180Degrees(aimAngle - currAngle);

        if (Math.abs(initialTurnDirection) < 0.1) {
          SmartDashboard.putString("AAS-state:", "Timedout: initialize turn");
          initialTurnDirection = Math.signum(angleDelta);
          turnStartTime = System.currentTimeMillis();
        } else if (initialTurnDirection > 0) {
          SmartDashboard.putString("AAS-state:", "Timedout: turn right");
          mDrive.setSpeedTurn(0.0, 0.5);
          if ((System.currentTimeMillis() - turnStartTime) > waitTime) {
            initialTurnDirection = -1.0;
            turnStartTime = System.currentTimeMillis();
            waitTime += 250;
          }
        } else {
          SmartDashboard.putString("AAS-state:", "Timedout: turn left");
          mDrive.setSpeedTurn(0.0, -0.5);
          if ((System.currentTimeMillis() - turnStartTime) > waitTime) {
            initialTurnDirection = 1.0;
            turnStartTime = System.currentTimeMillis();
            waitTime += 250;
          }
        }
      }
    } else {
      SmartDashboard.putString("AAS-state:", "Range Search");
      rangeOnTarget = driveDistance();
    }

    /*
     * 
     * if (range < minRange || !Vision.getInstance().seesTarget()) { mDrive.setSpeedTurn(-0.5, 0.0);
     * } else if (range > maxRange) { mDrive.setSpeedTurn(0.5, 0.0); } else {
     * mDrive.setSpeedTurn(0.0, 0.0); rangeOnTarget = true; }
     */

    if (rangeOnTarget) {
      mDrive.setCurrentController(mDriveController);
    }

    if (mDriveController.onTarget() && mShooterController.onTarget() && rangeOnTarget) {
      mShooter.setStatusLightState(true);
      onTargetCounter++;
      if (onTargetCounter > 10) {
        mIntake.setConveyorSpeed(1.0);
        if (startTime < 0.0)
          startTime = System.currentTimeMillis();
      }
    } else {
      mShooter.setStatusLightState(false);
      // onTargetCounter = 0;
    }

    SmartDashboard.putNumber("AAS_StartTime", startTime);
    
    // This checks to see if we are done shooting in order to end this state
    if (startTime > 0 && ((System.currentTimeMillis() - startTime) / 1000) >= 2) {
      SmartDashboard.putNumber("AAS_Counter", onTargetCounter);
      mBallFired = true;
      mIntake.setConveyorSpeed(0.0);
      mShooter.setSpeed(0.0);
    }
    SmartDashboard.putNumber("AAS_ElapsedTime", System.currentTimeMillis() - startTime);

    SmartDashboard.putBoolean("AAS_BallFired", mBallFired);
  }

  /**
   * Sets the drive to move forward or backwards, depending on whether the robot is too close
   * or too far from the target. 
   * @return True if the robot is within the desired range from the target
   */
  public boolean driveDistance() {
    boolean rangeOnTarget = false;
    double range = Vision.getInstance().getRange();

    if (range < minRange || !Vision.getInstance().seesTarget()) {
      mDrive.setSpeedTurn(-0.5, 0.0);
    } else if (range > maxRange) {
      mDrive.setSpeedTurn(0.5, 0.0);
    } else {
      mDrive.setSpeedTurn(0.0, 0.0);
      rangeOnTarget = true;
    }

    return rangeOnTarget;
  }

  @Override
  public boolean isDone() {
    return mBallFired;
  }

  /**
   * Ensures the robot is in the desired state when this routine is done
   */
  public void exit() {
    mShooter.setOpenLoop();
    mShooter.setSpeed(0.0);
    mDrive.setOpenLoop();
    mDrive.setSpeed(0.0, 0.0);
    mShooter.setVisionLightState(false);
    mIntake.setConveyorSpeed(0.0);
  }
}

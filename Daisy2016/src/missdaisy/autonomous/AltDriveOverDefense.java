package missdaisy.autonomous;

import missdaisy.loops.Navigation;
import missdaisy.subsystems.Drive;
import missdaisy.utilities.DaisyMath;

/**
 * An alternate state to drive over a defense which does not use software controllers. Combines
 * sensor data from the gyroscope, encoders, and banner sensor in order to determine when it has
 * successfully crossed a defense. This state will timeout after a certain number of milliseconds.
 * 
 * @author Josh Sizer
 */
public final class AltDriveOverDefense extends TimeoutState {
  private final Drive mDrive;
  private final double kP = 0.03;
  private final double mMinEncoderDistance = 65.0;
  private double mSpeed = 0;

  private final Navigation mNav;
  private final double mPitchTolerance = 3;
  private double mStartYaw;
  private double mStartPitch;
  private double mStartEncoderDistance;

  public AltDriveOverDefense(double speed, int timeout) {
    super("AltDriveOverDefense", timeout);
    mSpeed = speed;
    mDrive = Drive.getInstance();
    mNav = Navigation.getInstance();
  }

  @Override
  public void enter() {
    super.enter();
    mStartYaw = mNav.getHeadingInDegrees();
    mStartPitch = mNav.getPitchInDegrees();
    mNav.resetEncoders();
    mStartEncoderDistance = mNav.getAverageEncoderDistance();
  }

  /**
   * While this state is not done, it
   */
  @Override
  public void running() {
    double turn = getYawError() * kP;
    mDrive.setSpeedTurn(mSpeed, turn);
  }

  /**
   * This state is considered done if it has been running longer than its timeout, the pitch the
   * robot is at is approximately equal to the pitch it started the state at, the Drive base does
   * not see a defense, and the encoders measured that the wheels have spun at least the
   * minEncoderDistance.
   */
  @Override
  public boolean isDone() {
    return super.isDone() || (isPitchOnTarget() && distanceOnTarget());
    // isPitchOnTarget() && !mDrive.seesDefense() && distanceOnTarget()
  }

  private boolean isPitchOnTarget() {
    double error = DaisyMath.boundAngle0to360Degrees(mStartPitch - mNav.getPitchInDegrees());
    return (error > 180 ? error -= 360 : error) < mPitchTolerance;
  }

  private boolean distanceOnTarget() {
    return Math.abs(mNav.getAverageEncoderDistance()) > mMinEncoderDistance + mStartEncoderDistance;
  }

  private double getYawError() {
    double error = DaisyMath.boundAngle0to360Degrees(mStartYaw - mNav.getHeadingInDegrees());
    return error > 180 ? error -= 360 : error;
  }
}

package missdaisy.autonomous;

import missdaisy.loops.controllers.ShooterSpeedController;
import missdaisy.subsystems.Intake;
import missdaisy.subsystems.Shooter;

/**
 * A class meant for the spybot autonomous, where the robot simply needs to shoot a ball.
 *
 */
public class JustShoot extends TimeoutState {
  private final Shooter mShooter;
  private final Intake mIntake;
  private final ShooterSpeedController mShooterSpeedController;
  private double mStartTime = -1;
  private double mOnTargetCounter = 0;
  private boolean mBallFired;

  /**
   * The amount of time to spin the conveyor to shoot the ball.
   * @param timeout
   */
  public JustShoot(int timeout) {
    super("JustShoot", timeout);
    mShooter = Shooter.getInstance();
    mIntake = Intake.getInstance();
    mShooterSpeedController = ShooterSpeedController.getInstance();
  }

  @Override
  public void enter() {
    // all timeout states must call super.enter to start the timer.
    super.enter();
    mShooterSpeedController.setGoal(4750);
    mShooter.setCurrentController(mShooterSpeedController);
    mBallFired = false;
  }

  /**
   * Waits until the shooter is up to speed, and has been up to speed for a certain number of 
   * loops. Then, when it has been up to speed for a while, the conveyor runs.
   */
  @Override
  public void running() {
    if (mShooterSpeedController.onTarget()) {
      mShooter.setStatusLightState(true);
      mOnTargetCounter++;
      if (mOnTargetCounter > 5) {
        mIntake.setConveyorSpeed(1.0);
        if (mStartTime < 0.0)
          mStartTime = System.currentTimeMillis();
      }
    } else {
      mShooter.setStatusLightState(false);
      mOnTargetCounter = 0;
      mStartTime = -1;
    }

    if (mStartTime > 0 && Math.abs(System.currentTimeMillis() - mStartTime) / 1000 >= 2) {
      mBallFired = true;
      mIntake.setConveyorSpeed(0.0);
      mShooter.setSpeed(0.0);
    }

  }

  /**
   * This state will exit if either the specified time has elapsed or if the robot has shot a ball
   */
  @Override
  public boolean isDone() {
    return super.isDone() || mBallFired;
  }

}

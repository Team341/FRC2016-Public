package missdaisy.loops.controllers;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import missdaisy.Constants;
import missdaisy.Vision;
import missdaisy.fileio.PropertySet;
import missdaisy.loops.Navigation;
import missdaisy.loops.SynchronousPID;
import missdaisy.subsystems.Drive;
import missdaisy.utilities.DaisyMath;

/**
 * Uses PID calculations to drive a certain distance in a straight line.
 * 
 */
// The class's inherited PID is the PID that controls distance.
public class DriveOverDefenseController extends SynchronousPID implements Controller {
  private static DriveOverDefenseController driveDistanceControllerInstance = null;
  private Drive mDrive;
  private Navigation mNavigation;
  private Vision mVision;
  private SynchronousPID mTurnPID; // This makes sure we drive in a straight line
  private double mMaxOutput = 1.0; // the maximum output to the drive motors
  private double mGoalAngle = 0.0;
  private double mCurrentAngle;

  private double mStartingPitch;
  private double mDistanceToTarget;
  private double mMinDistance;

  private boolean mDistanceMet = false;
  private long startTime;
  private double mStartingEncoderValue;

  /**
   * Gets the instance of the drive straight controller. Used in order to never have more than one
   * drive straight controller object, ever.
   * 
   * @return The one and only instance of the drive straight controller
   */
  public static DriveOverDefenseController getInstance() {
    if (driveDistanceControllerInstance == null)
      driveDistanceControllerInstance = new DriveOverDefenseController();
    return driveDistanceControllerInstance;
  }

  private DriveOverDefenseController() {
    mTurnPID = new SynchronousPID();
    mTurnPID.setContinuous();
    mTurnPID.setInputRange(0, 360);
    mTurnPID.setOutputRange(-0.2, 0.2);
    mDrive = Drive.getInstance();
    mVision = Vision.getInstance();
    mNavigation = Navigation.getInstance();
    loadProperties();
  }

  /**
   * Set the desired distance to travel and the maximum output of the PID, a number between 0.0 and
   * 1.0. The value between 0.0 and 1.0 indicates speeds between full stop and full power The
   * distance to travel should be positive or negative, the max power should be only positive
   * 
   * @param distance The distance to travel, either backwards or forwards (negative or positive)
   * @param maxoutput The maximum output for the drive motors
   */
  public synchronized void setGoal(double distanceToTarget, double speed, double minDistance) {
    super.reset();
    mTurnPID.reset();
    mNavigation.resetEncoders();
    mStartingEncoderValue = mNavigation.getAverageEncoderDistance();
    // we'll assume that the angle you wish to drive at is the same
    // as the one when you set the goal
    mGoalAngle = mNavigation.getHeadingInDegrees();
    mStartingPitch = mNavigation.getPitchInDegrees();
    mDistanceToTarget = distanceToTarget;
    // you can't know how far you've traveled unless you know where you started
    // makes sure that the maxVelocity is a positive number and not greater than 1.0
    if (speed < 0.0)
      speed *= -1.0;
    if (speed > 1.0)
      speed = 1.0;
    mMaxOutput = speed;
    super.setOutputRange(-mMaxOutput, mMaxOutput);
    mMinDistance = minDistance;
    // we want the difference between where we want to be and where we are to be 0
    super.setSetpoint(mNavigation.getAverageEncoderDistance() + minDistance);
    mTurnPID.setSetpoint(mGoalAngle);
    // same with the angle
    startTime = System.currentTimeMillis();
  }

  /**
   * Attempts to drive the robot a certain distance in a straight line
   */
  @Override
  public synchronized void run() {
    // what angle you're at right now
    mCurrentAngle = mNavigation.getHeadingInDegrees();
    // how far you've traveled since you've set your goal

    if (!onTarget()) {
      double turn = mCurrentAngle * mTurnPID.getP();
      mDrive.setSpeedTurn(mMaxOutput, turn);
    } else {
      mDrive.setSpeedTurn(0.0, 0.0);
    }

    if ((System.currentTimeMillis() - startTime) / 1000 > 2.0) {
      mDistanceMet = true;
    } else {
      mDistanceMet = false;
    }
    SmartDashboard.putNumber("DOD_TimeDelta", (System.currentTimeMillis() - startTime) / 1000);
    mDistanceMet = mNavigation.getAverageEncoderDistance() > super.getSetpoint();
  }

  /**
   * Resets all internal variables
   */
  @Override
  public synchronized void reset() {
    super.reset();
    mTurnPID.reset();
    mMaxOutput = 1.0;
    mGoalAngle = 0.0;
    mCurrentAngle = mNavigation.getHeadingInDegrees();
  }

  /**
   * Returns true if the robot has driven the desired distance and its heading is the same as it
   * started
   */
  @Override
  public synchronized boolean onTarget() {

    double pitchError =
        DaisyMath.boundAngle0to360Degrees(mStartingPitch - mNavigation.getPitchInDegrees());
    if (pitchError > 180)
      pitchError -= 360;
    pitchError = Math.abs(pitchError);

    SmartDashboard.putBoolean("DOD_MetDistance", mDistanceMet);
    SmartDashboard.putBoolean("DOD_VisionDistToTarget", mVision.getRange() < mDistanceToTarget);
    SmartDashboard.putBoolean("DOD_PitchOnTarget", pitchError < 5.0);

    return (mDistanceMet
        // !mDrive.seesDefense()
        // && (mVision.getRange() < mDistanceToTarget)
        && (pitchError < 5.0))
        || (Math.abs(mNavigation.getAverageEncoderDistance() - mStartingEncoderValue) > 3
            * mMinDistance);
  }

  /**
   * Loads the PID multipliers and the angle and distance tolerances
   */
  @Override
  public void loadProperties() {
    PropertySet mPropertySet = PropertySet.getInstance();
    double kp = mPropertySet.getDoubleValue("distanceKp", 0.05);
    double ki = mPropertySet.getDoubleValue("distanceKi", 0.0);
    double kd = mPropertySet.getDoubleValue("distanceKd", 0.0);
    super.setPID(kp, ki, kd);
    kp = mPropertySet.getDoubleValue("angleKp", 0.01);
    mTurnPID.setPID(kp, 0.0, 0.0);
  }

  @Override
  public String toString() {
    return "DriveDistanceController";
  }
}

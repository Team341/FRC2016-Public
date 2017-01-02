package missdaisy.subsystems;

import missdaisy.Constants;
import missdaisy.fileio.PropertySet;
import missdaisy.loops.Navigation;
import missdaisy.loops.controllers.DriveTurnController;
import missdaisy.utilities.AlphaFilter;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The drive base of the robot. Has methods to set the speed of the left and right motors.
 * 
 * @author Josh Sizer
 */
public final class Drive extends DaisySubsystem {
  private static Drive driveInstance = null;
  private Talon mLeftDriveMotor;
  private Talon mRightDriveMotor;
  private AlphaFilter mAlphaFilter;
  private DigitalInput mDefenseSensor;
  private DigitalInput mDriveRailSensor;
  private boolean mUseAlphaFilter = true;

  /**
   * Gets the instance of the drive base. Used in order to never have more than one drive base
   * object, ever.
   * 
   * @return The one and only instance of the drive base
   */
  public static Drive getInstance() {
    if (driveInstance == null)
      driveInstance = new Drive();
    return driveInstance;
  }

  private Drive() {
    mLeftDriveMotor = new Talon(Constants.PWMs.DRIVE_LEFT_MOTOR);
    mRightDriveMotor = new Talon(Constants.PWMs.DRIVE_RIGHT_MOTOR);
    // either the left or the right drive will be backwards
    mRightDriveMotor.setInverted(true);
    mAlphaFilter = new AlphaFilter(Constants.Properties.DRIVE_ALPHA_FILTER_GAIN_INTAKE_UP);
    mDefenseSensor = new DigitalInput(Constants.DigitalInputs.DEFENSE_BANNER);
    mDriveRailSensor = new DigitalInput(Constants.DigitalInputs.DRIVE_RAIL_LIMIT);
    loadProperties();
  }

  /**
   * Set the speed of each motor individually. Useful for tank drive or any other case that each
   * motor must be set precisely and separate from each other
   * 
   * @param leftMotorSpeed a double between -1.0 and 1.0, representing full forward or full reverse
   *        for the left motor.
   * @param rightMotorSpeed a double between -1.0 and 1.0, representing full forward or full reverse
   *        for the right motor.
   */
  public void setSpeed(double leftMotorSpeed, double rightMotorSpeed) {
    set(leftMotorSpeed, rightMotorSpeed);
  }

  /**
   * Set the speed and the turn of the robot, rather than each motor individually. Useful for arcade
   * drive.
   * 
   * @param speed a double between -1.0 and 1.0, representing the full forward or full reverse.
   * @param turn a double between -1.0 and 1.0, representing turn anti-clockwise or clockwise.
   */
  public void setSpeedTurn(double speed, double turn) {
    if (mUseAlphaFilter) {
      speed = mAlphaFilter.calculate(speed);
    }

    double leftMotorSpeed = speed + turn;
    double rightMotorSpeed = speed - turn;
    set(leftMotorSpeed, rightMotorSpeed);
  }

  /**
   * Tells the drive to use the alpha filter or not to limit acceleration
   * @param use
   */
  public void useAlphaFilter(boolean use) {
    mUseAlphaFilter = use;
  }

  /**
   * Set's the alpha gain for the alpha filter
   * @param alpha The alpha gain
   */
  public void setAlpha(double alpha) {
    mAlphaFilter.setAlpha(alpha);
  }

  /**
   * This is to simplify other methods that set the motor speed. The arguments can only be values
   * from -1.0 to 1.0, a scaled number representing the output voltage of the speed controller (-12v
   * to 12v).
   */
  private void set(double leftMotorSpeed, double rightMotorSpeed) {
    mLeftDriveMotor.set(leftMotorSpeed);
    mRightDriveMotor.set(rightMotorSpeed);
  }

  /**
   * Detects if there is a defense present under the robot.
   * 
   * @return True if there is a defense present under the robot. False if there is not.
   */
  public boolean seesDefense() {
    return mDefenseSensor.get();
  }

  public boolean stuckOnDefense() {
    return !mDriveRailSensor.get();
  }

  /**
   * Set's the robot to a known state of stopped!
   */
  public synchronized void reset() {
    set(0.0, 0.0);
  }

  public void loadProperties() {
    // Nothing to load!
  }

  /**
   * Logs this subsystem's specific properties to smartdashboard. 
   */
  public void logToDashBoard() {
    SmartDashboard.putNumber("DriveLeftMotorOutput", mLeftDriveMotor.get());
    SmartDashboard.putNumber("DriveRightMotorOutput", mRightDriveMotor.get());
    SmartDashboard.putBoolean("DriveTurnOnTarget", DriveTurnController.getInstance().onTarget());
    SmartDashboard.putBoolean("DriveSeesDefense", seesDefense());
    SmartDashboard.putBoolean("DriveStuckOnDefense", stuckOnDefense());

    if (getCurrentController() != null)
      SmartDashboard.putString("DriveCurrentController", getCurrentController().toString());
    else
      SmartDashboard.putString("DriveCurrentController", "OpenLoop");
  }
}

package missdaisy.loops.controllers;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import missdaisy.Constants;
import missdaisy.fileio.PropertySet;
import missdaisy.loops.SynchronousPID;
import missdaisy.subsystems.Shooter;
import missdaisy.utilities.MovingAverageFilter;

/**
 * Controls the RPM of the shooter by using PID calculations.
 * 
 */
public class ShooterSpeedController extends SynchronousPID implements Controller {
  private static ShooterSpeedController shooterSpeedControllerInstance = null;
  private Shooter mShooter = Shooter.getInstance();
  private double mShooterMotorOutput;
  private double shooterRPMTolerance;
  private double kffv = 1 / 5000.0;
  private boolean useSmartDashboardValues = true;
  private boolean onTarget = false;
  private int onTargetCounter = 0;
  private MovingAverageFilter mAverageOutput;

  /**
   * Gets the instance of the shooter speed controller. Used in order to never have more than one
   * shooter speed controller object, ever.
   * 
   * @return The one and only instance of the shooter speed controller
   */
  public static ShooterSpeedController getInstance() {
    if (shooterSpeedControllerInstance == null)
      shooterSpeedControllerInstance = new ShooterSpeedController();
    return shooterSpeedControllerInstance;
  }

  private ShooterSpeedController() {
    SmartDashboard.putNumber("ShooterkP", 0.005);
    SmartDashboard.putNumber("ShooterkI", 0.0);
    SmartDashboard.putNumber("ShooterkD", 0.001);
    loadProperties();
    reset();
    mAverageOutput = new MovingAverageFilter(50);
    mShooter = Shooter.getInstance();
  }

  /**
   * Set the desired shooter RPM
   * 
   * @param shooterRPM the desired shooter RPM
   */
  public synchronized void setGoal(double shooterRPM) {
    super.setSetpoint(shooterRPM);
    loadProperties();
  }

  /**
   * Ramps up the shooter to the desired RPM
   */
  @Override
  public synchronized void run() {
    /*
     * mShooterMotorOutput = calculate(mShooter.getRPM());
     * SmartDashboard.putNumber("ShooterMotorPIDCommand", mShooterMotorOutput);
     * SmartDashboard.putNumber("ShooterMotorOutput", Math.max(0.0, mShooterMotorOutput +
     * kffv*getSetpoint())); mShooter.setSpeed(Math.max(0.0, mShooterMotorOutput +
     * kffv*getSetpoint()));
     */
    if (mShooter.getRPM() > super.getSetpoint()) {
      mShooterMotorOutput = 0.0;
    } else if (mShooter
        .getRPM() < (super.getSetpoint() - Constants.Properties.PID_SHOOTER_RPM_TOLERANCE)) {
      mShooterMotorOutput = 1.0;
    }
    mShooter.setSpeed(mShooterMotorOutput);

    onTarget = Math
        .abs(getSetpoint() - mShooter.getRPM()) < Constants.Properties.PID_SHOOTER_RPM_TOLERANCE;

    // if (onTarget) {
    // onTargetCounter++;

    mAverageOutput.setInput(mShooterMotorOutput);
    // }

    /*
     * //if (onTarget()) { // mAverageOutput.setInput(mShooterMotorOutput); //if (onTargetCounter >
     * 50){ mShooter.setSpeed(mAverageOutput.run()); //} else { //
     * mShooter.setSpeed(mShooterMotorOutput); //}
     */
    SmartDashboard.putNumber("ShooterError", getSetpoint() - mShooter.getRPM());

    // onTarget = Math.abs(getSetpoint() - mShooter.getRPM()) <
    // Constants.kDefaultShooterRPMTolerance;

    /*
     * if (mShooter.getRPM() < 2500) { onTarget = false; }
     */
  }

  /**
   * Returns true if the shooter is within the accepted tolerance for the RPM
   */
  @Override
  public synchronized boolean onTarget() {
    SmartDashboard.putBoolean("ShooterRPMMet", onTarget);

    return onTarget;
  }

  @Override
  public void reset() {
    super.reset(); // resets the PID terms
    mShooterMotorOutput = 0.0;
    onTargetCounter = 0;
    setGoal(0.0);
    onTarget = false;
  }

  /**
   * Loads the PID multipliers and shooter RPM tolerance
   */
  @Override
  public void loadProperties() {
    if (useSmartDashboardValues) {
      double kp = SmartDashboard.getNumber("ShooterkP", 0.1);
      double ki = SmartDashboard.getNumber("ShooterkI", 0.0);
      double kd = SmartDashboard.getNumber("ShooterkD", 0.0);
      setPID(kp, ki, kd);
    } else {
      PropertySet mPropertySet = PropertySet.getInstance();
      double kp = mPropertySet.getDoubleValue("shooterKp", 0.008);
      double ki = mPropertySet.getDoubleValue("shooterKi", 0.0000);
      double kd = mPropertySet.getDoubleValue("shooterKd", 0.0001);
      shooterRPMTolerance = mPropertySet.getDoubleValue("shooterRPMTolerance",
          Constants.Properties.PID_SHOOTER_RPM_TOLERANCE);
      setPID(kp, ki, kd);
    }
  }

  @Override
  public String toString() {
    return "ShooterSpeedController";
  }
}

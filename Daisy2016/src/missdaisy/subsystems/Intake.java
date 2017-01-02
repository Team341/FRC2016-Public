package missdaisy.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import missdaisy.Constants;

/**
 * The intake of the robot which acquires boulders from the ground
 * 
 * @author Josh Sizer
 *
 */
public final class Intake extends DaisySubsystem {
  private static Intake intakeInstance = null;
  private static Popper mPopper;
  private Victor mIntakeMotor;
  private Victor mConveyorMotor;
  private DoubleSolenoid mIntakePiston;
  private DigitalInput mBallSensor;

  /**
   * Gets the instance of the intake. Used in order to never have more than one intake object, ever.
   * 
   * @return The one and only instance of the intake
   */
  public static Intake getInstance() {
    if (intakeInstance == null)
      intakeInstance = new Intake();
    return intakeInstance;
  }

  private Intake() {
    mPopper = new Popper();
    mIntakeMotor = new Victor(Constants.PWMs.INTAKE_MOTOR);
    mIntakeMotor.setInverted(false);
    mIntakePiston = new DoubleSolenoid(Constants.Solenoids.INTAKE_DOUBLE_1,
        Constants.Solenoids.INTAKE_DOUBLE_2);
    mConveyorMotor = new Victor(Constants.PWMs.CONVEYOR_MOTOR);
    mConveyorMotor.setInverted(true);
    mBallSensor = new DigitalInput(Constants.DigitalInputs.CONVEYOR_BANNER);
  }

  // because the Popper can be thought of its own little subsystem,
  // but it is not worth making its own class for it
  private final class Popper {
    // a double solenoid will retain it's state even after the robot is disabled
    DoubleSolenoid mPopperPiston = null;
    private boolean isDeployed = false;

    private Popper() {
      mPopperPiston = new DoubleSolenoid(Constants.Solenoids.POPPER_DOUBLE_1,
          Constants.Solenoids.POPPER_DOUBLE_2);
    }

    private void deploy() {
      mPopperPiston.set(DoubleSolenoid.Value.kForward);
      isDeployed = true;
    }

    private void retract() {
      mPopperPiston.set(DoubleSolenoid.Value.kReverse);
      isDeployed = false;
    }

    private boolean isDeployed() {
      return isDeployed;
    }
  }

  /**
   * Set the speed of the intake's motors in order to pick up a ball
   * 
   * @param speed a value between 1.0 and -1.0, indication intake or spit
   */
  public void setIntakeSpeed(double speed) {
    mIntakeMotor.set(speed);
  }

  public void setConveyorSpeed(double speed) {
    mConveyorMotor.set(speed);
  }

  /**
   * Puts the intake to the floor
   */
  public void deploy() {
    mIntakePiston.set(DoubleSolenoid.Value.kReverse);
  }

  /**
   * Pulls the intake off the floor
   */
  public void retract() {
    mIntakePiston.set(DoubleSolenoid.Value.kForward);
  }

  /**
   * Detects if there is a ball present in the robot.
   * 
   * @return True if there is a ball present in the shooter. False if there is not.
   */
  public boolean seesBall() {
    return mBallSensor.get();
  }

  /**
   * Manually set the position of the intake
   * 
   * @param position True for deploy. False for retract.
   */
  public void setPosition(boolean position) {
    if (position)
      deploy();
    else
      retract();
  }

  public void deployPopper() {
    mPopper.deploy();
  }

  public void retractPopper() {
    mPopper.retract();
  }

  public void logToDashboard() {
    SmartDashboard.putBoolean("ConveyorSeesBall", mBallSensor.get());

    if (mIntakePiston.get() == DoubleSolenoid.Value.kForward)
      SmartDashboard.putString("IntakePosition", "Deployed");
    else if (mIntakePiston.get() == DoubleSolenoid.Value.kReverse)
      SmartDashboard.putString("IntakePosition", "Retracted");
  }

  public synchronized void reset() {
    setIntakeSpeed(0.0);
    setConveyorSpeed(0.0);
    retract();
  }
}

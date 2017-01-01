package missdaisy.subsystems;

import missdaisy.Constants;
import missdaisy.loops.controllers.ShooterSpeedController;
import missdaisy.utilities.MovingAverageFilter;
import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.Counter;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The shooter of the robot which launches boulders into goals
 * 
 * @author Josh Sizer
 *
 */
public final class Shooter extends DaisySubsystem {
  private static Shooter shooterInstance = null;
  private Hood mHood = null;
  private Barrier mBarrier = null;
  private Intake mIntake;
  private CANTalon mWheelMotor;
  private Solenoid mBallLight;
  private Solenoid mStatusLight;
  private Solenoid mVisionLight;
  private Counter mWheelCounter;
  private MovingAverageFilter mCounterFilter;
  private int kCountsPerRev = 2;
  private int blinkCounter = 0;
  private static final boolean BATTER_POSITION = true;
  private static final boolean OUTERWORKS_POSITION = false;

  /**
   * Gets the instance of the shooter. Used in order to never have more than one shooter object,
   * ever.
   * 
   * @return The one and only instance of the shooter
   */
  public static Shooter getInstance() {
    if (shooterInstance == null)
      shooterInstance = new Shooter();
    return shooterInstance;
  }

  private Shooter() {
    mHood = new Hood();
    mBarrier = new Barrier();
    mIntake = Intake.getInstance();
    mWheelMotor = new CANTalon(Constants.CAN.SHOOTER_TALONSRX_ID);
    mWheelMotor.setInverted(true);
    mWheelCounter = new Counter(Constants.DigitalInputs.SHOOTER_BANNER);
    mBallLight = new Solenoid(Constants.Solenoids.BALL_LIGHT);
    mStatusLight = new Solenoid(Constants.Solenoids.STATUS);
    // mVisionLight = new Solenoid(Constants.Solenoids.VISION);
    mCounterFilter = new MovingAverageFilter(10);
  }

  // because the Hood can be thought of its own little subsystem,
  // but it is not worth making its own class for it
  private final class Hood {
    Solenoid mHoodPiston = null;

    private Hood() {
      mHoodPiston = new Solenoid(Constants.Solenoids.HOOD);
    }

    private void setBatterPosition() {
      mHoodPiston.set(BATTER_POSITION);
    }

    private void setOuterworksPosition() {
      mHoodPiston.set(OUTERWORKS_POSITION);
    }

    private boolean get() {
      return mHoodPiston.get();
    }
  }

  /**
   * This is a tiny class for the "POW Sticks" which encourage defenders to stay a larger distance
   * away from the our robot in order to not encur foul points. 
   * @author Adam N.
   *
   */
  private final class Barrier {
    Servo mLeftServo = null;
    Servo mRightServo = null;

    private Barrier() {
      mLeftServo = new Servo(Constants.SERVOs.BARRIER_LEFT);
      mRightServo = new Servo(Constants.SERVOs.BARRIER_RIGHT);
    }

    private void deployBarrier() {
      mLeftServo.setAngle(0);
      mRightServo.setAngle(90);
    }

    private void retractBarrier() {
      mLeftServo.setAngle(90);
      mRightServo.setAngle(0);
    }
  }

  /**
   * Sets the conveyor on in order to shoot
   */
  public void shoot() {
    mIntake.setConveyorSpeed(1.0);
  }

  /**
   * Sets the shooter hood to be in the position to shoot from the batter, ie very steep angle
   */
  public void setBatterPosition() {
    mHood.setBatterPosition();
  }

  /**
   * Sets the shooter hood to be in the position to shoot from the outer works, ie shallow angle
   */
  public void setOuterworksPosition() {
    mHood.setOuterworksPosition();
  }

  public boolean isHoodBatterPosition() {
    return mHood.get() == BATTER_POSITION;
  }

  public boolean isHoodOuterworksPosition() {
    return mHood.get() == OUTERWORKS_POSITION;
  }

  public void deployBarrier() {
    mBarrier.deployBarrier();
  }

  public void retractBarrier() {
    mBarrier.retractBarrier();
  }

  /**
   * Set the speed of the shooter's motor.
   * 
   * @param motorSpeed A value between -1.0 and 1.0, representing full reverse and full forward.
   */
  public void setSpeed(double motorSpeed) {
    mWheelMotor.set(motorSpeed);
  }

  /**
   * Turns the LEDs on the status light on the shooter on or off.
   * 
   * @param on True for on, false for off
   */
  public void setBallLightState(boolean on) {
    mBallLight.set(on);
  }

  /**
   * Turns the LEDs on the status light on the shooter on or off.
   * 
   * @param on True for on, false for off
   */
  public void setStatusLightState(boolean on) {
    mStatusLight.set(on);
  }

  /**
   * Turns the green vision light on or off
   * 
   * @param on True for on, false for off
   */
  public void setVisionLightState(boolean on) {
    // mVisionLight.set(on);
  }

  public void lightShow() {
    if (blinkCounter == 1)
      setStatusLightState(true);
    else if (blinkCounter == 10)
      setStatusLightState(false);
    else if (blinkCounter == 20)
      setBallLightState(true);
    else if (blinkCounter == 30)
      setBallLightState(false);
    else if (blinkCounter == 40) {
      setStatusLightState(true);
      setBallLightState(true);
    } else if (blinkCounter == 50) {
      setStatusLightState(false);
      setBallLightState(false);
    } else if (blinkCounter == 59)
      blinkCounter = 0;

    blinkCounter++;
  }


  /**
   * Gets the average shooter RPM
   * 
   * @return the shooter's average RPM
   */
  public double getRPM() {
    return mCounterFilter.getAverage();
  }

  /**
   * Finds the average RPM of the shooter based on a specific sample size
   */
  public void runInputFilters() {
    // 1 rev 60 sec 60
    // ---------------- * ------ = ----- rpm

    // period (#sec/rev) 1 min period

    // multiply period by 2 because one period is half a revolution
    double rpm = 60.0 / (mWheelCounter.getPeriod() * (double) kCountsPerRev);
    if (rpm < 8000.0) {
      // This check ensures that wild spurious readings are rejected
      mCounterFilter.setInput(rpm);
      mCounterFilter.run();
    }
  }

  public double getCurrent() {
    return mWheelMotor.getOutputCurrent();
  }



  /**
   * Turns the shooter off and resets the moving average filter
   */
  public synchronized void reset() {
    setSpeed(0.0);
    mHood.setOuterworksPosition();
    mCounterFilter.reset();
  }

  public synchronized void loadProperties() {

  }

  public void logToDashboard() {
    SmartDashboard.putNumber("ShooterRPM", getRPM());
    SmartDashboard.putBoolean("ShooterRPMOnTarget",
        ShooterSpeedController.getInstance().onTarget());
    SmartDashboard.putString("ShooterHoodPosition",
        mHood.get() == BATTER_POSITION ? "Batter" : "Outerworks");
    if (getCurrentController() != null)
      SmartDashboard.putString("ShooterCurrentController", getCurrentController().toString());
    else
      SmartDashboard.putString("ShooterCurrentController", "OpenLoop");
  }
}

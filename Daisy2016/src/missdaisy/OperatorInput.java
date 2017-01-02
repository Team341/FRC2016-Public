package missdaisy;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import missdaisy.loops.Navigation;
import missdaisy.loops.controllers.AutoAimDriveController;
import missdaisy.loops.controllers.AutoAimShooterController;
import missdaisy.loops.controllers.DriveTurnController;
import missdaisy.loops.controllers.ShooterSpeedController;
import missdaisy.subsystems.*;
import missdaisy.utilities.DaisyMath;
// import missdaisy.utilities.DaisyUSBCamera;
import missdaisy.utilities.RobotVisionDualUSB;
import missdaisy.utilities.XboxController;

/**
 * All the input from the operator and driver. This is the bread and butter of teleop mode
 *
 */
public class OperatorInput {
  private static OperatorInput operatorInputInstance = null;
  protected XboxController mDriverController;
  protected XboxController mOperatorController;

  // subsystems
  private Drive mDrive;
  private Shooter mShooter;
  private Intake mIntake;
  private Hanger mHanger;
  // private DaisyUSBCamera mCameras;
  private RobotVisionDualUSB mDualUSB;

  // controllers
  private AutoAimDriveController mAutoAimDrive;
  private AutoAimShooterController mAutoAimShooter;
  private ShooterSpeedController mShooterSpeed;

  // operator input values
  private double mLeftMotorSpeed;
  private double mRightMotorSpeed;
  private double mSpeed;
  private double mTurn;
  private double mIntakeSpeed;
  private double mManualShooterSpeed;
  private double shooterRPMBatter;
  private double shooterRPMOuterWorks;
  private double mSpeedDecrease = 1.0;
  private long mShootStartTime = 0;

  private boolean lastBackButtonState = false;
  private boolean lastStartButtonState = false;
  private boolean triedToHang = false;
  private double turnOutputValue = 0.0;

  // some booleans that control robot decisions
  boolean mIsAutoAimControllers = false;
  boolean mIsAutoAimOnTarget = false;
  boolean mReadyToShoot = false;
  boolean mResetHeading = true;

  boolean lastLeftStickClick = false;
  boolean lastRightStickClick = false;
  boolean hangingStarted = false;
  int deployingHangerCounter = 0;

  private boolean mArcadeDrive = true;

  private boolean mIsShooterOnTarget;

  public static OperatorInput getInstance() {
    if (operatorInputInstance == null)
      operatorInputInstance = new OperatorInput();
    return operatorInputInstance;
  }

  private OperatorInput() {
    mDriverController = new XboxController(Constants.XBOXController.DRIVER_PORT);
    mOperatorController = new XboxController(Constants.XBOXController.OPERATOR_PORT);
    mDrive = Drive.getInstance();
    mShooter = Shooter.getInstance();
    mIntake = Intake.getInstance();
    mHanger = Hanger.getInstance();
    // mCameras = new DaisyUSBCamera();
    // mDualUSB = RobotVisionDualUSB.getInstance();
    mAutoAimDrive = AutoAimDriveController.getInstance();
    mAutoAimShooter = AutoAimShooterController.getInstance();
    mShooterSpeed = ShooterSpeedController.getInstance();
    SmartDashboard.putNumber("DesiredShooterRPMBatter", 2750);
    SmartDashboard.putNumber("DesiredShooterRPMOuterWorks", 4250);
    SmartDashboard.putNumber("shootervoltage", 0.1);
  }

  public void processInputs() {
    /*
     * if (mDriverController.getLeftStickClick()) mCameras.setBackCamera(); else if
     * (mDriverController.getRightStickClick()) mCameras.setFrontCamera();
     * 
     * mCameras.run();
     */

    // Check if we need to toggle camera views
    /*
     * if ((mDriverController.getLeftStickClick() && !lastLeftStickClick) ||
     * (mDriverController.getRightStickClick() && !lastRightStickClick)){
     * //mDualUSB.switchCameras(); } lastLeftStickClick = mDriverController.getLeftStickClick();
     * lastRightStickClick = mDriverController.getRightStickClick();
     */

    if (mDriverController.getBackButton() && !lastBackButtonState) {
      turnOutputValue -= 0.01;
    } else if (mDriverController.getStartButton() && !lastStartButtonState) {
      turnOutputValue += 0.01;
    }
    lastBackButtonState = mDriverController.getBackButton();
    lastStartButtonState = mDriverController.getStartButton();
    SmartDashboard.putNumber("turnOutputValue", turnOutputValue);

    // Driver Inputs
    mLeftMotorSpeed = -1.0 * DaisyMath.applyDeadband(mDriverController.getLeftYAxis(),
        Constants.XBOXController.DEAD_BAND);

    mRightMotorSpeed = -1.0 * DaisyMath.applyDeadband(mDriverController.getRightYAxis(),
        Constants.XBOXController.DEAD_BAND);

    mSpeed = mLeftMotorSpeed;
    mTurn = 0.7 * DaisyMath.applyDeadband(mDriverController.getRightXAxis(),
        Constants.XBOXController.DEAD_BAND);

    // Operator Inputs
    mIntakeSpeed = -1 * DaisyMath.applyDeadband(mOperatorController.getRightYAxis(),
        Constants.XBOXController.DEAD_BAND);

    mManualShooterSpeed = -1 * DaisyMath.applyDeadband(mOperatorController.getLeftYAxis(),
        Constants.XBOXController.DEAD_BAND + 0.3);

    shooterRPMBatter = SmartDashboard.getNumber("DesiredShooterRPMBatter", 3000);
    shooterRPMOuterWorks = SmartDashboard.getNumber("DesiredShooterRPMOuterWorks", 5000);

    if (mOperatorController.getLeftStickClick() && !lastLeftStickClick) {
      shooterRPMOuterWorks += 250;
    } else if (mOperatorController.getRightStickClick() && !lastRightStickClick) {
      shooterRPMOuterWorks -= 250;
    }
    SmartDashboard.putNumber("DesiredShooterRPMOuterWorks", shooterRPMOuterWorks);
    lastLeftStickClick = mOperatorController.getLeftStickClick();
    lastRightStickClick = mOperatorController.getRightStickClick();

    // Checks if the auto aim is on target
    // as long as the current controllers are the auto aim ones,
    // and they are on target, set mReadyToShoot to true

    mIsAutoAimControllers = mDrive.getCurrentController() == mAutoAimDrive
        && mShooter.getCurrentController() == mAutoAimShooter;

    mIsAutoAimOnTarget = mAutoAimDrive.onTarget() && mAutoAimShooter.onTarget();
    mIsShooterOnTarget =
        mShooterSpeed.onTarget() && mShooter.getCurrentController() == mShooterSpeed;

    mReadyToShoot = (mIsAutoAimControllers && mIsAutoAimOnTarget) || mIsShooterOnTarget;
    SmartDashboard.putBoolean("ReadyToShoot", mReadyToShoot);

    /**
     * Driver Inputs: 1) Left and right sticks: moving robot 2) LT: Auto-Aim 3) RT: Hold for no
     * output filtering for the drive (disable smooth acceleration) 4) LB: Half speed 5) RB: Full
     * speed 6) A: Tank Drive 7) B: Arcade Drive
     * 
     */
    mDrive.setAlpha(Constants.Properties.DRIVE_ALPHA_FILTER_GAIN_INTAKE_DOWN);
    if (mDrive.getCurrentController() == null) {

      if (mDriverController.getRightTrigger()) {
        mArcadeDrive = true;
        mSpeedDecrease = 0.25;
      } else if (mDriverController.getRB()) {
        mArcadeDrive = false;
        mSpeedDecrease = 0.5;
      } else {
        mArcadeDrive = true;
        mSpeedDecrease = 1.0;
      }

      // if (mDriveController.getLB())
      // mSpeedDecrease = 0.5;
      // else if (mDriveController.getRB())
      // mSpeedDecrease = 1.0;
      if (mDriverController.getBButton()) {
        mDrive.useAlphaFilter(false);
        mDrive.setSpeedTurn(Constants.Properties.DRIVE_BREAK_SPEED, 0.0);
      } else if (mDriverController.getXButton()) {
        // mDrive.setSpeedTurn(0.0, turnOutputValue);
        mDrive.setCurrentController(DriveTurnController.getInstance());
      } else {
        if (mDriverController.getLB())
          mDrive.useAlphaFilter(false);
        else
          mDrive.useAlphaFilter(true);

        if (mArcadeDrive)
          mDrive.setSpeedTurn(mSpeed, mSpeedDecrease * mTurn);
        else
          mDrive.setSpeed(mSpeedDecrease * mLeftMotorSpeed, mSpeedDecrease * mRightMotorSpeed);
      }
    } else {
      mDrive.useAlphaFilter(false);
    }

    /**
     * Operator Inputs: 1) RT: Shoot (if the auto-aim is on target) 2) LT: Rev shooter (based on
     * hood position) 3) RB: Hood down (outerworks position) 4) LB: Hood up (batter position) 5)
     * Right Stick: Intake in and out 6) Left Stick: Hanger Winch 7) A-Button: Intake to the floor
     * (deploy) 8) B-Button: Intake up (retract) 9) X-Button: Extend hanger arms (deploy) 10)
     * Y-Button: Retract hanger arms (retract)
     */
    if (!mIntake.seesBall() || mIntakeSpeed < 0.0) {
      mIntake.setIntakeSpeed(mIntakeSpeed);
      mIntake.setConveyorSpeed(Math.min(mIntakeSpeed, 0.6));
    } else if (mReadyToShoot) {
      if (mOperatorController.getRightTrigger())
        mShooter.shoot();
    } else {
      mIntake.setIntakeSpeed(0.0);
      mIntake.setConveyorSpeed(0.0);
    }

    // intake piston
    if (mOperatorController.getAButton()) {
      mIntake.deploy();
      mDrive.setAlpha(Constants.Properties.DRIVE_ALPHA_FILTER_GAIN_INTAKE_UP);
    } else if (mOperatorController.getBButton()) {
      mIntake.retract();
      mDrive.setAlpha(Constants.Properties.DRIVE_ALPHA_FILTER_GAIN_INTAKE_DOWN);
    }

    // shooter hood
    if (mOperatorController.getRB())
      mShooter.setBatterPosition();
    else if (mOperatorController.getLB())
      mShooter.setOuterworksPosition();

    // hanger piston
    if (mOperatorController.getXButton()) {
      mHanger.deploy();
      deployingHangerCounter = 0;
    } else if (mOperatorController.getYButton() || deployingHangerCounter > 0) {
      deployingHangerCounter++;
      mIntake.retract();
      mShooter.setOuterworksPosition();
      if (deployingHangerCounter > 5) {
        mHanger.retract();
      }
    }


    // set the speed of the winch
    if (mOperatorController.getStartButton()) {
      mHanger.setSpeed(1.0);
      triedToHang = true;
    } else if (mOperatorController.getBackButton()) {
      mHanger.setSpeed(-1.0);
      triedToHang = true;
    } else {
      mHanger.setSpeed(0.0);
    }

    // this is where drive controller logic must go
    // if none of the statements are satisfied, then drive
    // will be an open loop

    // Handle drive controller
    if (mDriverController.getLeftTrigger()) { // auto aim
      mDrive.setCurrentController(mAutoAimDrive);
      // } else if (mDriverController.getXButton()) {
      // mDrive.setCurrentController(DriveTurnController.getInstance());
    } else {
      mDrive.setOpenLoop();
      DriveTurnController.getInstance().reset();
      DriveTurnController.getInstance().setGoal(
          DaisyMath.boundAngle0to360Degrees(Navigation.getInstance().getHeadingInDegrees() - 20));
    }


    // Handle shooter controller, separated out from drive controller logic because there are times
    // when they
    // are independent and certain button combinations would prevent the shooter from being manually
    // revved.
    if (mShooter.isHoodOuterworksPosition()) {
      mShooterSpeed.setGoal(shooterRPMOuterWorks);
      // mShooterSpeed.setGoal(Constants.kDefaultShooterRPMOuterWorks);
    } else if (mShooter.isHoodBatterPosition()) {
      mShooterSpeed.setGoal(shooterRPMBatter);
      // mShooterSpeed.setGoal(Constants.kDefaultShooterRPMBatter);
    }

    if (mDriverController.getLeftTrigger()) { // auto aim
      mShooter.setCurrentController(mAutoAimShooter);
      if (mShooter.isHoodOuterworksPosition())
        mShooter.deployBarrier();

    } else if (mOperatorController.getLeftTrigger()) {
      mShooter.setCurrentController(mShooterSpeed);

      if (mShooter.isHoodOuterworksPosition())
        mShooter.deployBarrier();
      // double newSpeed = SmartDashboard.getNumber("shootervoltage", 0.1);
      // mShooter.setSpeed(newSpeed);
    } else {
      mShooterSpeed.reset();
      mShooter.setOpenLoop();
      mShooter.setSpeed(mManualShooterSpeed);
      mAutoAimShooter.setGoal(); // based on hood position

      if (mShooter.isHoodOuterworksPosition())
        mShooter.retractBarrier();
      // Set parameters for all controllers
      /*
       * if (mShooter.isHoodOuterworksPosition()) { //mShooterSpeed.setGoal(shooterRPMOuterWorks);
       * mShooterSpeed.setGoal(Constants.Properties.SHOOTER_RPM_OUTERWORKS); } else if
       * (mShooter.isHoodBatterPosition()) { //mShooterSpeed.setGoal(shooterRPMBatter);
       * mShooterSpeed.setGoal(Constants.Properties.SHOOTER_RPM_BATTER); }
       * 
       */
      /*
       * // this might brake something if (mManualShooterSpeed > 0.0) {
       * mShooter.setSpeed(mManualShooterSpeed); mIntake.setConveyorSpeed(mIntakeSpeed); } else {
       * mShooter.setSpeed(0.0); }
       */
    }

    // manual green vision light control

    // if (mDriveController.getXButton() ||
    // mDriveController.getLeftTrigger())
    // mShooter.setVisionLightState(true);
    // else
    // mShooter.setVisionLightState(false);


    if (mDriverController.getYButton()) {
      // mShooter.setStatusLightState(true);
      mShooter.lightShow();
    } else {
      mShooter.setStatusLightState(mReadyToShoot);
      if (mIntake.seesBall())
        mShooter.setBallLightState(true);
      else
        mShooter.setBallLightState(false);
    }

    if (mDriverController.getAButton()) {
      mIntake.deployPopper();
    } else {
      mIntake.retractPopper();
    }

    /*
     * if (mShooter.getRPM() > 1500.0 || (mOperatorController.getLeftYAxis() >
     * Constants.XBOXController.DEAD_BAND && mOperatorController.getRightYAxis() >
     * Constants.XBOXController.DEAD_BAND)) { //|| (mShooter.getCurrent() > 0.2 &&
     * mShooter.getCurrentController() == null)) { if (mShooter.getCurrentController() ==
     * mShooterSpeed && mOperatorController.getRightTrigger()) { mShooter.shoot(); } else {
     * //mIntake.setConveyorSpeed(mIntakeSpeed); } }
     */
    if (mReadyToShoot || triedToHang || mShootStartTime > 0) {
      if (mOperatorController.getRightTrigger()) {
        mShootStartTime = System.currentTimeMillis();
      }

      if (System.currentTimeMillis() - mShootStartTime > 1000) {
        mShootStartTime = 0;
      } else {
        mShooter.shoot();
      }
    }
    /*
     * if (mSpeed >= 0.0) {
     * DaisyUSBCamera.getInstance().setCamera(DaisyUSBCamera.getInstance().FRONT_CAMERA); } else {
     * DaisyUSBCamera.getInstance().setCamera(DaisyUSBCamera.getInstance().BACK_CAMERA); }
     * DaisyUSBCamera.getInstance().run();
     */
  }
}

/*
 * Useful for telling the turn controller a certain angle to turn to if (mDriveController.getRB())
 * mDrive.setCurrentController(DriveTurnController.getInstance()); else {
 * DriveTurnController.getInstance().reset(); DriveTurnController.getInstance().setGoal(
 * DaisyMath.boundAngle0to360Degrees( Navigation.getInstance().getHeadingInDegrees() + 90));
 * mDrive.setOpenLoop(); }
 */

/*
 * mIsAutoAimControllers = (mDrive.getCurrentController() == mAutoAimDrive &&
 * mShooter.getCurrentController() == mAutoAimShooter) || mShooter.getCurrentController() ==
 * mShooterSpeed; mIsAutoAimOnTarget = (mAutoAimDrive.onTarget() || mShooter.isHoodBatterPosition())
 * && (mAutoAimShooter.onTarget() || mShooterSpeed.onTarget()); mReadyToShoot =
 * mIsAutoAimControllers && mIsAutoAimOnTarget; SmartDashboard.putBoolean("ReadyToShoot",
 * mReadyToShoot);
 */


package missdaisy;

import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.vision.USBCamera;
import missdaisy.autonomous.AutonomousParser;
import missdaisy.autonomous.StateMachine;
import missdaisy.fileio.PropertyReader;
import missdaisy.loops.FastLoopTimer;
import missdaisy.loops.Navigation;
import missdaisy.loops.controllers.DriveDistanceController;
import missdaisy.loops.controllers.DriveTurnController;
import missdaisy.loops.controllers.ShooterSpeedController;
import missdaisy.subsystems.Drive;
import missdaisy.subsystems.Hanger;
import missdaisy.subsystems.Intake;
import missdaisy.subsystems.Shooter;
// import missdaisy.utilities.DaisyUSBCamera;
import missdaisy.utilities.RobotVisionDualUSB;

/**
 * The robot. This is the entry point for all functions and threads that are spawned. This class has
 * some key methods which include the autonomousPeriodic, teleopPeriodic, disabledPeriodic, and
 * testPeriodic. These are the functions that get called during that specific time during the match,
 * and are called every 20 ms.
 * 
 */
public class Daisy2016 extends IterativeRobot {
  // This reads in properties from some file location on the roborio.
  private PropertyReader mPropertyReader;
  // This controls the timing of the Controller threads.
  private FastLoopTimer mFastLoopTimer;
  // This is the operator's controller.
  private OperatorInput mOperatorInput;
  // This is the class which directs autonomous flow during
  // the autonomous period.
  private StateMachine mStateMachine;

  /*
   * These are just the paths on the roborio where we can find the properties files and autonomous
   * files. The reason we read some values in from a file is because it is quicker to connect over
   * FTP to the roborio's file system and change data inside a file, than it is to recompile code
   * with the new property value.
   */
  private static final String kPropertiesFilePath = "/home/lvuser/properties/properties.txt";
  private static final String kAutonomousFilePath = "/home/lvuser/autonomous/";

  /*
   * The "Last button state" booleans keep track of whether or not the button in question is being
   * held down. If the last button state is true, that means that the button on the controller has
   * not been let go since the last loop iteration.
   */
  boolean mLastAButtonState = false;
  boolean mLastBButtonState = false;
  boolean mLastYButtonState = false;

  /*
   * These are variables to allow for quick addition of different autonomous modes.
   */
  private int mNumAutoDefenses = 4;
  private int mNumAutoPos = 10; // 6 is for cross, 7 is do nothing,
                                // 8 for reach, 9 for spybot, 10 for CDF
  private boolean mAutoStealBall = false;
  private int mAutoDefense = 1;
  private int mAutoPos = 2;
  String mAutoMode;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  public void robotInit() {
    /*
     * Here, we just instantiate or get the instance of the various objects
     */
    mPropertyReader = new PropertyReader();
    mPropertyReader.parseFile(kPropertiesFilePath);
    loadAllProperties();
    mOperatorInput = OperatorInput.getInstance();
    mFastLoopTimer = FastLoopTimer.getInstance();
    // starts the fast loop timer executing input & output filters and controllers
    mFastLoopTimer.start();
    // this is the default autonomous mode that will execute if the drive/operator
    // do not choose to use another one.
    chooseAutoMode(mAutoDefense, 9, false);

    /*
     * This is where we tried to instantiate the USB camera's that the driver used to have better
     * visuals of the field. Unfortunately, the Camera Server class was not written very well.
     */
    CameraServer server = CameraServer.getInstance();
    server.setQuality(50);
    //// server.startAutomaticCapture("cam2");
    server.startAutomaticCapture("cam1");

    // cameraServer = new RobotVisionDualUSB("cam1", "cam2");
    // cameraServer.initializeCameras();

  }

  public void disabledPeriodic() {
    /*
     * Just insure that nothing is trying to control the robot going into the start of autonomous
     */
    Drive.getInstance().setOpenLoop();
    Shooter.getInstance().setOpenLoop();
    /**
     * This function is constantly called (remember, these functions are called in a loop) and
     * listens for a button press from the driver to change the autonomous mode before a match
     */
    listenForAutoChanges();
    // cameraServer.endCameras();
  }

  public void autonomousInit() {
    // mPropertyReader.parseAutonomousFile(kAutonomousFilePath + "RoughTerrain.txt");
    /**
     * Instantiates the StateMachine, which ensure that the robot follows a very defined transition
     * between each state in autonomous. The AutonomousParser is a class that will read in an
     * autonomous file from the Roborio's file system, and parses it for the series of states that
     * the file contains.
     */
    mStateMachine = new StateMachine(new AutonomousParser().parseStates(mAutoPos, mAutoStealBall));
    logToDashboard();
  }

  /**
   * This function is called periodically during autonomous
   */
  public void autonomousPeriodic() {
    /**
     * Runs the state machine
     */
    mStateMachine.run();
    logToDashboard();
  }

  public void teleopInit() {
    /**
     * Ensures that nothing is attempting to control the drive base at the very start of the
     * teleoperated period.
     */
    Drive.getInstance().setOpenLoop();
    logToDashboard();
    // cameraServer.initializeCameras();
  }

  /**
   * This function is called periodically during operator control
   */
  public void teleopPeriodic() {
    /*
     * This is the meat and potatoes of where decisions are made depending on what buttons are
     * pressed by the Operator and Driver.
     */
    mOperatorInput.processInputs();
    logToDashboard();
  }

  /**
   * This function is called periodically during test mode
   */
  public void testPeriodic() {

  }

  /**
   * Listens for a button click from the Driver/Operator, indicating a change in autonomous modes.
   */
  private void listenForAutoChanges() {
    if (mOperatorInput.mDriverController.getAButton() && !mLastAButtonState) {
      mAutoPos++;
      if (mAutoPos > mNumAutoPos) {
        mAutoPos = 2;
      }
      chooseAutoMode(mAutoDefense, mAutoPos, mAutoStealBall);
    }

    mLastAButtonState = mOperatorInput.mDriverController.getAButton();

    if (mOperatorInput.mDriverController.getBButton() && !mLastBButtonState) {
      mAutoDefense++;
      if (mAutoDefense > mNumAutoDefenses) {
        mAutoDefense = 1;
      }
      chooseAutoMode(mAutoDefense, mAutoPos, mAutoStealBall);
    }

    mLastBButtonState = mOperatorInput.mDriverController.getBButton();

    if (mOperatorInput.mDriverController.getYButton() && !mLastYButtonState) {
      mAutoStealBall = !mAutoStealBall;
      chooseAutoMode(mAutoDefense, mAutoPos, mAutoStealBall);
    }

    mLastYButtonState = mOperatorInput.mDriverController.getYButton();
  }

  /**
   * Points the Property Reader to which autonomous file to parse, based on the inputs of:
   * 
   * @param autoDefense Which defense we want to travel over
   * @param autoPos Which position we start at (1, 2, 3, 4, 5)
   * @param mAutoStealBall Whether we want to steal a ball. Steal ball was never used during a
   *        match.
   */
  private void chooseAutoMode(int autoDefense, int autoPos, boolean mAutoStealBall) {
    switch (autoDefense) {
      case 1:
        mPropertyReader.parseAutonomousFile(kAutonomousFilePath + "RoughTerrain.txt");
        mAutoMode = "Rough Terrain";
        break;
      case 2:
        mPropertyReader.parseAutonomousFile(kAutonomousFilePath + "RockWall.txt");
        mAutoMode = "Rock Wall";
        break;
      case 3:
        mPropertyReader.parseAutonomousFile(kAutonomousFilePath + "Rampparts.txt");
        mAutoMode = "Rampparts";
        break;
      case 4:
        mPropertyReader.parseAutonomousFile(kAutonomousFilePath + "Moat.txt");
        mAutoMode = "Moat";
        break;
      default:
        mPropertyReader.parseAutonomousFile(kAutonomousFilePath + "RoughTerrain.txt");
        mAutoMode = "Default Auto Mode";
        break;
    }

    switch (autoPos) {
      case 6:
        mAutoMode = "Cross";
        break;
      case 7:
        mAutoMode = "Do Nothing";
        break;
      case 8:
        mAutoMode = "Reach";
        break;
      case 9:
        mPropertyReader.parseAutonomousFile(kAutonomousFilePath + "Spybot.txt");
        mAutoMode = "Spy Bot";
        break;
      case 10:
        mPropertyReader.parseAutonomousFile(kAutonomousFilePath + "CDF.txt");
        mAutoMode = "Chevy --> Levy";
        break;
      default:
        mAutoMode += " Pos: " + Integer.toString(autoPos);

        if (mAutoStealBall) {
          mAutoMode += " StealBall: TRUE";
        } else {
          mAutoMode += " StealBall: FALSE";
        }

        break;
    }

    SmartDashboard.putString("Autonomous Mode:", mAutoMode);
  }

  /**
   * Calls all of the Subsystems logToDashboard functions, which each publishes different values to
   * the SmartDashboard for debugging purposes.
   */
  public void logToDashboard() {
    Navigation.getInstance().logToDashboard();
    Shooter.getInstance().logToDashboard();
    Drive.getInstance().logToDashBoard();
    Intake.getInstance().logToDashboard();
    Hanger.getInstance().logToDashboard();
  }

  /**
   * This tells each subsystem to load subsystem specific properties.
   */
  public void loadAllProperties() {
    DriveTurnController.getInstance().loadProperties();
    ShooterSpeedController.getInstance().loadProperties();
    DriveDistanceController.getInstance().loadProperties();
  }
}

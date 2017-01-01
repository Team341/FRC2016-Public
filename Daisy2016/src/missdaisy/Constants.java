package missdaisy;

/**
 * A list of all inputs, as well as other constants like RPM set points and PID gains. These are
 * just the port number that these components are plugged into on the roborio.
 * 
 * @author Joshua Sizer
 *
 */
public final class Constants {
  /*
   * All PWM inputs, mostly just for speed controllers.
   */
  public static class PWMs {
    public static final int DRIVE_LEFT_MOTOR = 0;
    public static final int DRIVE_RIGHT_MOTOR = 1;
    public static final int CONVEYOR_MOTOR = 2;
    public static final int INTAKE_MOTOR = 3;
    public static final int HANER_MOTOR = 4;
  }

  /*
   * Servo input numbers
   */
  public static final class SERVOs {
    public static final int BARRIER_LEFT = 6;
    public static final int BARRIER_RIGHT = 7;
  }

  /*
   * Digital inputs, like encoders, banner sensors, or anything that has two different outputs
   */
  public static final class DigitalInputs {
    public static final int DRIVE_LEFT_ENCODER_1 = 0;
    public static final int DRIVE_LEFT_ENCODER_2 = 1;
    public static final int DRIVE_RIGHT_ENCODER_1 = 2;
    public static final int DRIVE_RIGHT_ENCODER_2 = 3;
    public static final int SHOOTER_BANNER = 7;

    // 5 - for comp bot, 6 for twin
    public static final int CONVEYOR_BANNER = 5;

    // 6 - for comp bot, 7 for twin
    public static final int DEFENSE_BANNER = 6;

    // 8 - for stuck on defense sensor
    public static final int DRIVE_RAIL_LIMIT = 9;
  }

  /*
   * Solenoids are the components that control airflow in a pnuematic system. Basically just a
   * piston.
   */
  public static final class Solenoids {
    public static final int BALL_LIGHT = 1;
    public static final int STATUS = 2;
    public static final int VISION = 3;
    // 4 for competition bot, 0 for twin
    public static final int INTAKE_DOUBLE_1 = 4;
    public static final int INTAKE_DOUBLE_2 = 5;
    public static final int HOOD = 6;
    public static final int HANGER = 7;

    public static final int POPPER_DOUBLE_1 = 0;
    public static final int POPPER_DOUBLE_2 = 3;
  }

  /*
   * Can is a special type of communication interface.
   */
  public static final class CAN {
    public static final int SHOOTER_TALONSRX_ID = 0;
  }

  /*
   * These are the ports that the controllers are plugged into the driver station, as well as the
   * dead band for the controllers.
   */
  public static final class XBOXController {
    public static final int DRIVER_PORT = 0;
    public static final int OPERATOR_PORT = 1;
    public static final double DEAD_BAND = 0.3;
  }

  /** Holds specific values for properties, especially for PID */
  public static final class Properties {
    /**
     * The RPM that best shoots the ball into the tower from the outer works
     */
    public static final double SHOOTER_RPM_OUTERWORKS = 5400;

    /** The RPM that best shoots the ball into the tower from the batter */
    public static final double SHOOTER_RPM_BATTER = 3000;

    /**
     * The acceptable deviation from the setpoint on the shooter's RPM PID
     */
    public static final double PID_SHOOTER_RPM_TOLERANCE = 50;

    /**
     * The acceptable deviation from the setpoint on the Drive Distance PID
     */
    public static final double PID_DRIVE_DISTANCE_TOLERANCE = .5;

    /**
     * The acceptable deviation from the setpoint on the drive's turn PID
     */
    public static final double PID_DRIVE_ANGLE_TOLERANCE = 1.0; // Normally
                                                                // use 0.5

    /**
     * The maximum output to give to the DriveTurn PID, to aid in the reduction of overshoot
     */
    public static final double PID_DRIVE_TURN_MAX_OUTPUT = 0.5;

    /**
     * The minimum output to give to the DriveTurn PID, to aid in the reduction of steady state
     * error
     */
    public static final double PID_DRIVE_TURN_MIN_OUTPUT = 0.14;// 0.15;

    /**
     * The gain for the drive's alpha filter, which is used to limit acceleration
     */ // 0.07 for competition, 0.1 for twin
    public static final double DRIVE_ALPHA_FILTER_GAIN_INTAKE_UP = 0.05;
    public static final double DRIVE_ALPHA_FILTER_GAIN_INTAKE_DOWN = 0.05;

    public static final double DRIVE_BREAK_SPEED = 0.15;

    /**
     * The distance the robot moves per shaft rotation. Used to calculate the speed of the robot in
     * the <code>Drive</code> class.
     */
    public static final double DRIVE_DISTANCE_PER_PULSE = (Math.PI * 6) / 255;
    /**
     * The length, in milliseconds, of each loop for the fast loop timer (which executes input and
     * output filters and the subsystem's current controllers
     */
    public static final long FAST_LOOP_TIMER_PERIOD = 10L;
  }
}

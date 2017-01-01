package missdaisy.autonomous;

import missdaisy.loops.controllers.ShooterSpeedController;
import missdaisy.subsystems.Shooter;

/**
 * This states revs the shooter to a specified RPM by setting the shooter controller to be the
 * shooterSpeedController.
 * 
 *
 */
public class StartShooter extends State {
  private Shooter mShooter;
  private ShooterSpeedController mShooterController;
  private double RPM;

  public StartShooter(double RPM) {
    super("StartShooter");
    mShooter = Shooter.getInstance();
    mShooterController = ShooterSpeedController.getInstance();
    this.RPM = RPM;
  }

  public void enter() {
    mShooter.setOpenLoop();
    mShooterController.setGoal(RPM);
    mShooter.setCurrentController(mShooterController);
  }

  @Override
  public void running() {
    // the controller gets executed every 5 ms in its own thread
  }

  @Override
  public boolean isDone() {
    return true;
  }

}

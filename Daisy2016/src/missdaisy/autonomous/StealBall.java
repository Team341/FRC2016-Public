package missdaisy.autonomous;

import missdaisy.subsystems.Intake;

/**
 * Meant to be used at the start of autonomous 
 * 
 * @author Adam N.
 *
 */
public class StealBall extends State {
  private Intake mIntake;
  private long startTime;

  public StealBall() {
    super("Intake");
    mIntake = Intake.getInstance();
  }

  public void enter() {
    mIntake.retract();
    startTime = System.currentTimeMillis();
  }

  @Override
  public void running() {
    if (mIntake.seesBall()) {
      mIntake.setIntakeSpeed(0.0);
      mIntake.setConveyorSpeed(0.0);
    } else {
      mIntake.setIntakeSpeed(1.0);
      mIntake.setConveyorSpeed(0.6);
    }

  }

  public void exit() {
    // retract is really deploy because someone miss-named them
    mIntake.setIntakeSpeed(0.0);
    mIntake.setConveyorSpeed(0.0);
    mIntake.deploy();
  }

  @Override
  public boolean isDone() {
    return mIntake.seesBall() || (System.currentTimeMillis() - startTime) > 2000;
  }
}

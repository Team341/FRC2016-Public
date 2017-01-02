package missdaisy.autonomous;

import missdaisy.subsystems.Intake;

/**
 * Simply deploys or retracts the intake, depending on the input argument.
 *
 */
public class DeployIntake extends State {
  private Intake mIntake;
  private boolean mDeployed = false;

  public DeployIntake(double state) {
    super("Intake");
    mIntake = Intake.getInstance();
    if (state > 0.0)
      mDeployed = true;
  }

  /**
   * Deploys the intake if the parameter passed in is greater than 0.0. Retracts the intake
   * if the parameter passed in is equal to 0.0
   */
  public void enter() {
    if (mDeployed) {
      mIntake.retract();
    } else {
      mIntake.deploy();
    }
  }

  @Override
  public void running() {}

  public void exit() {}

  /**
   * This autonomous state only does one thing. It does not need to loop more than once.
   */
  @Override
  public boolean isDone() {
    return true;
  }

}

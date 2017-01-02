package missdaisy.autonomous;

/**
 * State to wait for a specified number of milliseconds.
 * 
 * @author Jared341
 */
public class WaitForTime extends TimeoutState {
  long mTimeout;

  public WaitForTime(int aMilliseconds) {
    super("WaitForTime", aMilliseconds);
  }

  public void enter() {
    super.enter();
  }

  public void running() {}
}

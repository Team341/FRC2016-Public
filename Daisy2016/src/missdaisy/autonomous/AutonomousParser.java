package missdaisy.autonomous;

import missdaisy.fileio.*;
import java.lang.Class;
import java.lang.reflect.Constructor;
import java.lang.reflect.InvocationTargetException;
import java.lang.reflect.Type;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Parses autonomous modes from an auto mode text file.
 * 
 * Uses reflection to create instances of each auto state based on its name If it cannot be
 * instantiated for various reasons, WaitForTime(0) is instantiated
 * 
 * @author Joshua Sizer
 */
public class AutonomousParser {
  private final PropertySet mProperties;
  private Class<?> currentState;
  private Constructor<?> mStateConstructor;
  private Type[] mParamTypes;
  int mNumberOfParameters;

  public AutonomousParser() {
    mProperties = PropertySet.getInstance();
  }

  public State[] parseStates(int pos, boolean stealBall) {
    State[] fullAuto;
    System.out.println("Searching for selected auto,  ");
    switch (pos) {
      case 7: // Do nothing
        System.out.println("Auto Selected: Do Nothing,  ");
        fullAuto = new State[] {new WaitForTime(0)};
        break;
      case 8: // Reach
        System.out.println("Auto Selected: Reach,  ");
        fullAuto = new State[] {new DriveDistance(75, 0.5)};
        break;
      case 9: // Spy bot
        System.out.println("Auto Selected: Spybot,  ");
        fullAuto = parseStates();
        break;
      case 10: // Chevy
        fullAuto = parseStates();
        break;
      default:
        System.out.println("Auto Selected: Default,  ");
        State[] defense = parseStates();
        // every autonomous will be:
        // drive forward
        // cross defense (with specific parameters parsed from text file)
        // turn to face target
        // autoaim and shoot
        State angle = null;

        int numStealBallSteps = 0;
        if (stealBall) {
          numStealBallSteps = 1;
        }


        fullAuto = new State[defense.length + 4 + numStealBallSteps];
        fullAuto[0] = new ResetHeading(0.0);
        int count = 1;

        if (stealBall) {
          fullAuto[count] = new StealBall();
          count++;
        }

        for (int i = 0; i < defense.length; i++) {
          fullAuto[count] = defense[i];
          count++;
        }

        fullAuto[count] = new WaitForTime(750); // 250
        count++;

        double angleValue = 0.0;
        switch (pos) {
          case 2:
            angleValue = 15.0;
            angle = new TurnToAngle(15);
            break;
          case 3:
            angleValue = 5.0;
            angle = new TurnToAngle(5);
            break;
          case 4:
            angleValue = 5.0;
            angle = new TurnToAngle(5);
            break;
          case 5:
            angleValue = 345.0;
            angle = new TurnToAngle(345);
            break;
          default:
            angleValue = 0.0;
            angle = new WaitForTime(0);
        }

        fullAuto[count] = angle;
        count++;

        fullAuto[count] = new AutoAimAndShoot(angleValue);
        break;
    }

    for (State curState : fullAuto) {
      if (curState == null)
        curState = new WaitForTime(0);
    }

    return fullAuto;
  }

  public State[] parseStates() {
    State[] lStates;
    int lNumStates = mProperties.getIntValue("AutonomousNumStates", -1);
    System.out.println("Found " + Integer.toString(lNumStates) + " auto states");

    if (lNumStates < 1) {
      lStates = new State[1];
      lStates[0] = new WaitForTime(0);
      System.out.println("Insufficent number of states,  ");
    } else {
      lStates = new State[lNumStates];
      for (int i = 0; i < lNumStates; i++) {
        System.out.println("Parsing State " + Integer.toString(i + 1) + ",  ");

        String lStateName =
            mProperties.getStringValue("AutonomousState" + Integer.toString(i + 1), "");
        try {
          currentState = Class.forName(this.getClass().getPackage().getName() + "." + lStateName);
          // currentState = Class.forName( "missdaisy.autonomous." + lStateName);
          // there should only be one constructor
          mStateConstructor = currentState.getConstructors()[0];
          mNumberOfParameters = mStateConstructor.getParameterCount();

          Object mParamValues[] = new Object[mNumberOfParameters];

          mParamTypes = mStateConstructor.getParameterTypes();

          // fills parameter array with values
          System.out.println("Parsing parameters for state,  ");
          for (int k = 0; k < mStateConstructor.getParameters().length; k++) {
            // returns 0.0 or 0 if property cannot be found
            if (mParamTypes[k] == double.class) {
              mParamValues[k] = mProperties.getDoubleValue(
                  "AutonomousState" + Integer.toString(i + 1) + "Param" + Integer.toString(k + 1),
                  0.0);
            } else if (mParamTypes[k] == int.class) {
              // truncates value from autonomous file to be an
              // integer (if a number is entered as a double,
              // but is meant to be an integer)
              mParamValues[k] = (int) mProperties.getDoubleValue(
                  "AutonomousState" + Integer.toString(i + 1) + "Param" + Integer.toString(k + 1),
                  0.0);
            }
          }

          try {
            lStates[i] = (State) mStateConstructor.newInstance(mParamValues);
            System.out.println("Instantiated " + lStates[i].toString() + " with parameter(s): "
                + formatParam(mParamValues));
          } catch (InstantiationException | IllegalAccessException | InvocationTargetException
              | IllegalArgumentException e) {
            e.printStackTrace();
            System.out.println("Error instantiating " + "AutonomousState" + Integer.toString(i + 1)
                + ": " + lStateName + ". WaitForTime instantiated instead.");
          }

        } catch (ClassNotFoundException | SecurityException e) {
          System.out.println("Could not find specified state: " + lStateName
              + ". WaitForTime (0) instantiated instead.");
        }

        if (lStates[i] == null) {
          lStates[i] = new WaitForTime(0);
        }
      }
    }
    return lStates;
  }

  private String formatParam(Object[] paramValues) {
    String lFormattedString = "";
    for (int k = 0; k < paramValues.length; k++) {
      lFormattedString += paramValues[k].toString();
      if (k == paramValues.length - 1)
        break;

      lFormattedString += ", ";
    }
    return lFormattedString;
  }
}

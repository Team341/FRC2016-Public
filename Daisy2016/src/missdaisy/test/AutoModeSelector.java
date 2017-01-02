package missdaisy.test;

import java.util.Scanner;
import missdaisy.autonomous.AutonomousParser;
import missdaisy.autonomous.StateMachine;
import missdaisy.fileio.PropertyReader;

/**
 * This class is designed to help test the autonomous mode selector.
 * 
 * @author joshs
 *
 */
public class AutoModeSelector {
  private static final String kAutonomousFilePath =
      System.getProperty("user.home") + "\\Desktop\\Autonomous_Modes\\";
  private static int mNumAutoDefenses = 4;
  private static int mNumAutoPos = 9;
  private static int mAutoDefense = 1;
  private static int mAutoPos = 2;
  private static String mAutoMode;
  private static String selector = "";
  private static Scanner in = new Scanner(System.in);
  private static PropertyReader mPropertyReader = new PropertyReader();

  public static void main(String[] args) {
    System.out.println("Autonomous file path: " + kAutonomousFilePath);
    chooseAutoMode(mAutoDefense, mAutoPos);
    selector = in.nextLine();
    while (true) {
      if (selector.equals("p")) {
        mAutoPos++;
        if (mAutoPos > mNumAutoPos) {
          mAutoPos = 2;
        }
        chooseAutoMode(mAutoDefense, mAutoPos);
      }

      else if (selector.equals("d")) {
        mAutoDefense++;
        if (mAutoDefense > mNumAutoDefenses) {
          mAutoDefense = 1;
        }
        chooseAutoMode(mAutoDefense, mAutoPos);
      } else if (selector.equals("r")) {
        StateMachine machine =
            new StateMachine(new AutonomousParser().parseStates(mAutoPos, false));
        while (true) {
          machine.run();
          try {
            Thread.sleep(19);
          } catch (InterruptedException e) {
            e.printStackTrace();
          }
        }
      }
      selector = in.nextLine();
    }
  }

  public static void chooseAutoMode(int autoDefense, int autoPos) {
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
      default:
        mAutoMode += " Pos: " + Integer.toString(autoPos);
        break;
    }
    System.out.println(mAutoMode);
  }
}

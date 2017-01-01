package missdaisy.utilities;

import missdaisy.utilities.CustomCameraServer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.vision.USBCamera;

public class RobotVisionDualUSB {
  private CustomCameraServer cameraServer;
  private USBCamera firstCam = null;
  private USBCamera secondCam = null;
  private String camName1 = "cam1";
  private String camName2 = "cam2";

  String currCam = "cam1";

  private static RobotVisionDualUSB dualUSB;
  // boolean isCam1 = true;

  public static RobotVisionDualUSB getInstance() {
    if (dualUSB == null) {
      dualUSB = new RobotVisionDualUSB("cam1", "cam2");
    }
    return dualUSB;
  }

  public RobotVisionDualUSB(String cam1, String cam2) {
    cameraServer = CustomCameraServer.getInstance();
    this.camName1 = cam1;
    this.camName2 = cam2;
    cameraServer.startAutomaticCapture(cam1);
  }

  public void switchCameras() {
    if (currCam.equals(camName1)) {
      cameraServer.startAutomaticCapture(camName2);
      currCam = camName2;
      SmartDashboard.putString("CameraUsed", "Forward Cam");
    } else {
      cameraServer.startAutomaticCapture(camName1);
      currCam = camName1;
      SmartDashboard.putString("CameraUsed", "Backup Cam");
    }
    /*
     * if(isCam1) { cameraServer.startAutomaticCapture(secondCam); currCam = camName2; isCam1 =
     * false; } else { cameraServer.startAutomaticCapture(firstCam); currCam = camName1; isCam1 =
     * true; }
     */
  }

  public void endCameras() {
    if (firstCam != null) {
      firstCam.closeCamera();
      firstCam = null;
    }
    if (secondCam != null) {
      secondCam.closeCamera();
      secondCam = null;
    }
  }

  public void initializeCameras() {
    endCameras();
    try {
      firstCam = new USBCamera(camName1);
    } catch (Exception e) {
      firstCam = null;
    }

    try {
      secondCam = new USBCamera(camName2);
    } catch (Exception e) {
      secondCam = null;
    }

    if (cameraServer == null) {
      cameraServer = CustomCameraServer.getInstance();
      cameraServer.setQuality(50);
    }
    currCam = camName1;
    cameraServer.startAutomaticCapture(firstCam);
  }
}

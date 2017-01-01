package missdaisy.utilities;

import com.ni.vision.NIVision;
import com.ni.vision.NIVision.Image;
import edu.wpi.first.wpilibj.CameraServer;

public class DaisyUSBCamera {
  int currSession;
  int sessionfront;
  int sessionback;
  Image frame;

  public DaisyUSBCamera() {
    frame = NIVision.imaqCreateImage(NIVision.ImageType.IMAGE_RGB, 0);
    sessionfront = NIVision.IMAQdxOpenCamera("cam2",
        NIVision.IMAQdxCameraControlMode.CameraControlModeController);
    sessionback = NIVision.IMAQdxOpenCamera("cam3",
        NIVision.IMAQdxCameraControlMode.CameraControlModeController);
    currSession = sessionback;
    NIVision.IMAQdxConfigureGrab(currSession);
    CameraServer.getInstance().setQuality(25);
  }

  public void setFrontCamera() {
    changeCam(sessionfront);
  }

  public void setBackCamera() {
    changeCam(sessionback);
  }

  public void run() {
    NIVision.IMAQdxGrab(currSession, frame, 1);
    CameraServer.getInstance().setImage(frame);
  }

  /**
   * Change the camera to get imgs from to a different one
   * 
   * @param newId for camera
   */
  public void changeCam(int camera) {
    NIVision.IMAQdxStopAcquisition(currSession);
    currSession = camera;
    NIVision.IMAQdxConfigureGrab(currSession);
  }
}

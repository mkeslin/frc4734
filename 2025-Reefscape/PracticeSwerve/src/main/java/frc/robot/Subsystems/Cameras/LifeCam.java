package frc.robot.Subsystems.Cameras;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;

public class LifeCam {
    public UsbCamera m_Camera;
    public boolean live = false;

    public LifeCam(int port) {
        m_Camera = CameraServer.startAutomaticCapture(port);
        m_Camera.setFPS(1);
        m_Camera.setResolution(320, 240);
        live = true;
    }

    public boolean isLive() {
        return live;
    }

    public void startStream() {
        m_Camera.setFPS(15);
    }

    public void stopStream() {
        m_Camera.setFPS(1);
    }
}

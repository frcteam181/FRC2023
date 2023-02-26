package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.common.hardware.VisionLEDMode;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.*;

public class Vision extends SubsystemBase{

    PhotonCamera m_limelight;
    
    NetworkTableEntry m_tx, m_ty, m_ta;

    public Vision() {

        m_limelight = new PhotonCamera("eagle");

        m_limelight.setPipelineIndex(k_REFLECTIVE_PIPELINE);

        setLEDoff();

    }

    public void setLEDon() {
        m_limelight.setLED(VisionLEDMode.kOn);
    }

    public void setLEDoff() {
        m_limelight.setLED(VisionLEDMode.kOff);
    }

    public void toggleLED() {
        if (m_limelight.getLEDMode() != VisionLEDMode.kOff) {
            m_limelight.setLED(VisionLEDMode.kOff);
        } else {
            m_limelight.setLED(VisionLEDMode.kOn);
        }
    }
    
}

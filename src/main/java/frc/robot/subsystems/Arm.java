package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class Arm extends CommandBase {

    private boolean m_isTuning;

    public Arm(boolean isTuning) {

        m_isTuning = isTuning;

    }
    
}

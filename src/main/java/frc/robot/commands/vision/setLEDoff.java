package frc.robot.commands.vision;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Vision;

public class setLEDoff extends CommandBase {

    private final Vision m_vision;

    public setLEDoff(Vision vision) {
        
        m_vision = vision;

        addRequirements(m_vision);

    }

    @Override
    public void execute() {
        m_vision.setLEDoff();
    }
    
}

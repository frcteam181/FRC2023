package frc.robot.commands.vision_commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Vision;

public class toggleLED extends CommandBase {

    private final Vision m_vision;

    public toggleLED(Vision vision) {
        
        m_vision = vision;

        addRequirements(m_vision);

    }

    @Override
    public void execute() {
        m_vision.toggleLED();;
    }
    
}

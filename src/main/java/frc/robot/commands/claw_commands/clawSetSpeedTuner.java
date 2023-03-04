package frc.robot.commands.claw_commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Claw;

public class clawSetSpeedTuner extends CommandBase {

    private final Claw m_claw;

    public clawSetSpeedTuner (Claw claw) {
        
        m_claw = claw;

        addRequirements(m_claw);
    }

    @Override
    public void execute() {
        m_claw.setClawSpeedToTuning();
    }
    
}
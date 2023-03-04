package frc.robot.commands.claw_commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Claw;

public class closeClaw extends CommandBase {

    private Claw m_claw;

    public closeClaw(Claw claw) {

        m_claw = claw;

        addRequirements(m_claw);
    }

    @Override
    public void initialize() {
        m_claw.close();
    }
    
}

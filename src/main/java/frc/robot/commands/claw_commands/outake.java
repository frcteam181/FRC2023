package frc.robot.commands.claw_commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Claw;

public class outake extends CommandBase {

    private Claw m_claw;

    public outake(Claw claw) {

        m_claw = claw;

        addRequirements(m_claw);

    }

    @Override
    public void initialize() {
        //m_claw.setSpeed(-1);
        m_claw.outake();
    }
    
}

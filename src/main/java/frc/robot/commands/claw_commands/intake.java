package frc.robot.commands.claw_commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Claw;

public class intake extends CommandBase {

    private Claw m_claw;

    public intake(Claw claw) {

        m_claw = claw;

        addRequirements(m_claw);

    }

    @Override
    public void initialize() {
        //m_claw.setSpeed(0.1);
        m_claw.intakeOn();
    }

    @Override
    public void end(boolean interrupted) {
        m_claw.stopIntake();
    }
    
}

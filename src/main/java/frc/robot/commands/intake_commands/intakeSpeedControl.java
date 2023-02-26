package frc.robot.commands.intake_commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;

public class intakeSpeedControl extends CommandBase {

    private Intake m_intake;
    private double m_speed;
    
    public intakeSpeedControl(Intake intake, double speed) {

        m_intake = intake;
        m_speed = speed;

        addRequirements(m_intake);

    }

    @Override
    public void execute() {
        m_intake.speedControl(m_speed);
    }

    @Override
    public void end(boolean interrupted) {
        m_intake.stop();
    }

}

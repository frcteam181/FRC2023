package frc.robot.commands.autos;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveBase;

public class balance extends CommandBase {

    private DriveBase m_driveBase;

    private PIDController m_forwardController;

    private double m_currentAngle, m_output;

    public balance(DriveBase driveBase) {

        m_driveBase = driveBase;

        addRequirements(m_driveBase);

        m_forwardController = new PIDController(0, 0, 0);
        m_forwardController.setTolerance(2.0);

    }

    @Override
    public void initialize() {
        m_forwardController.setSetpoint(0);
    }

    @Override
    public void execute() {
        m_currentAngle = m_driveBase.getPitch();
        m_output = m_forwardController.calculate(m_currentAngle, 0);
    }

    @Override
    public void end(boolean interrupted) {
        //m_driveBase.stopMotors();
    }

    @Override
    public boolean isFinished() {
        return m_forwardController.atSetpoint();
    }
    
}

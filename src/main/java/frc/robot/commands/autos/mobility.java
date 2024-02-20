package frc.robot.commands.autos;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveBase;

public class mobility extends CommandBase {

    private DriveBase m_driveBase;
    private boolean m_isFinished;
    private double m_driveGoalMeters;

    public mobility (DriveBase driveBase, double driveGoalMeters) {

        m_driveBase = driveBase;
        m_driveGoalMeters = driveGoalMeters;
        m_isFinished = false;

        addRequirements(m_driveBase);

    }

    @Override
    public void initialize() {

        m_driveBase.resetEncoders();

        if(m_driveGoalMeters >= 0) {
            m_driveBase.driveSpeed(0.2);
        } else {
            m_driveBase.driveSpeed(-0.2);
        }
        
    }

    @Override
    public void execute() {
        if(Math.abs(m_driveBase.getAverageWheelPosition()) >= Math.abs(m_driveGoalMeters)) {
            m_isFinished = true;
        }
    }

    @Override
    public void end(boolean interrupted) {
        m_driveBase.driveSpeed(0);
    }

    @Override
    public boolean isFinished() {
        return m_isFinished;
    }
    
}

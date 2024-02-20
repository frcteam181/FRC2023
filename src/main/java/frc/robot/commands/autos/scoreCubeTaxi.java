package frc.robot.commands.autos;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveBase;

public class scoreCubeTaxi extends CommandBase {

    private DriveBase m_driveBase;
    private boolean m_isFinished;

    public scoreCubeTaxi(DriveBase driveBase) {

        m_driveBase = driveBase;
        m_isFinished = false;

        addRequirements(m_driveBase);

    }

    @Override
    public void initialize() {
        m_driveBase.resetEncoders();
        m_driveBase.resetHeading();
        m_driveBase.enable();
        m_driveBase.setDriveGoal(20);
    }

    @Override
    public boolean isFinished() {
        return m_driveBase.isAtSetpoint();
    }
    
}

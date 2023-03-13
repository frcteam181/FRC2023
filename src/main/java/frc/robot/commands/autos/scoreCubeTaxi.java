package frc.robot.commands.autos;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveBase;

public class scoreCubeTaxi extends CommandBase {

    private DriveBase m_driveBase;

    public scoreCubeTaxi(DriveBase driveBase) {

        m_driveBase = driveBase;

        addRequirements(m_driveBase);

    }

    @Override
    public void initialize() {
        System.out.println("Initialized");
        m_driveBase.resetEncoders();
        m_driveBase.resetHeading();
        m_driveBase.enable();
        m_driveBase.setDriveGoal(20);
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("Ended");
        m_driveBase.disable();
    }
    
}

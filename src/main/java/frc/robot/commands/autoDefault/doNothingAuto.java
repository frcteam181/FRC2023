package frc.robot.commands.autoDefault;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveBase;

public class doNothingAuto extends CommandBase {

    private DriveBase m_driveBase;

    public doNothingAuto(DriveBase driveBase) {

        m_driveBase = driveBase;

        addRequirements(m_driveBase);

    }

    @Override
    public void initialize() {
        m_driveBase.teleopDrive(0, 0);
    }
    
}

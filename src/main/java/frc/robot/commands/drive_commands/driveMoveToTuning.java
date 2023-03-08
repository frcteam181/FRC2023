package frc.robot.commands.drive_commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveBase;

public class driveMoveToTuning extends CommandBase {

    private DriveBase m_driveBase;

    public driveMoveToTuning(DriveBase driveBase) {

        m_driveBase = driveBase;

        addRequirements(m_driveBase);

    }

    @Override
    public void initialize() {
        
    }
    
}

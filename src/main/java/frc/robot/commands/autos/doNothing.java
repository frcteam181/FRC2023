package frc.robot.commands.autos;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveBase;

public class doNothing extends CommandBase {

    private DriveBase m_driveBase;

    public doNothing(DriveBase driveBase) {

        m_driveBase = driveBase;

        addRequirements(m_driveBase);

    }

    @Override
    public void initialize() {
        m_driveBase.teleopDrive(0, 0);
        System.out.println("Do nothing");
    }
    
}

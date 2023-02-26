package frc.robot.commands.default_commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveBase;

public class DriveBaseDefaultCommand extends CommandBase{

    private final DriveBase m_driveBase;
    private final Joystick m_driverController;

    public DriveBaseDefaultCommand(DriveBase driveBase, Joystick driverController) {
        
        m_driveBase = driveBase;
        m_driverController = driverController;

        addRequirements(m_driveBase);
    }

    @Override
    public void execute() {

        double m_forwardValue, m_turnValue;
        m_forwardValue = (-1) * m_driverController.getY();
        m_turnValue = (-1) * m_driverController.getZ();
        m_driveBase.teleopDrive(m_forwardValue, m_turnValue);

    }
    
}
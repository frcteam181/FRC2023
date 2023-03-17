package frc.robot.commands.default_commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.DriveBase;

public class DriveBaseDefaultCommand extends CommandBase{

    private final DriveBase m_driveBase;
    private final Claw m_claw;

    private final Joystick m_driverController;

    public DriveBaseDefaultCommand(DriveBase driveBase, Claw claw, Joystick driverController) {
        
        m_driveBase = driveBase;
        m_claw = claw;

        m_driverController = driverController;

        addRequirements(m_driveBase, m_claw);
    }

    @Override
    public void execute() {

        double m_forwardValue, m_turnValue;
        m_forwardValue = (-1) * m_driverController.getY();
        m_turnValue = (-1) * m_driverController.getZ();
        m_driveBase.teleopDrive(m_forwardValue, m_turnValue);

    }
    
}
package frc.robot.commands.default_commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.DriveBase;

public class DriveBaseXboxDefaultCommand extends CommandBase {

    private final DriveBase m_driveBase;
    private final Claw m_claw;

    private final XboxController m_driverController;

    public DriveBaseXboxDefaultCommand(DriveBase driveBase, Claw claw, XboxController driverController) {
        
        m_driveBase = driveBase;
        m_claw = claw;

        m_driverController = driverController;

        addRequirements(m_driveBase, m_claw);
    }

    @Override
    public void execute() {

        var leftY = m_driverController.getLeftY();
        var rightY = m_driverController.getRightY();

        m_driveBase.teleopTankDrive(leftY, rightY);

    }
    
}
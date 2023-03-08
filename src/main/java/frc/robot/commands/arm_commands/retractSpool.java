package frc.robot.commands.arm_commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;

public class retractSpool extends CommandBase {

    private Arm m_arm;

    public retractSpool(Arm arm) {

        m_arm = arm;

        addRequirements(m_arm);

    }

    @Override
    public void initialize() {
        m_arm.retractSpool();
    }

    @Override
    public void end(boolean interrupted) {
        m_arm.stopSpool();
    }
    
}

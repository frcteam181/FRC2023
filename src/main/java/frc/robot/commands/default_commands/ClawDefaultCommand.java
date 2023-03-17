package frc.robot.commands.default_commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Spool;

public class ClawDefaultCommand extends CommandBase{

    private Arm m_arm;
    private Spool m_spool;
    private Claw m_claw;

    private XboxController m_operatorController;

    public ClawDefaultCommand(Arm arm, Spool spool, Claw claw, XboxController operatorController) {

        m_arm = arm;
        m_spool = spool;
        m_claw = claw;

        m_operatorController = operatorController;

        addRequirements(m_arm, m_spool, m_claw);
    }
    
}
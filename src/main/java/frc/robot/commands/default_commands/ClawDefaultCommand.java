package frc.robot.commands.default_commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Claw;

public class ClawDefaultCommand extends CommandBase{

    private Claw m_claw;
    private XboxController m_operatorController;

    public ClawDefaultCommand(Claw claw, XboxController operatorController) {

        m_claw = claw;
        m_operatorController = operatorController;

        addRequirements(m_claw);
    }
    
}
package frc.robot.commands.default_commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;

public class ArmDefaultCommand extends CommandBase{

    private final Arm m_arm;
    private final XboxController m_operatorController;

    public ArmDefaultCommand(Arm arm, XboxController operatorController) {
        
        m_arm = arm;
        m_operatorController = operatorController;

        addRequirements(m_arm);
    }

    @Override
    public void execute() {

        double m_moveValue;
        m_moveValue = m_operatorController.getLeftY();
        m_arm.freeMove(m_moveValue);

    }
    
}
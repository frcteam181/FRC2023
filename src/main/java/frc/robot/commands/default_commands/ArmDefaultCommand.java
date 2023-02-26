package frc.robot.commands.default_commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Turret;

public class ArmDefaultCommand extends CommandBase{

    private final Arm m_arm;
    private Turret m_turret;
    private final XboxController m_operatorController;

    public ArmDefaultCommand(Arm arm, Turret turret, XboxController operatorController) {
        
        m_arm = arm;
        m_turret = turret;
        m_operatorController = operatorController;

        addRequirements(m_arm, m_turret);
    }

    @Override
    public void execute() {

        double m_moveValue;
        //m_moveValue = m_operatorController.getLeftY();
        //m_arm.freeMovePivot(m_moveValue);
        //m_turret.move(m_moveValue);


    }
    
}
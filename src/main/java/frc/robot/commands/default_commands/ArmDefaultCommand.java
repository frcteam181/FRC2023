package frc.robot.commands.default_commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Turret;

public class ArmDefaultCommand extends CommandBase{

    private Arm m_arm;
    private Turret m_turret;
    private Claw m_claw;
    private final XboxController m_operatorController;

    public ArmDefaultCommand(Arm arm, Turret turret, Claw claw, XboxController operatorController) {
        
        m_arm = arm;
        m_turret = turret;
        m_claw = claw;
        m_operatorController = operatorController;

        addRequirements(m_arm, m_turret, m_claw);
    }

    @Override
    public void execute() {

        double m_moveValue;
        //m_moveValue = m_operatorController.getLeftY();
        //m_arm.freeMovePivot(m_moveValue);
        //m_turret.move(m_moveValue);


    }
    
}
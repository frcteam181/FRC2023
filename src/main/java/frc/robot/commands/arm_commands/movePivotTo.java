package frc.robot.commands.arm_commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;

public class movePivotTo extends CommandBase{

    private Arm m_arm;
    private double m_angle;

    public movePivotTo(Arm arm, double angle) {
        
        m_angle = angle;
        m_arm = arm;

        addRequirements(m_arm);
    }

    @Override
    public void initialize() {
        m_arm.movePivotTo(m_angle);
    }
    
}
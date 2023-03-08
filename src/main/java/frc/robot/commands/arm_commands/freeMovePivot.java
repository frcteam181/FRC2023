package frc.robot.commands.arm_commands;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Arm;

import static frc.robot.Constants.*;

public class freeMovePivot extends CommandBase{

    private Arm m_arm;
    private double m_goal, m_maxPivotAngle, m_minPivotAngle;
    private boolean m_isMovingUp;

    public freeMovePivot(Arm arm, boolean isMovingUp) {
        
        m_arm = arm;
        m_isMovingUp = isMovingUp;

        addRequirements(m_arm);
    }

    @Override
    public void initialize() {
        m_minPivotAngle = Units.radiansToDegrees(k_pivotOffset);
        m_maxPivotAngle = Units.radiansToDegrees(k_maxPivotAngle);
        m_goal = m_arm.getPivotAngleDeg();
    }

    @Override
    public void execute() {
        if (m_isMovingUp) {
            /*if (m_goal >= m_maxPivotAngle) {
                m_goal = m_maxPivotAngle;
            } else {
                m_goal += 5;
            }*/
            m_goal = m_maxPivotAngle;
        } else {
            /*if(m_goal <= m_minPivotAngle) {
                m_goal = m_minPivotAngle;
            } else {
                m_goal -= 5;
            }*/
            m_goal = m_minPivotAngle;
        }
    }

    @Override
    public void end(boolean interrupted) {
        m_arm.setPivotGoal(Units.degreesToRadians(m_goal)).andThen(new WaitCommand(2));
    }
    
}
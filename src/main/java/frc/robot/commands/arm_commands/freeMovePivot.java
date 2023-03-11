package frc.robot.commands.arm_commands;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;

import static frc.robot.Constants.*;

public class freeMovePivot extends CommandBase {

    private Arm m_arm;
    private double m_maxPivotAngle, m_minPivotAngle;
    private boolean m_isMovingUp;

    public freeMovePivot(Arm arm, boolean isMovingUp) {
        
        m_arm = arm;
        m_isMovingUp = isMovingUp;

        addRequirements(m_arm);
    }

    @Override
    public void initialize() {
        m_minPivotAngle = Units.radiansToDegrees(k_pivotOffsetRad);
        m_maxPivotAngle = Units.radiansToDegrees(k_maxPivotAngleRad);
        m_arm.disable();
    }

    @Override
    public void execute() {
        if (m_isMovingUp) {
            if(m_arm.getPivotAngleDeg() >= m_maxPivotAngle) {
                m_arm.freeMovePivot(0);
            } else {
                m_arm.freeMovePivot(0.2);
            }
        } else {
            if(m_arm.getPivotAngleDeg() <= m_minPivotAngle) {
                m_arm.freeMovePivot(0);
            } else {
                m_arm.freeMovePivot(-0.8);
            }
        }
        m_arm.updateState();
    }

    @Override
    public void end(boolean interrupted) {
        m_arm.enable();
    }
    
}
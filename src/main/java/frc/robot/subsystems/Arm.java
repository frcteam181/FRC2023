package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.*;

public class Arm extends SubsystemBase {

    private CANSparkMax m_leftPivot, m_rightPivot;
    private RelativeEncoder m_leftEncoder;
    private SparkMaxPIDController m_leftPivotPID;


    public Arm() {

        m_leftPivot = new CANSparkMax(k_LEFT_PIVOT, MotorType.kBrushless);
        m_rightPivot = new CANSparkMax(k_RIGHT_PIVOT, MotorType.kBrushless);

        m_leftPivot.restoreFactoryDefaults();
        m_rightPivot.restoreFactoryDefaults();

        m_leftEncoder = m_leftPivot.getEncoder();

        m_leftPivotPID = m_leftPivot.getPIDController();

        m_leftPivotPID.setP(k_PivotGains.kP, k_PIVOT_SLOT_ID);
        m_leftPivotPID.setI(k_PivotGains.kI, k_PIVOT_SLOT_ID);
        m_leftPivotPID.setD(k_PivotGains.kD, k_PIVOT_SLOT_ID);
        m_leftPivotPID.setIZone(k_PivotGains.kIzone, k_PIVOT_SLOT_ID);
        m_leftPivotPID.setFF(k_PivotGains.kFF, k_PIVOT_SLOT_ID);
        m_leftPivotPID.setOutputRange(k_PivotGains.kMinOutput, k_PivotGains.kMaxOutput, k_PIVOT_SLOT_ID);

        m_rightPivot.follow(m_leftPivot);

    }

    public double deadband(double value) {
        if (Math.abs(value) >= k_OperatorDeadband) {
            return value;
        } else {
            return 0;
        }
    }

    public void moveUp() {
        m_leftPivot.set(0.5);
    }
    
    public void moveDown() {
        m_leftPivot.set(-0.5);
    }

    public void stop() {
        m_leftPivot.set(0);
    }

    public void freeMove(double moveValue) {
        m_leftPivot.set(deadband(moveValue));
    }
}

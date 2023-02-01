package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.*;

public class Arm extends SubsystemBase {

    private CANSparkMax m_leftPivot, m_rightPivot, m_spool;
    private RelativeEncoder m_spoolEncoder;
    private AbsoluteEncoder m_leftEncoder;
    private SparkMaxPIDController m_leftPivotPID, m_spoolPID;

    private double m_ffEffort;


    public Arm() {

        m_leftPivot = new CANSparkMax(k_LEFT_PIVOT, MotorType.kBrushless);
        m_rightPivot = new CANSparkMax(k_RIGHT_PIVOT, MotorType.kBrushless);
        m_spool = new CANSparkMax(k_SPOOL, MotorType.kBrushless);

        m_leftPivot.restoreFactoryDefaults();
        m_rightPivot.restoreFactoryDefaults();
        m_spool.restoreFactoryDefaults();

        m_leftEncoder = m_leftPivot.getAbsoluteEncoder(Type.kDutyCycle);
        m_spoolEncoder = m_spool.getEncoder();

        m_leftPivotPID = m_leftPivot.getPIDController();
        m_spoolPID = m_spool.getPIDController();

        m_leftPivotPID.setP(k_PivotGains.kP, k_PIVOT_SLOT_ID);
        m_leftPivotPID.setI(k_PivotGains.kI, k_PIVOT_SLOT_ID);
        m_leftPivotPID.setD(k_PivotGains.kD, k_PIVOT_SLOT_ID);
        m_leftPivotPID.setIZone(k_PivotGains.kIzone, k_PIVOT_SLOT_ID);
        m_leftPivotPID.setFF(k_PivotGains.kFF, k_PIVOT_SLOT_ID);
        m_leftPivotPID.setOutputRange(k_PivotGains.kMinOutput, k_PivotGains.kMaxOutput, k_PIVOT_SLOT_ID);

        m_spoolPID.setP(k_SpoolGains.kP, k_SPOOL_SLOT_ID);
        m_spoolPID.setI(k_SpoolGains.kI, k_SPOOL_SLOT_ID);
        m_spoolPID.setD(k_SpoolGains.kD, k_SPOOL_SLOT_ID);
        m_spoolPID.setIZone(k_SpoolGains.kIzone, k_SPOOL_SLOT_ID);
        m_spoolPID.setFF(k_SpoolGains.kFF, k_SPOOL_SLOT_ID);
        m_spoolPID.setOutputRange(k_SpoolGains.kMinOutput, k_SpoolGains.kMaxOutput, k_SPOOL_SLOT_ID);

        m_leftPivot.setInverted(false);
        m_spool.setInverted(false);

        m_rightPivot.follow(m_leftPivot, true);

    }

    /*double ffEffort = Math.cos(armTheta) * calcArmKG(armExtension);
    double pidEffort = m_leftPivotPID.calculate(armAngle, armDesiredAngle);

    m_leftPivotPID.set(ffEffort + pidEffort);*/

    public double getArmTheta() {
        return m_leftEncoder.getPosition();
    }

    public double calcArmKG() {
        return (k_ArmKG_m_Gain * m_spoolEncoder.getPosition()) + k_ArmKG_b_Gain;  //mx+b
    }

    public void calcFFEffort() {
        m_ffEffort = Math.cos(getArmTheta()) * calcArmKG();
    }

    public double deadband(double value) {
        if (Math.abs(value) >= k_OperatorDeadband) {
            return value;
        } else {
            return 0;
        }
    }

    public void moveTo(double setpointAngle) {
        m_leftPivotPID.setReference(setpointAngle, ControlType.kPosition, k_PIVOT_SLOT_ID, m_ffEffort);
    }

    public void moveUp() {
        m_leftPivot.set(0.1);
    }
    
    public void moveDown() {
        m_leftPivot.set(-0.1);
    }

    public void stop() {
        m_leftPivot.set(0);
    }

    public void freeMove(double moveValue) {
        m_leftPivot.set(deadband(moveValue));
    }

    @Override
    public void periodic() {
        calcFFEffort();
    }
}

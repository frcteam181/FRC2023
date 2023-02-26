package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.*;

public class Arm2 extends SubsystemBase {

    private CANSparkMax m_leftPivot, m_rightPivot, m_spool;
    private RelativeEncoder m_pivotEncoder, m_spoolEncoder;
    private SparkMaxPIDController m_leftPivotPID, m_spoolPID;

    private double m_ffEffort;

    private DigitalInput m_armHomeBeam, m_armZeroBeam, m_spoolHomeBeam, m_spoolMaxBeam;
    private DoubleSolenoid m_claw;

    public Arm2() {

        m_leftPivot = new CANSparkMax(k_LEFT_PIVOT, MotorType.kBrushless);
        m_rightPivot = new CANSparkMax(k_RIGHT_PIVOT, MotorType.kBrushless);
        m_spool = new CANSparkMax(k_SPOOL, MotorType.kBrushless);

        m_leftPivot.restoreFactoryDefaults();
        m_rightPivot.restoreFactoryDefaults();
        m_spool.restoreFactoryDefaults();

        m_pivotEncoder = m_leftPivot.getEncoder();
        m_pivotEncoder.setPositionConversionFactor(k_pivotPosFac);
        m_pivotEncoder.setVelocityConversionFactor(k_pivotVelFac);
        m_pivotEncoder.setPosition(k_pivotOffset);

        m_spoolEncoder = m_spool.getEncoder();
        m_spoolEncoder.setPositionConversionFactor(1);

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

        m_armHomeBeam = new DigitalInput(k_PIVOT_HOME_BEAM_ID);
        m_armZeroBeam = new DigitalInput(k_PIVOT_ZERO_BEAM_ID);
        m_spoolHomeBeam = new DigitalInput(k_SPOOL_HOME_BEAM_ID);
        m_spoolMaxBeam = new DigitalInput(k_SPOOL_MAX_BEAM_ID);

        m_claw = new DoubleSolenoid(PneumaticsModuleType.REVPH, k_CLAW_GRAB, k_CLAW_RELEASE);

    }

    @Override
    public void periodic() {
        calcFFEffort();
    }

    public double getArmAngle() {
        return m_pivotEncoder.getPosition();
    }

    public double calcArmKG() {
        return ((k_pivotKG_m_Gain * m_spoolEncoder.getPosition()) + k_pivotKG_b_Gain);
    }

    public void calcFFEffort() {
        m_ffEffort = Math.cos(getArmAngle()) * calcArmKG();
    }

    public double deadband(double value) {
        if (Math.abs(value) >= k_OperatorDeadband) {
            return value;
        } else {
            return 0;
        }
    }

    public void movePivotTo(double setpointAngle) {
        m_leftPivotPID.setReference(setpointAngle, ControlType.kPosition, k_PIVOT_SLOT_ID, m_ffEffort);
    }

    public void pivotUp() {
        m_leftPivot.set(0.1);
    }
    
    public void pivotDown() {
        m_leftPivot.set(-0.01);
    }

    public void stopPivot() {
        m_leftPivot.set(0);
    }

    public void freeMovePivot(double speed) {
        m_leftPivot.set(deadband(speed));
    }

    public void extendSpool() {
        m_spool.set(0.1);
    }

    public void retractSpool() {
        m_spool.set(-0.1);
    }

    public void stopSpool() {
        m_spool.set(0);
    }

    public void freeMoveSpool(double speed) {
        m_spool.set(speed);
    }

    public void resetPivotEncoder() {
        m_pivotEncoder.setPosition(0);
    }

    public void resetSpoolEncoder() {
        m_spoolEncoder.setPosition(0);
    }

    public boolean isArmHome() {
        return !m_armHomeBeam.get();
    }

    public boolean isArmHorizontal() {
        return !m_armZeroBeam.get();
    }

    public boolean isSpoolHome() {
        return !m_spoolHomeBeam.get();
    }

    public boolean isSpoolMaxed() {
        return m_spoolMaxBeam.get();
    }

    public void grabClaw() {
        m_claw.set(Value.kForward);
    }

    public void releaseClaw() {
        m_claw.set(Value.kReverse);
    }

    public void toggleClaw() {
        if (m_claw.get() == Value.kForward) {
            m_claw.set(Value.kReverse);
        } else {
            m_claw.set(Value.kForward);
        }
    }
}

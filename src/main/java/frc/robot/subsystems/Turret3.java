package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.*;

public class Turret3 extends SubsystemBase {

    private CANSparkMax m_turretMotor;
    private RelativeEncoder m_encoder;
    private SparkMaxPIDController m_turretPID;
    private TrapezoidProfile.Constraints m_constraints;
    private TrapezoidProfile.State m_start, m_state, m_goal;
    private TrapezoidProfile m_turretProfile;
    private boolean m_enabled;
    private double m_period, m_arbFF;
    private AnalogEncoder m_absEncoder;

    public Turret3() {

        m_turretMotor = new CANSparkMax(k_TURRET, MotorType.kBrushless);

        m_turretMotor.restoreFactoryDefaults();

        m_turretMotor.setInverted(true);

        m_turretPID = m_turretMotor.getPIDController();

        m_turretPID.setP(k_TurretGains.kP, k_TURRET_SLOT_ID);
        m_turretPID.setI(k_TurretGains.kI, k_TURRET_SLOT_ID);
        m_turretPID.setD(k_TurretGains.kD, k_TURRET_SLOT_ID);
        m_turretPID.setIZone(k_TurretGains.kIzone, k_TURRET_SLOT_ID);
        m_turretPID.setFF(k_TurretGains.kFF, k_TURRET_SLOT_ID);
        m_turretPID.setOutputRange(k_TurretGains.kMinOutput, k_TurretGains.kMaxOutput, k_TURRET_SLOT_ID);

        m_arbFF = 0;

        m_encoder = m_turretMotor.getEncoder();
        m_encoder.setPositionConversionFactor(k_turretPosFac);
        m_encoder.setVelocityConversionFactor(k_turretVelFac);

        //m_absEncoder = new AnalogEncoder(0);

        m_constraints = new TrapezoidProfile.Constraints(k_maxPivotVel, k_maxTurretAcc);

        m_start = new TrapezoidProfile.State(0, 0);
        m_state = new TrapezoidProfile.State(0, 0);
        m_goal = new TrapezoidProfile.State(0, 0);

        m_turretProfile = new TrapezoidProfile(m_constraints, m_goal, m_start);

        m_enabled = true;

        m_period = 0.02;

    }

    @Override
    public void periodic() {
        updateProfile();
    }

    public void updateProfile() {
        m_turretProfile = new TrapezoidProfile(m_constraints, m_goal, m_state);
        m_state = m_turretProfile.calculate(m_period);
        if (m_enabled) {
        useState(m_state);
        }
    }

    public void setGoal(TrapezoidProfile.State goal) {
        m_goal = goal;
    }

    public void setGoal(double position) {
        setGoal(new TrapezoidProfile.State(position, 0));
    }

    public void disable() {
        m_enabled = false;
    }

    public void enable() {
        m_enabled = true;
    }

    public void useState(TrapezoidProfile.State setpoint) {
        m_turretPID.setReference(setpoint.position, ControlType.kPosition, k_TURRET_SLOT_ID, m_arbFF);
    }

    public double getAngle() {
        return m_encoder.getPosition();
    }

    public double getSpeed() {
        return m_encoder.getVelocity();
    }

    public void move(double value) {
        m_turretMotor.set(value);
    }

    public void stop() {
        m_turretMotor.set(0);
    }
    
}

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.*;
 
public class Turret extends SubsystemBase {

    private CANSparkMax m_turretMotor;
    private RelativeEncoder m_encoder;
    private SparkMaxPIDController m_turretPID;
    private SimpleMotorFeedforward m_turretFF;
    private double m_period, m_ffValue, m_setpointPos, m_setpointVel, m_turretKp, m_turretKi, m_turretKd, m_turretKs, m_turretTuningSetpoint, m_turretSetpointBuffer;
    private double[] m_response;
    private boolean m_isTuning, m_enabled;
    private ShuffleboardTab m_tab;
    private GenericEntry m_turretKpEntry, m_turretKiEntry, m_turretKdEntry, m_turretKsEntry, m_turretAngleEntry, m_turretSetpointEntry, m_turretOutputVoltageEntry, m_turretTuningSetpointEntry;
    private TrapezoidProfile.State m_state, m_goal;
    private TrapezoidProfile.Constraints m_constraints;

    public Turret(boolean isTuning) {

        TrapezoidProfileSubsystem(new TrapezoidProfile.Constraints(k_maxTurretVel, k_maxTurretAcc), k_turretOffset, 0.02);

        m_turretMotor = new CANSparkMax(k_TURRET, MotorType.kBrushless);

        m_turretMotor.restoreFactoryDefaults();

        m_turretMotor.setInverted(false);

        m_encoder = m_turretMotor.getEncoder();

        m_encoder.setPositionConversionFactor(k_turretPosFacRad);
        m_encoder.setVelocityConversionFactor(k_turretVelFacRadPerSec);

        m_turretPID = m_turretMotor.getPIDController();

        m_turretPID.setP(k_TurretGains.kP, k_TURRET_SLOT_ID);
        m_turretPID.setI(k_TurretGains.kI, k_TURRET_SLOT_ID);
        m_turretPID.setD(k_TurretGains.kD, k_TURRET_SLOT_ID);
        m_turretPID.setIZone(k_TurretGains.kIzone, k_TURRET_SLOT_ID);
        m_turretPID.setFF(k_TurretGains.kFF, k_TURRET_SLOT_ID);
        m_turretPID.setOutputRange(k_TurretGains.kMinOutput, k_TurretGains.kMaxOutput, k_TURRET_SLOT_ID);

        m_turretFF = new SimpleMotorFeedforward(k_turretKs, k_turretKv, k_turretKa);

        enable();

        /* Tuning */
        m_isTuning = isTuning;
        if(m_isTuning){tune();}

    }

    protected void useState(TrapezoidProfile.State setpoint) {
        m_setpointPos = setpoint.position;
        m_setpointVel = setpoint.velocity;
        //m_ffValue = m_turretFF.calculate(setpoint.position, setpoint.velocity);
        m_turretPID.setReference(setpoint.position, ControlType.kPosition, k_PIVOT_SLOT_ID/*, m_ffValue/12*/);
    }

    @Override
    public void periodic() {
        if(m_isTuning){tuningPeriodic();}

        var profile = new TrapezoidProfile(m_constraints, m_goal, m_state);
        m_state = profile.calculate(m_period);
        if (m_enabled) {
            useState(m_state);
        }
    }

    public void TrapezoidProfileSubsystem(TrapezoidProfile.Constraints constraints, double initialPosition, double period) {
        m_constraints = constraints;
        m_state = new TrapezoidProfile.State(initialPosition, 0);
        setGoal(initialPosition);
        m_period = period;
    }

    public void setGoal(TrapezoidProfile.State goal) {
        System.out.println("Goal Updated!");
        m_goal = goal;
    }

    public void setGoal(double goal) {
        System.out.println("Turret Goal Set!");
        setGoal(new TrapezoidProfile.State(goal, 0));
    }

    public Command setTurretGoal(double turretGoalRad) {
        System.out.println("Command Called!");
        return Commands.runOnce(() -> setGoal(turretGoalRad), this);
    }

    public void updateState() {
        m_state = new TrapezoidProfile.State(getAngleRad(), 0);
    }

    public Command setFinalTurretGoal() {
        enable();
        return Commands.runOnce(() -> setGoal(getAngleRad()), this);
    }

    public void disable() {
        m_enabled = false;
    }

    public void enable() {
        m_enabled = true;
    }

    public double getAngleRad() {
        return m_encoder.getPosition();
    }

    public double getAngleDeg() {
        return Units.radiansToDegrees(getAngleRad());
    }

    public double getSetpointAngleDeg() {
        return Units.radiansToDegrees(m_setpointPos);
    }

    public double getVelocity() {
        return m_encoder.getVelocity();
    }

    public double getSetpointVel() { 
        return m_setpointVel;
    }

    public void move(double value) {
        m_turretMotor.set(value);
    }

    public void stop() {
        m_turretMotor.set(0);
    }

    public double getOutputVoltage() {
        return (m_turretMotor.getBusVoltage() * m_turretMotor.getAppliedOutput());
    }

    public double[] getResponse() {
        m_response[0] = getSetpointAngleDeg();
        m_response[1] = getAngleDeg();
        return m_response;
    }

    /* Tuning */

    public void tune() {

        m_tab = Shuffleboard.getTab("Turret Tuning");

        // Turret
        m_turretKpEntry = m_tab.add("Turret Kp", k_TurretGains.kP).withPosition(0, 0).getEntry();
        m_turretKiEntry = m_tab.add("Turret Ki", k_TurretGains.kI).withPosition(0, 1).getEntry();
        m_turretKdEntry = m_tab.add("Turret Kd", k_TurretGains.kD).withPosition(0, 2).getEntry();
        m_turretKsEntry = m_tab.add("Turret Ks", k_turretKs).withPosition(0, 3).getEntry();

        m_turretKp = k_TurretGains.kP;
        m_turretKi = k_TurretGains.kI;
        m_turretKd = k_TurretGains.kD;
        m_turretKs = k_turretKs;
        m_response = new double[2];

        m_turretAngleEntry = m_tab.add("Turret Angle", getAngleRad()).withPosition(1, 0).getEntry();
        m_tab.addDoubleArray("Response", this::getResponse).withPosition(1, 1).withSize(3, 3).withWidget(BuiltInWidgets.kGraph);
        m_turretTuningSetpointEntry = m_tab.add("Turret Tuning Setpoint (deg)", m_turretTuningSetpoint).withPosition(1, 4).getEntry();
        m_turretSetpointEntry = m_tab.add("Turret Setpoint", getSetpointAngleDeg()).withPosition(2, 0).getEntry();
        m_turretOutputVoltageEntry = m_tab.add("Turret Output Voltage", getOutputVoltage()).withPosition(3, 0).getEntry();

    }

    public void tuningPeriodic() {

        // Turret
        var turretKp = m_turretKpEntry.getDouble(k_TurretGains.kP);
        var turretKi = m_turretKiEntry.getDouble(k_TurretGains.kI);
        var turretKd = m_turretKdEntry.getDouble(k_TurretGains.kD);
        var turretKs = m_turretKsEntry.getDouble(k_turretKs);

        m_turretSetpointBuffer = m_turretTuningSetpointEntry.getDouble(0);

        if(Math.abs(m_turretSetpointBuffer) >= 180) {
            if(m_turretSetpointBuffer > 0) {
                var turretTuningSetpoint = 180;
                if(turretTuningSetpoint != m_turretTuningSetpoint) {m_turretTuningSetpoint = turretTuningSetpoint;}
            } else {
                var turretTuningSetpoint = -180;
                if(turretTuningSetpoint != m_turretTuningSetpoint) {m_turretTuningSetpoint = turretTuningSetpoint;}
            }
        } else {
            var turretTuningSetpoint = m_turretSetpointBuffer;
            if(turretTuningSetpoint != m_turretTuningSetpoint) {m_turretTuningSetpoint = turretTuningSetpoint;}
        }

        if(turretKp != m_turretKp) {m_turretPID.setP(turretKp, k_TURRET_SLOT_ID);m_turretKp = turretKp;}
        if(turretKi != m_turretKi) {m_turretPID.setI(turretKi, k_TURRET_SLOT_ID);m_turretKi = turretKi;}
        if(turretKd != m_turretKd) {m_turretPID.setD(turretKd, k_TURRET_SLOT_ID);m_turretKd = turretKd;}
        if(turretKs != m_turretKs) {m_turretFF = new SimpleMotorFeedforward(turretKs, k_turretKv, k_turretKa);m_turretKs = turretKs;}

        m_turretAngleEntry.setDouble(getAngleRad());
        m_turretSetpointEntry.setDouble(getSetpointAngleDeg());
        m_turretOutputVoltageEntry.setDouble(getOutputVoltage());

    }

    public void moveTurretToTuning() {
        this.setTurretGoal(Units.degreesToRadians(m_turretTuningSetpoint));
    }
    
}

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

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

public class Arm extends SubsystemBase {

    private CANSparkMax m_leftPivot, m_rightPivot;
    private AbsoluteEncoder m_pivotEncoder;
    private SparkMaxPIDController m_pivotPID;
    private double m_ffEffort, m_pivotSetpointPos, m_period, m_pivotSetpointVel, m_pivotKp, m_pivotKi, m_pivotKd, m_pivotKG_b_Gain, m_pivotKG_m_Gain, m_pivotTuningSetpoint, m_pivotSetpointBuffer;
    private double[] m_pivotResponse;
    private boolean m_isTuning, m_enabled;
    private ShuffleboardTab m_tab;
    private GenericEntry m_pivotKpEntry, m_pivotKiEntry, m_pivotKdEntry, m_pivotAngleEntry, m_pivotSetpointAngleEntry, m_pivotOutputVoltageEntry, m_pivotKG_b_GainEntry, m_pivotKG_m_GainEntry, m_pivotTuningSetpointEntry;
    private TrapezoidProfile.Constraints m_constraints;
    private TrapezoidProfile.State m_state, m_goal;
    
    public Arm(boolean isTuning) {

        TrapezoidProfileSubsystem(new TrapezoidProfile.Constraints(k_maxPivotVel, k_maxPivotAcc), k_pivotHomeRad/*k_pivotOffsetRad*/, 0.02);

        // motors

        m_leftPivot = new CANSparkMax(k_LEFT_PIVOT, MotorType.kBrushless);
        m_rightPivot = new CANSparkMax(k_RIGHT_PIVOT, MotorType.kBrushless);

        m_leftPivot.restoreFactoryDefaults();
        m_rightPivot.restoreFactoryDefaults();

        // pivot

        m_rightPivot.follow(m_leftPivot, true);

        m_pivotEncoder = m_leftPivot.getAbsoluteEncoder(Type.kDutyCycle);

        m_pivotEncoder.setPositionConversionFactor(k_pivotPosFacRad);
        m_pivotEncoder.setVelocityConversionFactor(k_pivotVelFacRadPerSec);
        // m_pivotEncoder.setZeroOffset(k_pivotOffsetRad);

        m_pivotPID = m_leftPivot.getPIDController();
        m_pivotPID.setFeedbackDevice(m_pivotEncoder);

        m_pivotPID.setP(k_PivotGains.kP, k_PIVOT_SLOT_ID);
        m_pivotPID.setI(k_PivotGains.kI, k_PIVOT_SLOT_ID);
        m_pivotPID.setD(k_PivotGains.kD, k_PIVOT_SLOT_ID);
        m_pivotPID.setIZone(k_PivotGains.kIzone, k_PIVOT_SLOT_ID);
        m_pivotPID.setFF(k_PivotGains.kFF, k_PIVOT_SLOT_ID);
        m_pivotPID.setOutputRange(k_PivotGains.kMinOutput, k_PivotGains.kMaxOutput, k_PIVOT_SLOT_ID);

        m_pivotKG_b_Gain = k_pivotKG_b_Gain;

        enablePivotBreak();

        /* Tuning */
        m_isTuning = isTuning;
        if(m_isTuning){tune();}

    }

    public void TrapezoidProfileSubsystem(TrapezoidProfile.Constraints constraints, double initialPosition, double period) {
        m_constraints = constraints;
        m_state = new TrapezoidProfile.State(initialPosition, 0);
        setGoal(initialPosition);
        m_period = period;
    }

    public void setGoal(TrapezoidProfile.State goal) {
        m_goal = goal;
    }

    public void setGoal(double goal) {
        setGoal(new TrapezoidProfile.State(goal, 0));
    }

    /* Trapezoid & Overrride */ 

    protected void useState(TrapezoidProfile.State setpoint) {
        m_pivotSetpointPos = setpoint.position;
        m_pivotSetpointVel = setpoint.velocity;
        m_pivotPID.setReference(setpoint.position, CANSparkMax.ControlType.kPosition, k_PIVOT_SLOT_ID, m_ffEffort);
    }

    public Command setPivotGoal(double pivotGoalRad) {
        return Commands.runOnce(() -> setGoal(pivotGoalRad), this);
    }

    @Override
    public void periodic() {
        // calcFFEffort();
        calcFFEffortModified();
        if(m_isTuning){tuningPeriodic();}

        var profile = new TrapezoidProfile(m_constraints, m_goal, m_state);
        m_state = profile.calculate(m_period);
        if (m_enabled) {
            useState(m_state);
        }
    }

    /* Pivot */

    public double calcPivotKG() {
        return ((m_pivotKG_m_Gain * m_pivotEncoder.getPosition()) + m_pivotKG_b_Gain);
    }

    public void calcFFEffort() {
        m_ffEffort = Math.cos(getPivotAngleRadModified()) * calcPivotKG();
    }

    public void calcFFEffortModified() {
        m_ffEffort = Math.cos(Units.degreesToRadians(getPivotAngleDeg() - 188)) * calcPivotKG();
    }

    public void enablePivotBreak() {
        m_leftPivot.setIdleMode(IdleMode.kBrake);
    }

    public double getPivotAngleRad() {
        return m_pivotEncoder.getPosition();
        //return m_pivotEncoder.getPosition() - k_pivotThroughboreOffsetRad;
    }

    public double getPivotAngleRadModified() {
        return getPivotAngleRad() - k_addedAngleRad;
    }

    public double getPivotAngleDeg() {
        return Units.radiansToDegrees(getPivotAngleRad());
    }

    public double getPivotSetpointAngle() {
        return Units.radiansToDegrees(m_pivotSetpointPos);
    }

    public double[] getPivotResponse() {
        m_pivotResponse[0] = getPivotSetpointAngle();
        m_pivotResponse[1] = getPivotAngleDeg();
        return m_pivotResponse;
    }

    public double getPivotSetpointVel() {
        return m_pivotSetpointVel;
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

    //public void resetPivotEncoder() {
    //    m_pivotEncoder.setPosition(0);
    //}

    public double getPivotOutput() {
        return (m_leftPivot.getAppliedOutput() * m_leftPivot.getBusVoltage());
    }

    public void updateState() {
        m_state = new TrapezoidProfile.State(getPivotAngleRad(), 0);
    }

    public void disable() {
        m_enabled = false;
    }

    public void enable() {
        m_enabled = true;
    }

    //#region Utilities

    public double deadband(double value) {
        if (Math.abs(value) >= k_OperatorDeadband) {
            return value;
        } else {
            return 0;
        }
    }

    //#endregion Utilities

    //#region Tuning

    public void tune() {

        m_tab = Shuffleboard.getTab("Arm Tuning");

        // Pivot PID
        m_pivotKp = k_PivotGains.kP;
        m_pivotKi = k_PivotGains.kI;
        m_pivotKd = k_PivotGains.kD;

        m_pivotTuningSetpoint = Units.radiansToDegrees(k_pivotOffsetRad);

        // Pivot FF
        m_pivotResponse = new double[2];

        m_pivotKpEntry = m_tab.add("Pivot Kp", m_pivotKp).withPosition(0, 0).getEntry();
        m_pivotKiEntry = m_tab.add("Pivot Ki", m_pivotKi).withPosition(0, 1).getEntry();
        m_pivotKdEntry = m_tab.add("Pivot Kd", m_pivotKd).withPosition(0, 2).getEntry();
        m_pivotKG_b_GainEntry = m_tab.add("Pivot Kg_b", m_pivotKG_b_Gain).withPosition(0, 4).getEntry();
        m_pivotKG_m_GainEntry = m_tab.add("Pivot Kg_m", m_pivotKG_m_Gain).withPosition(0, 5).getEntry();

        m_pivotAngleEntry = m_tab.add("Pivot Angle", getPivotAngleDeg()).withPosition(1, 0).getEntry();
        m_tab.addDoubleArray("Pivot Response", this::getPivotResponse).withPosition(1, 1).withSize(3, 3).withWidget(BuiltInWidgets.kGraph);
        m_pivotTuningSetpointEntry = m_tab.add("Pivot Tuning Setpoint (deg)", m_pivotTuningSetpoint).withPosition(4, 3).getEntry();
        m_pivotSetpointAngleEntry = m_tab.add("Pivot Setpoint", getPivotSetpointAngle()).withPosition(2, 0).getEntry();
        m_pivotOutputVoltageEntry = m_tab.add("Pivot Output Voltage", getPivotOutput()).withPosition(3, 0).getEntry();

    }

    public void tuningPeriodic() {

        // Pivot
        var pivotKp = m_pivotKpEntry.getDouble(k_PivotGains.kP);
        var pivotKi = m_pivotKiEntry.getDouble(k_PivotGains.kI);
        var pivotKd = m_pivotKdEntry.getDouble(k_PivotGains.kD);
        var pivotKG_b_Gain = m_pivotKG_b_GainEntry.getDouble(k_pivotKG_b_Gain);
        var pivotKG_m_Gain = m_pivotKG_m_GainEntry.getDouble(k_pivotKG_m_Gain);

        if(pivotKp != m_pivotKp) {m_pivotPID.setP(pivotKp, k_PIVOT_SLOT_ID);m_pivotKp = pivotKp;}
        if(pivotKi != m_pivotKi) {m_pivotPID.setI(pivotKi, k_PIVOT_SLOT_ID);m_pivotKi = pivotKi;}
        if(pivotKd != m_pivotKd) {m_pivotPID.setD(pivotKd, k_PIVOT_SLOT_ID);m_pivotKd = pivotKd;}
        if(pivotKG_b_Gain != m_pivotKG_b_Gain) {m_pivotKG_b_Gain = pivotKG_b_Gain;}
        if(pivotKG_m_Gain != m_pivotKG_m_Gain) {m_pivotKG_m_Gain = pivotKG_m_Gain;}

        m_pivotAngleEntry.setDouble(getPivotAngleDeg());
        m_pivotSetpointAngleEntry.setDouble(getPivotSetpointAngle());
        m_pivotOutputVoltageEntry.setDouble(getPivotOutput());

        m_pivotSetpointBuffer = m_pivotTuningSetpointEntry.getDouble(Units.radiansToDegrees(k_pivotOffsetRad));

        if(m_pivotSetpointBuffer <= Units.radiansToDegrees(k_pivotOffsetRad)) {
            var pivotTuningSetpoint = Units.radiansToDegrees(k_pivotOffsetRad);
            if(pivotTuningSetpoint != m_pivotTuningSetpoint) {m_pivotTuningSetpoint = pivotTuningSetpoint;}
        } else if (m_pivotSetpointBuffer >= Units.radiansToDegrees(k_maxPivotAngleRad)) {
            var pivotTuningSetpoint = Units.radiansToDegrees(k_maxPivotAngleRad);
            if(pivotTuningSetpoint != m_pivotTuningSetpoint) {m_pivotTuningSetpoint = pivotTuningSetpoint;}
        } else {
            var pivotTuningSetpoint = m_pivotSetpointBuffer;
            if(pivotTuningSetpoint != m_pivotTuningSetpoint) {m_pivotTuningSetpoint = pivotTuningSetpoint;}
        }

    }

    public Command setTuningPivotGoal() {
        return Commands.runOnce(() -> setGoal(Units.degreesToRadians(m_pivotTuningSetpoint)), this);
    }

    public Command setFinalPivotGoal() {
        return Commands.runOnce(() -> setGoal(getPivotAngleRad()), this);
    }

    //#endregion Tuning

}

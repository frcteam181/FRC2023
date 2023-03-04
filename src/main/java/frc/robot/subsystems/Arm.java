package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.TrapezoidProfileSubsystem;

import static frc.robot.Constants.*;

public class Arm extends TrapezoidProfileSubsystem {

    private CANSparkMax m_leftPivot, m_rightPivot, m_spool;
    private RelativeEncoder m_pivotEncoder, m_spoolEncoder;
    private SparkMaxPIDController m_pivotPID, m_spoolPID;
    private ArmFeedforward m_armFF;
    private double m_ffValue, m_ffEffort, m_pivotSetpointPos, m_pivotSetpointVel, m_pivotKp, m_pivotKi, m_pivotKd, m_pivotKg, m_spoolKp, m_spoolKi, m_spoolKd, m_spoolSetpoint, m_pivotKG_b_Gain, m_pivotKG_m_Gain, m_pivotTuningSetpoint, m_spoolTuningSetpoint, m_pivotSetpointBuffer, m_spoolSetpointBuffer;
    private double[] m_pivotResponse, m_spoolResponse;
    private boolean m_isTuning;
    private DoubleSolenoid m_claw;
    private DigitalInput m_pivotHomeBeam, m_pivotZeroBeam, m_spoolHomeBeam, m_spoolMaxBeam;
    private ShuffleboardTab m_tab;
    private GenericEntry m_pivotKpEntry, m_pivotKiEntry, m_pivotKdEntry, m_pivotKgEntry, m_spoolKpEntry, m_spoolKiEntry, m_spoolKdEntry, m_pivotAngleEntry, m_pivotSetpointAngleEntry, m_pivotOutputVoltageEntry, m_spoolPosEntry, m_spoolSetpointEntry, m_spoolOutputVoltageEntry, m_pivotKG_b_GainEntry, m_pivotKG_m_GainEntry, m_pivotTuningSetpointEntry, m_spoolTuningSetpointEntry;
    private TrapezoidProfile.State m_pivotTuningState;
    
    public Arm() {

        super(new TrapezoidProfile.Constraints(k_maxPivotVel, k_maxPivotAcc), k_pivotOffset);

        // motors

        m_leftPivot = new CANSparkMax(k_LEFT_PIVOT, MotorType.kBrushless);
        m_rightPivot = new CANSparkMax(k_RIGHT_PIVOT, MotorType.kBrushless);
        m_spool = new CANSparkMax(k_SPOOL, MotorType.kBrushless);

        m_leftPivot.restoreFactoryDefaults();
        m_rightPivot.restoreFactoryDefaults();
        m_spool.restoreFactoryDefaults();

        // pivot

        m_rightPivot.follow(m_leftPivot, true);

        m_pivotEncoder = m_leftPivot.getEncoder();
        m_pivotEncoder.setPositionConversionFactor(k_pivotPosFac);
        m_pivotEncoder.setVelocityConversionFactor(k_pivotVelFac);
        m_pivotEncoder.setPosition(k_pivotOffset);

        m_pivotPID = m_leftPivot.getPIDController();

        m_pivotPID.setP(k_PivotGains.kP, k_PIVOT_SLOT_ID);
        m_pivotPID.setI(k_PivotGains.kI, k_PIVOT_SLOT_ID);
        m_pivotPID.setD(k_PivotGains.kD, k_PIVOT_SLOT_ID);
        m_pivotPID.setIZone(k_PivotGains.kIzone, k_PIVOT_SLOT_ID);
        m_pivotPID.setFF(k_PivotGains.kFF, k_PIVOT_SLOT_ID);
        m_pivotPID.setOutputRange(k_PivotGains.kMinOutput, k_PivotGains.kMaxOutput, k_PIVOT_SLOT_ID);

        m_armFF = new ArmFeedforward(k_pivotKs, k_pivotKg, k_pivotKv, k_pivotKa);

        m_pivotKG_b_Gain = k_pivotKG_b_Gain;

        enablePivotBreak();

        // spool

        m_spool.setInverted(false);

        m_spoolEncoder = m_spool.getEncoder();
        m_spoolEncoder.setPositionConversionFactor(k_spoolPosFac);

        m_spoolPID = m_spool.getPIDController();
 
        m_spoolPID.setP(k_SpoolGains.kP, k_SPOOL_SLOT_ID);
        m_spoolPID.setI(k_SpoolGains.kI, k_SPOOL_SLOT_ID);
        m_spoolPID.setD(k_SpoolGains.kD, k_SPOOL_SLOT_ID);
        m_spoolPID.setIZone(k_SpoolGains.kIzone, k_SPOOL_SLOT_ID);
        m_spoolPID.setFF(k_SpoolGains.kFF, k_SPOOL_SLOT_ID);
        m_spoolPID.setOutputRange(k_SpoolGains.kMinOutput, k_SpoolGains.kMaxOutput, k_SPOOL_SLOT_ID);

        enableSpoolBreak();

        // claw

        //m_claw = new DoubleSolenoid(PneumaticsModuleType.REVPH, k_CLAW_CLOSE, k_CLAW_OPEN);

        // Beams

        m_pivotHomeBeam = new DigitalInput(k_PIVOT_HOME_BEAM_ID);
        m_pivotZeroBeam = new DigitalInput(k_PIVOT_ZERO_BEAM_ID);
        m_spoolHomeBeam = new DigitalInput(k_SPOOL_HOME_BEAM_ID);
        m_spoolMaxBeam = new DigitalInput(k_SPOOL_MAX_BEAM_ID);

        /* Tuning */
        m_isTuning = true;
        if(m_isTuning){tune();}

    }

    /* Trapezoid & Overrride */

    @Override
    protected void useState(TrapezoidProfile.State setpoint) {
        m_pivotSetpointPos = setpoint.position;
        m_pivotSetpointVel = setpoint.velocity;
        m_ffValue = m_armFF.calculate(setpoint.position, setpoint.velocity);
        m_pivotPID.setReference(setpoint.position, ControlType.kPosition, k_PIVOT_SLOT_ID, m_ffEffort);
    }

    public Command setPivotGoal(double pivotGoalRad) {
        return Commands.runOnce(() -> setGoal(pivotGoalRad), this);
    }

    @Override
    public void periodic() {
        super.periodic();
        calcFFEffort();
        if(m_isTuning){tuningPeriodic();}
    }

    /* Pivot */

    public double calcPivotKG() {
        return ((m_pivotKG_m_Gain * m_pivotEncoder.getPosition()) + m_pivotKG_b_Gain);
    }

    public void calcFFEffort() {
        m_ffEffort = Math.cos(getPivotAngle()) * calcPivotKG();
    }

    public void enablePivotBreak() {
        m_leftPivot.setIdleMode(IdleMode.kBrake);
    }

    public double getPivotAngle() {
        return m_pivotEncoder.getPosition();
    }

    public double getPivotAngleDeg() {
        return Units.radiansToDegrees(getPivotAngle());
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

    public void resetPivotEncoder() {
        m_pivotEncoder.setPosition(0);
    }

    public double getPivotOutput() {
        return (m_leftPivot.getAppliedOutput() * m_leftPivot.getBusVoltage());
    }

    /* Spool */

    public double getSpoolPos() {
        return m_spoolEncoder.getPosition();
    }

    public double getSpoolSetpoint() {
        return m_spoolSetpoint;
    }

    public double[] getSpoolResponse() {
        m_spoolResponse[0] = getSpoolSetpoint();
        m_spoolResponse[1] = getSpoolPos();
        return m_spoolResponse;
    }

    public double getSpoolOutput() {
        return (m_spool.getAppliedOutput() * m_spool.getBusVoltage());
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

    public void resetSpoolEncoder() {
        m_spoolEncoder.setPosition(0);
    }

    public void moveSpoolTo(double setpoint) {
        m_spoolSetpoint = setpoint;
        m_spoolPID.setReference(setpoint, ControlType.kPosition, k_SPOOL_SLOT_ID);
    }

    public void enableSpoolBreak() {
        m_spool.setIdleMode(IdleMode.kBrake);
    }

    /* Beams */

    public boolean isPivotHome() {
        return !m_pivotHomeBeam.get();
    }

    public boolean isPivotHorizontal() {
        return !m_pivotZeroBeam.get();
    }

    public boolean isSpoolHome() {
        return !m_spoolHomeBeam.get();
    }

    public boolean isSpoolMaxed() {
        return m_spoolMaxBeam.get();
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

        m_pivotTuningSetpoint = k_pivotOffset;

        // Pivot FF
        m_pivotKg = k_pivotKg;
        m_pivotResponse = new double[2];

        m_pivotKpEntry = m_tab.add("Pivot Kp", m_pivotKp).withPosition(0, 0).getEntry();
        m_pivotKiEntry = m_tab.add("Pivot Ki", m_pivotKi).withPosition(0, 1).getEntry();
        m_pivotKdEntry = m_tab.add("Pivot Kd", m_pivotKd).withPosition(0, 2).getEntry();
        m_pivotKgEntry = m_tab.add("Pivot Kg", m_pivotKg).withPosition(0, 3).getEntry();
        m_pivotKG_b_GainEntry = m_tab.add("Pivot Kg_b", m_pivotKG_b_Gain).withPosition(0, 4).getEntry();
        m_pivotKG_m_GainEntry = m_tab.add("Pivot Kg_m", m_pivotKG_m_Gain).withPosition(0, 5).getEntry();

        m_pivotAngleEntry = m_tab.add("Pivot Angle", getPivotAngleDeg()).withPosition(1, 0).getEntry();
        m_tab.addDoubleArray("Pivot Response", this::getPivotResponse).withPosition(1, 1).withSize(3, 3).withWidget(BuiltInWidgets.kGraph);
        m_pivotTuningSetpointEntry = m_tab.add("Pivot Tuning Setpoint (deg)", m_pivotTuningSetpoint).withPosition(1, 4).getEntry();
        m_pivotSetpointAngleEntry = m_tab.add("Pivot Setpoint", getPivotSetpointAngle()).withPosition(2, 0).getEntry();
        m_pivotOutputVoltageEntry = m_tab.add("Pivot Output Voltage", getPivotOutput()).withPosition(3, 0).getEntry();

        // Spool
        m_spoolKp = k_SpoolGains.kP;
        m_spoolKi = k_SpoolGains.kI;
        m_spoolKd = k_SpoolGains.kD;
        m_spoolResponse = new double[2];

        m_spoolKpEntry = m_tab.add("Spool Kp", m_spoolKp).withPosition(4, 0).getEntry();
        m_spoolKiEntry = m_tab.add("Spool Ki", m_spoolKi).withPosition(4, 1).getEntry();
        m_spoolKdEntry = m_tab.add("Spool Kd", m_spoolKd).withPosition(4, 2).getEntry();

        m_spoolPosEntry = m_tab.add("Spool Position", getSpoolPos()).withPosition(5, 0).getEntry();
        m_tab.addDoubleArray("Spool Response", this::getSpoolResponse).withPosition(5, 1).withSize(3, 3).withWidget(BuiltInWidgets.kGraph);
        m_spoolTuningSetpointEntry = m_tab.add("Spool Tuning Setpoint (in)", m_spoolTuningSetpoint).withPosition(5, 4).getEntry();
        m_spoolSetpointEntry = m_tab.add("Spool Setpoint", getSpoolSetpoint()).withPosition(6, 0).getEntry();
        m_spoolOutputVoltageEntry = m_tab.add("Spool Output Voltage", getSpoolOutput()).withPosition(7, 0).getEntry();

    }

    public void tuningPeriodic() {

        // Pivot
        var pivotKp = m_pivotKpEntry.getDouble(k_PivotGains.kP);
        var pivotKi = m_pivotKiEntry.getDouble(k_PivotGains.kI);
        var pivotKd = m_pivotKdEntry.getDouble(k_PivotGains.kD);
        var pivotKg = m_pivotKgEntry.getDouble(k_pivotKg);
        var pivotKG_b_Gain = m_pivotKG_b_GainEntry.getDouble(k_pivotKG_b_Gain);
        var pivotKG_m_Gain = m_pivotKG_m_GainEntry.getDouble(k_pivotKG_m_Gain);

        if(pivotKp != m_pivotKp) {m_pivotPID.setP(pivotKp, k_PIVOT_SLOT_ID);m_pivotKp = pivotKp;}
        if(pivotKi != m_pivotKi) {m_pivotPID.setI(pivotKi, k_PIVOT_SLOT_ID);m_pivotKi = pivotKi;}
        if(pivotKd != m_pivotKd) {m_pivotPID.setD(pivotKd, k_PIVOT_SLOT_ID);m_pivotKd = pivotKd;}
        if(pivotKg != m_pivotKg) {m_armFF = new ArmFeedforward(k_pivotKs, pivotKg, k_pivotKv);m_pivotKg = pivotKg;}
        if(pivotKG_b_Gain != m_pivotKG_b_Gain) {m_pivotKG_b_Gain = pivotKG_b_Gain;}
        if(pivotKG_m_Gain != m_pivotKG_m_Gain) {m_pivotKG_m_Gain = pivotKG_m_Gain;}

        m_pivotAngleEntry.setDouble(getPivotAngleDeg());
        m_pivotSetpointAngleEntry.setDouble(getPivotSetpointAngle());
        m_pivotOutputVoltageEntry.setDouble(getPivotOutput());

        m_pivotSetpointBuffer = m_pivotTuningSetpointEntry.getDouble(Units.radiansToDegrees(k_pivotOffset));

        if(m_pivotSetpointBuffer <= Units.radiansToDegrees(k_pivotOffset)) {
            var pivotTuningSetpoint = Units.radiansToDegrees(k_pivotOffset);
            if(pivotTuningSetpoint != m_pivotTuningSetpoint) {m_pivotTuningSetpoint = pivotTuningSetpoint;}
        } else if (m_pivotSetpointBuffer >= Units.radiansToDegrees(k_maxPivotRad)) {
            var pivotTuningSetpoint = Units.radiansToDegrees(k_maxPivotRad);
            if(pivotTuningSetpoint != m_pivotTuningSetpoint) {m_pivotTuningSetpoint = pivotTuningSetpoint;}
        } else {
            var pivotTuningSetpoint = m_pivotSetpointBuffer;
            if(pivotTuningSetpoint != m_pivotTuningSetpoint) {m_pivotTuningSetpoint = pivotTuningSetpoint;}
        }

        // Spool
        var spoolKp = m_spoolKpEntry.getDouble(k_SpoolGains.kP);
        var spoolKi = m_spoolKiEntry.getDouble(k_SpoolGains.kI);
        var spoolKd = m_spoolKdEntry.getDouble(k_SpoolGains.kD);

        if(spoolKp != m_spoolKp) {m_spoolPID.setP(spoolKp, k_SPOOL_SLOT_ID);m_spoolKp = spoolKp;}
        if(spoolKi != m_spoolKi) {m_spoolPID.setI(spoolKi, k_SPOOL_SLOT_ID);m_spoolKi = spoolKi;}
        if(spoolKd != m_spoolKd) {m_spoolPID.setD(spoolKd, k_SPOOL_SLOT_ID);m_spoolKd = spoolKd;}

        m_spoolPosEntry.setDouble(getSpoolPos());
        m_spoolSetpointEntry.setDouble(getSpoolSetpoint());
        m_spoolOutputVoltageEntry.setDouble(getSpoolOutput());

        m_spoolSetpointBuffer = m_spoolTuningSetpointEntry.getDouble(0);
        if(m_spoolSetpointBuffer > Units.metersToInches(k_maxSpoolExtention)) {
            var spoolTuningSetpoint = Units.metersToInches(k_maxSpoolExtention);
            if(spoolTuningSetpoint != m_spoolTuningSetpoint) {m_spoolTuningSetpoint = spoolTuningSetpoint;}
        } else if (m_spoolSetpointBuffer <= 0) {
            var spoolTuningSetpoint = 0;
            if(spoolTuningSetpoint != m_spoolTuningSetpoint) {m_spoolTuningSetpoint = spoolTuningSetpoint;}
        } else {
            var spoolTuningSetpoint = m_spoolSetpointBuffer;
            if(spoolTuningSetpoint != m_spoolTuningSetpoint) {m_spoolTuningSetpoint = spoolTuningSetpoint;}
        }

    }

    public void movePivotToTuning() {
        this.setPivotGoal(Units.degreesToRadians(m_pivotTuningSetpoint));
    }

    public void moveSpoolToTuning() {
        moveSpoolTo(Units.inchesToMeters(m_spoolTuningSetpoint));
    }

    //#endregion Tuning

}

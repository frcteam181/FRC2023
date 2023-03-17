package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.*;

public class Spool extends SubsystemBase {

    private CANSparkMax m_spool;
    private RelativeEncoder m_spoolEncoder;
    private SparkMaxPIDController m_spoolPID;
    private double m_spoolKp, m_spoolKi, m_spoolKd, m_spoolSetpoint, m_spoolTuningSetpoint, m_spoolSetpointBuffer;
    private double[] m_spoolResponse;
    private boolean m_isTuning;
    private ShuffleboardTab m_tab;
    private GenericEntry m_spoolKpEntry, m_spoolKiEntry, m_spoolKdEntry, m_spoolPosEntry, m_spoolSetpointEntry, m_spoolOutputVoltageEntry, m_spoolTuningSetpointEntry;
    
    public Spool(boolean isTuning) {

        // motors
        m_spool = new CANSparkMax(k_SPOOL, MotorType.kBrushless);

        m_spool.restoreFactoryDefaults();

        m_spool.setInverted(false);

        m_spoolEncoder = m_spool.getEncoder();

        m_spoolEncoder.setPositionConversionFactor(k_spoolPosFacMeter);
        m_spoolEncoder.setVelocityConversionFactor(k_spoolVelFacMeterPerSec);

        m_spoolPID = m_spool.getPIDController();
 
        m_spoolPID.setP(k_SpoolGains.kP, k_SPOOL_SLOT_ID);
        m_spoolPID.setI(k_SpoolGains.kI, k_SPOOL_SLOT_ID);
        m_spoolPID.setD(k_SpoolGains.kD, k_SPOOL_SLOT_ID);
        m_spoolPID.setIZone(k_SpoolGains.kIzone, k_SPOOL_SLOT_ID);
        m_spoolPID.setFF(k_SpoolGains.kFF, k_SPOOL_SLOT_ID);
        m_spoolPID.setOutputRange(k_SpoolGains.kMinOutput, k_SpoolGains.kMaxOutput, k_SPOOL_SLOT_ID);

        enableBreak();

        /* Tuning */
        m_isTuning = isTuning;
        if(m_isTuning){tune();}

    }

    @Override
    public void periodic() {
        if(m_isTuning){tuningPeriodic();}
    }

    public void resetSpoolEncoder() {
        m_spoolEncoder.setPosition(0);
    }

    public double getSpoolPos() {
        return m_spoolEncoder.getPosition();
    }

    public double getSpoolPosSetpoint() {
        return m_spoolSetpoint;
    }
    

    public double[] getSpoolResponse() {
        m_spoolResponse[0] = getSpoolPosSetpoint();
        m_spoolResponse[1] = getSpoolPos();
        return m_spoolResponse;
    }

    public double getSpoolOutputVoltage() {
        return (m_spool.getAppliedOutput() * m_spool.getBusVoltage());
    }

    public void extend() {
        m_spool.set(0.3);
    }

    public void retract() {
        m_spool.set(-0.3);
    }

    public void stop() {
        m_spool.set(0);
    }

    public Command extendCommand() {
        return Commands.runOnce(() -> extend(), this);
    }

    public Command retractCommand() {
        return Commands.runOnce(() -> retract(), this);
    }

    public Command stopCommand() {
        return Commands.runOnce(() -> stop(), this);
    }

    public void freeMove(double speed) {
        m_spool.set(speed);
    }

    public void moveTo(double setpoint) {
        System.out.println("pid setpoint updated");
        m_spoolSetpoint = setpoint;
        m_spoolPID.setReference(setpoint, CANSparkMax.ControlType.kPosition, k_SPOOL_SLOT_ID);
    }

    public boolean isSpoolAtGoal() {
        if(Math.abs(getSpoolPos() - getSpoolPosSetpoint()) < 0.0508) {
            return true;
        } else {
            return false;
        }
    }

    public Command moveToCommand(double setpointMeters) {
        return Commands.runOnce(() -> moveTo(setpointMeters), this);
    }

    public void enableBreak() {
        m_spool.setIdleMode(IdleMode.kBrake);
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

        m_tab = Shuffleboard.getTab("Spool Tuning");

        m_spoolKp = k_SpoolGains.kP;
        m_spoolKi = k_SpoolGains.kI;
        m_spoolKd = k_SpoolGains.kD;
        m_spoolResponse = new double[2];

        m_spoolKpEntry = m_tab.add("Spool Kp", m_spoolKp).withPosition(4, 0).getEntry();
        m_spoolKiEntry = m_tab.add("Spool Ki", m_spoolKi).withPosition(4, 1).getEntry();
        m_spoolKdEntry = m_tab.add("Spool Kd", m_spoolKd).withPosition(4, 2).getEntry();

        m_spoolPosEntry = m_tab.add("Spool Position", getSpoolPos()).withPosition(5, 0).getEntry();
        m_tab.addDoubleArray("Spool Response", this::getSpoolResponse).withPosition(5, 1).withSize(3, 3).withWidget(BuiltInWidgets.kGraph);
        m_spoolTuningSetpointEntry = m_tab.add("Spool Tuning Setpoint (in)", m_spoolTuningSetpoint).withPosition(8, 3).getEntry();
        m_spoolSetpointEntry = m_tab.add("Spool Setpoint", getSpoolPosSetpoint()).withPosition(6, 0).getEntry();
        m_spoolOutputVoltageEntry = m_tab.add("Spool Output Voltage", getSpoolOutputVoltage()).withPosition(7, 0).getEntry();

    }

    public void tuningPeriodic() {

        // Spool
        var spoolKp = m_spoolKpEntry.getDouble(k_SpoolGains.kP);
        var spoolKi = m_spoolKiEntry.getDouble(k_SpoolGains.kI);
        var spoolKd = m_spoolKdEntry.getDouble(k_SpoolGains.kD);

        if(spoolKp != m_spoolKp) {m_spoolPID.setP(spoolKp, k_SPOOL_SLOT_ID);m_spoolKp = spoolKp;}
        if(spoolKi != m_spoolKi) {m_spoolPID.setI(spoolKi, k_SPOOL_SLOT_ID);m_spoolKi = spoolKi;}
        if(spoolKd != m_spoolKd) {m_spoolPID.setD(spoolKd, k_SPOOL_SLOT_ID);m_spoolKd = spoolKd;}

        m_spoolPosEntry.setDouble(getSpoolPos());
        m_spoolSetpointEntry.setDouble(getSpoolPosSetpoint());
        m_spoolOutputVoltageEntry.setDouble(getSpoolOutputVoltage());

        m_spoolSetpointBuffer = m_spoolTuningSetpointEntry.getDouble(0);
        
        if(m_spoolSetpointBuffer > Units.metersToInches(k_maxSpoolExtentionMeter)) {
            var spoolTuningSetpoint = Units.metersToInches(k_maxSpoolExtentionMeter);
            if(spoolTuningSetpoint != m_spoolTuningSetpoint) {m_spoolTuningSetpoint = spoolTuningSetpoint;}
        } else if (m_spoolSetpointBuffer <= 0) {
            var spoolTuningSetpoint = 0;
            if(spoolTuningSetpoint != m_spoolTuningSetpoint) {m_spoolTuningSetpoint = spoolTuningSetpoint;}
        } else {
            var spoolTuningSetpoint = m_spoolSetpointBuffer;
            if(spoolTuningSetpoint != m_spoolTuningSetpoint) {m_spoolTuningSetpoint = spoolTuningSetpoint;}
        }

    }

    public Command moveToTuning() {
        return Commands.runOnce(() -> moveTo(Units.inchesToMeters(m_spoolTuningSetpoint)), this);
    }

    //#endregion Tuning

}
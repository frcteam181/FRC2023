package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.*;

public class Claw extends SubsystemBase {

    private CANSparkMax m_leftMotor, m_rightMotor;
    private RelativeEncoder m_encoder;
    private SparkMaxPIDController m_clawPID;
    private boolean m_isTuning, m_isPistonOpen;
    private ShuffleboardTab m_tab;
    private GenericEntry m_clawKpEntry, m_clawKiEntry, m_clawKdEntry, m_clawFFEntry, m_clawTuningSpeedEntry;
    private double m_intakeSpeed, m_clawKp, m_clawKi, m_clawKd, m_clawFF, m_setpointVel, m_clawTuningSpeed, m_clawSpeedBuffer;
    private double[] m_response;
    private PneumaticHub m_ph;
    private DoubleSolenoid m_piston;
    private SimpleMotorFeedforward m_clawMotorFF;
    private DigitalInput m_hasPiece;

    public Claw(boolean isTuning) {

        m_leftMotor = new CANSparkMax(k_LEFT_CLAW, MotorType.kBrushless);
        m_rightMotor = new CANSparkMax(k_RIGHT_CLAW, MotorType.kBrushless);

        m_leftMotor.restoreFactoryDefaults();
        m_rightMotor.restoreFactoryDefaults();

        m_rightMotor.follow(m_leftMotor, true);

        m_encoder = m_leftMotor.getEncoder();
        m_encoder.setPositionConversionFactor(k_clawPosFacMeter);
        m_encoder.setVelocityConversionFactor(k_clawVelFacMeterPerSec);

        m_clawPID = m_leftMotor.getPIDController();

        m_clawPID.setP(k_ClawGains.kP,k_CLAW_SLOT_ID);
        m_clawPID.setI(k_ClawGains.kI,k_CLAW_SLOT_ID);
        m_clawPID.setD(k_ClawGains.kD,k_CLAW_SLOT_ID);
        m_clawPID.setIZone(k_ClawGains.kIzone,k_CLAW_SLOT_ID);
        m_clawPID.setFF(k_ClawGains.kFF, k_CLAW_SLOT_ID);
        m_clawPID.setOutputRange(k_ClawGains.kMinOutput, k_ClawGains.kMaxOutput, k_CLAW_SLOT_ID);

        m_clawMotorFF = new SimpleMotorFeedforward(0.19943, 3.2742, 0.057855);

        m_ph = new PneumaticHub(k_PH);

        m_ph.enableCompressorAnalog(k_minPressure, k_maxPressure);

        m_piston = m_ph.makeDoubleSolenoid(k_CLAW_OPEN, k_CLAW_CLOSE);

        m_intakeSpeed = k_intakeSpeed;

        close();

        m_hasPiece = new DigitalInput(0);

        m_isTuning = isTuning;
        if(m_isTuning) {tune();}

        // m_leftMotor.setSecondaryCurrentLimit(10);
        // m_rightMotor.setSecondaryCurrentLimit(10);

        m_leftMotor.setIdleMode(IdleMode.kBrake);
        m_rightMotor.setIdleMode(IdleMode.kBrake);

    }

    @Override
    public void periodic() {
        if(m_isTuning) {periodicTuning();}
    }

    /* Motors */

    public void setSpeed(double setpointVel) {
        m_setpointVel = setpointVel;
        var clawFFEffort = m_clawMotorFF.calculate(setpointVel);
        m_clawPID.setReference(setpointVel, CANSparkMax.ControlType.kVelocity, k_CLAW_SLOT_ID, clawFFEffort);
    }

    public void setSpeedWithBeam(double setpointVel) {
        m_setpointVel = setpointVel;
        if(hasPiece()) {
            stopIntake();
        } else {
            var clawFFEffort = m_clawMotorFF.calculate(setpointVel);
            m_clawPID.setReference(setpointVel, CANSparkMax.ControlType.kVelocity, k_CLAW_SLOT_ID, clawFFEffort);
        }
    }

    public void intakeOn() {
        m_leftMotor.set(m_intakeSpeed);
    }

    public void outake() {
        m_leftMotor.set(m_intakeSpeed * -0.5);
    }

    public void stopIntake() {
        m_leftMotor.set(0);
    }

    public double getSetpointVel() {
        return m_setpointVel;
    }

    public double getSpeed() {
        return m_encoder.getVelocity();
    }

    public double[] getResponse() {
        m_response[0] = getSetpointVel();
        m_response[1] = getSpeed();
        return m_response;
    }

    /* Piston */

    public void open() {
        m_isPistonOpen = true;
        m_piston.set(DoubleSolenoid.Value.kForward);
    }

    public void close() {
        m_isPistonOpen = false;
        m_piston.set(DoubleSolenoid.Value.kReverse);
    }

    public void switchModes() {
        if(m_isPistonOpen) {
            close();
        } else {
            open();
        }
    }

    public Command switchModesCommand() {
        System.out.println("Switch Command Called");
        return Commands.runOnce(() -> switchModes(), this);
    }

    public double getStoredPSI() {
        return m_ph.getPressure(k_STORED_PSI);
    }
    
    public double getWorkingPSI() {
        return m_ph.getPressure(k_WORKING_PSI);
    }
    
    public boolean getCompressorStatus() {
        return m_ph.getCompressor();
    }

    public double getLeftIntakeVoltage() {
        return (m_leftMotor.getAppliedOutput() * m_leftMotor.getBusVoltage());
    }

    public double getRightIntakeVoltage() {
        return (m_rightMotor.getAppliedOutput() * m_rightMotor.getBusVoltage());
    }

    /* Tuning */

    public void tune() {

        m_tab = Shuffleboard.getTab("Claw Tuning");

        // Velocity PID
        m_clawKp = k_ClawGains.kP;
        m_clawKi = k_ClawGains.kI;
        m_clawKd = k_ClawGains.kD; 

        m_setpointVel = 0;

        m_response = new double[2];

        m_clawKpEntry = m_tab.add("Claw Kp", m_clawKp).withPosition(0, 0).getEntry();
        m_clawKiEntry = m_tab.add("Claw Ki", m_clawKi).withPosition(0, 1).getEntry();
        m_clawKdEntry = m_tab.add("Claw Kd", m_clawKd).withPosition(0, 2).getEntry();
        m_clawFFEntry = m_tab.add("Claw FF", m_clawFF).withPosition(0, 3).getEntry();

        m_tab.addNumber("Left Voltage", this::getLeftIntakeVoltage).withPosition(4, 0);
        m_tab.addNumber("Right Voltage", this::getRightIntakeVoltage).withPosition(5, 0);

        m_tab.addDoubleArray("Claw Response", this::getResponse).withPosition(1, 1).withSize(3, 3).withWidget(BuiltInWidgets.kGraph);
        m_clawTuningSpeedEntry = m_tab.add("Claw Tuning Speed (meter sec)", m_clawTuningSpeed).withPosition(1, 0).getEntry();

    }

    public void periodicTuning() {

        var clawKp = m_clawKpEntry.getDouble(k_ClawGains.kP);
        var clawKi = m_clawKiEntry.getDouble(k_ClawGains.kI);
        var clawKd = m_clawKdEntry.getDouble(k_ClawGains.kD);
        var clawFF = m_clawFFEntry.getDouble(k_ClawGains.kFF);

        if(clawKp != m_clawKp) {m_clawPID.setP(clawKp, k_CLAW_SLOT_ID);m_clawKp = clawKp;}
        if(clawKi != m_clawKi) {m_clawPID.setI(clawKi, k_CLAW_SLOT_ID);m_clawKi = clawKi;}
        if(clawKd != m_clawKd) {m_clawPID.setD(clawKd, k_CLAW_SLOT_ID);m_clawKd = clawKd;}
        if(clawFF != m_clawFF) {m_clawPID.setFF(clawFF, k_CLAW_SLOT_ID);m_clawFF = clawFF;}

        m_clawSpeedBuffer = m_clawTuningSpeedEntry.getDouble(0);

        if(Math.abs(m_clawSpeedBuffer) >= k_maxClawVelMeter) {
            var clawTuningSpeed = k_maxClawVelMeter;
            if(clawTuningSpeed != m_clawTuningSpeed) {m_clawTuningSpeed = clawTuningSpeed;}
        } else {
            var clawTuningSpeed = m_clawSpeedBuffer;
            if(clawTuningSpeed != m_clawTuningSpeed) {m_clawTuningSpeed = clawTuningSpeed;}
        }

    }

    public void setClawSpeedToTuning() {
        setSpeed(m_clawTuningSpeed);
    } 

    public boolean hasPiece() {
        return !m_hasPiece.get();
    }

    public void spitCube() {
        m_leftMotor.set(-0.12);
    }

    public Command spitCubeCommand() {
        return Commands.runOnce(() -> spitCube(), this);
    }
      
    public Command stopIntakeCommand() {
        return Commands.runOnce(() -> stopIntake(), this);
    }
      
    
}

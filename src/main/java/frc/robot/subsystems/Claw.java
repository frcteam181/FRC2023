package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.*;

public class Claw extends SubsystemBase {

    private CANSparkMax m_leftMotor, m_rightMotor;
    private RelativeEncoder m_encoder;
    private SparkMaxPIDController m_clawPID;
    private boolean m_isTuning;
    private ShuffleboardTab m_tab;
    private GenericEntry m_clawKpEntry, m_clawKiEntry, m_clawKdEntry, m_clawTuningSpeedEntry;
    private double m_clawKp, m_clawKi, m_clawKd, m_setpointVel, m_clawTuningSpeed, m_clawSpeedBuffer;
    private double[] m_response;
    private DoubleSolenoid m_piston;

    public Claw() {

        m_leftMotor = new CANSparkMax(k_LEFT_CLAW, MotorType.kBrushless);
        m_rightMotor = new CANSparkMax(k_RIGHT_CLAW, MotorType.kBrushless);

        m_leftMotor.restoreFactoryDefaults();
        m_rightMotor.restoreFactoryDefaults();

        m_rightMotor.follow(m_leftMotor, true);

        //m_leftMotor.setOpenLoopRampRate(0.2);
        m_leftMotor.setClosedLoopRampRate(0.2);

        m_encoder = m_leftMotor.getEncoder();
        m_encoder.setVelocityConversionFactor(k_clawVelFac);

        m_clawPID = m_leftMotor.getPIDController();

        m_clawPID.setP(k_ClawGains.kP,k_CLAW_SLOT_ID);
        m_clawPID.setI(k_ClawGains.kI,k_CLAW_SLOT_ID);
        m_clawPID.setD(k_ClawGains.kD,k_CLAW_SLOT_ID);
        m_clawPID.setIZone(k_ClawGains.kIzone,k_CLAW_SLOT_ID);
        m_clawPID.setFF(k_ClawGains.kFF, k_CLAW_SLOT_ID);
        m_clawPID.setOutputRange(k_ClawGains.kMinOutput, k_ClawGains.kMaxOutput, k_CLAW_SLOT_ID);

        m_piston = new DoubleSolenoid(PneumaticsModuleType.REVPH, k_CLAW_OPEN, k_CLAW_CLOSE);

        m_isTuning = true;
        if(m_isTuning) {tune();}

    }

    @Override
    public void periodic() {
        if(m_isTuning) {periodicTuning();}
    }

    /* Motors */

    public void setSpeed(double setpointVel) {
        m_setpointVel = setpointVel;
        m_clawPID.setReference(setpointVel, ControlType.kVelocity, k_CLAW_SLOT_ID);
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
        m_piston.set(Value.kForward);
    }

    public void close() {
        m_piston.set(Value.kReverse);
    }

    public void switchModes() {
        m_piston.toggle();
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

        m_tab.addDoubleArray("Claw Response", this::getResponse).withPosition(1, 1).withSize(3, 3).withWidget(BuiltInWidgets.kGraph);
        m_clawTuningSpeedEntry = m_tab.add("Claw Tuning Speed (m/s)", m_clawTuningSpeed).withPosition(1, 4).getEntry();

    }

    public void periodicTuning() {

        var clawKp = m_clawKpEntry.getDouble(k_ClawGains.kP);
        var clawKi = m_clawKiEntry.getDouble(k_ClawGains.kI);
        var clawKd = m_clawKdEntry.getDouble(k_ClawGains.kD);

        if(clawKp != m_clawKp) {m_clawPID.setP(clawKp, k_CLAW_SLOT_ID);m_clawKp = clawKp;}
        if(clawKi != m_clawKi) {m_clawPID.setI(clawKi, k_CLAW_SLOT_ID);m_clawKi = clawKi;}
        if(clawKd != m_clawKd) {m_clawPID.setD(clawKd, k_CLAW_SLOT_ID);m_clawKd = clawKd;}

        m_clawSpeedBuffer = m_clawTuningSpeedEntry.getDouble(0);

        if(Math.abs(m_clawSpeedBuffer) >= k_maxClawVel) {
            var clawTuningSpeed = k_maxClawVel;
            if(clawTuningSpeed != m_clawTuningSpeed) {m_clawTuningSpeed = clawTuningSpeed;}
        } else {
            var clawTuningSpeed = m_clawSpeedBuffer;
            if(clawTuningSpeed != m_clawTuningSpeed) {m_clawTuningSpeed = clawTuningSpeed;}
        }

    }

    public void setClawSpeedToTuning() {
        setSpeed(m_clawTuningSpeed);
    }
    
}

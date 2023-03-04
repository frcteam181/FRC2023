package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.*;

public class Intake extends SubsystemBase {

    private CANSparkMax m_leftIntake, m_rightIntake;
    private RelativeEncoder m_encoder;
    private SparkMaxPIDController m_intakePID;
    

    public Intake() {

        m_leftIntake = new CANSparkMax(k_LEFT_CLAW /*k_LEFT_INTAKE*/, MotorType.kBrushless);
        m_rightIntake = new CANSparkMax(k_RIGHT_CLAW /*k_LEFT_INTAKE*/, MotorType.kBrushless);

        m_leftIntake.restoreFactoryDefaults();
        m_rightIntake.restoreFactoryDefaults();

        m_rightIntake.follow(m_leftIntake, true);
        
        m_encoder = m_leftIntake.getEncoder();

        m_intakePID = m_leftIntake.getPIDController();

        m_intakePID.setP(k_IntakeGains.kP, k_INTAKE_SLOT_ID);
        m_intakePID.setI(k_IntakeGains.kI, k_INTAKE_SLOT_ID);
        m_intakePID.setD(k_IntakeGains.kD, k_INTAKE_SLOT_ID);
        m_intakePID.setIZone(k_IntakeGains.kIzone, k_INTAKE_SLOT_ID);
        m_intakePID.setFF(k_IntakeGains.kFF, k_INTAKE_SLOT_ID);
        m_intakePID.setOutputRange(k_IntakeGains.kMinOutput, k_IntakeGains.kMaxOutput, k_INTAKE_SLOT_ID);
        
    }

    public void speedControl(double speed) {
        //m_intakePID.setReference(speed, ControlType.kVelocity, k_INTAKE_SLOT_ID);
        m_leftIntake.set(speed);
    }

    public void stop() {
        //m_intakePID.setReference(0, ControlType.kVelocity, k_INTAKE_SLOT_ID);
        m_leftIntake.set(0);
    }
    
}

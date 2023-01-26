package frc.robot.subsystems;

import com.ctre.phoenix.sensors.WPI_Pigeon2;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.*;

public class DriveBase extends SubsystemBase {

  private PowerDistribution m_pdh;
  private PneumaticHub m_ph;

  private WPI_Pigeon2 m_pigeon;
  
  private CANSparkMax m_leftLeader, m_leftFollower, m_rightLeader, m_rightFollower;
  private SparkMaxPIDController m_leftPID, m_rightPID;
  private RelativeEncoder m_leftEncoder, m_rightEncoder;

  private DifferentialDrive m_diffDrive;

  public DriveBase() {

    m_pdh = new PowerDistribution(k_PDH, ModuleType.kRev);

    m_ph = new PneumaticHub(k_PH);

    m_pigeon = new WPI_Pigeon2(k_PIGEON);

    m_leftLeader = new CANSparkMax(k_LEFT_LEDER, MotorType.kBrushless);
    m_leftFollower = new CANSparkMax(k_LEFT_FOLLOWER, MotorType.kBrushless);
    m_rightLeader = new CANSparkMax(k_RIGHT_LEADER, MotorType.kBrushless);
    m_rightFollower = new CANSparkMax(k_RIGHT_FOLLOWER, MotorType.kBrushless);

    m_leftLeader.restoreFactoryDefaults();
    m_leftFollower.restoreFactoryDefaults();
    m_rightLeader.restoreFactoryDefaults();
    m_rightLeader.restoreFactoryDefaults();

    m_leftPID = m_leftLeader.getPIDController();
    m_rightPID = m_rightLeader.getPIDController();
    
    m_leftEncoder = m_leftLeader.getEncoder();
    m_rightEncoder = m_rightLeader.getEncoder();

    m_leftPID.setP(k_DriveGains.kP, k_DRIVE_SLOT_ID);
    m_leftPID.setI(k_DriveGains.kI, k_DRIVE_SLOT_ID);
    m_leftPID.setD(k_DriveGains.kD, k_DRIVE_SLOT_ID);
    m_leftPID.setIZone(k_DriveGains.kIzone, k_DRIVE_SLOT_ID);
    m_leftPID.setFF(k_DriveGains.kFF, k_DRIVE_SLOT_ID);
    m_leftPID.setOutputRange(k_DriveGains.kMinOutput, k_DriveGains.kMaxOutput, k_DRIVE_SLOT_ID);

    m_rightPID.setP(k_DriveGains.kP, k_DRIVE_SLOT_ID);
    m_rightPID.setI(k_DriveGains.kI, k_DRIVE_SLOT_ID);
    m_rightPID.setD(k_DriveGains.kD, k_DRIVE_SLOT_ID);
    m_rightPID.setIZone(k_DriveGains.kIzone, k_DRIVE_SLOT_ID);
    m_rightPID.setFF(k_DriveGains.kFF, k_DRIVE_SLOT_ID);
    m_rightPID.setOutputRange(k_DriveGains.kMinOutput, k_DriveGains.kMaxOutput, k_DRIVE_SLOT_ID);

    m_leftPID.setP(k_TurnGains.kP, k_TURN_SLOT_ID);
    m_leftPID.setI(k_TurnGains.kI, k_TURN_SLOT_ID);
    m_leftPID.setD(k_TurnGains.kD, k_TURN_SLOT_ID);
    m_leftPID.setIZone(k_TurnGains.kIzone, k_TURN_SLOT_ID);
    m_leftPID.setFF(k_TurnGains.kFF, k_TURN_SLOT_ID);
    m_leftPID.setOutputRange(k_TurnGains.kMinOutput, k_TurnGains.kMaxOutput, k_TURN_SLOT_ID);

    m_rightPID.setP(k_TurnGains.kP, k_TURN_SLOT_ID);
    m_rightPID.setI(k_TurnGains.kI, k_TURN_SLOT_ID);
    m_rightPID.setD(k_TurnGains.kD, k_TURN_SLOT_ID);
    m_rightPID.setIZone(k_TurnGains.kIzone, k_TURN_SLOT_ID);
    m_rightPID.setFF(k_TurnGains.kFF, k_TURN_SLOT_ID);
    m_rightPID.setOutputRange(k_TurnGains.kMinOutput, k_TurnGains.kMaxOutput, k_TURN_SLOT_ID);

    m_leftFollower.follow(m_leftLeader);
    m_rightFollower.follow(m_rightLeader);

    m_diffDrive = new DifferentialDrive(m_leftLeader, m_rightLeader);

  }

}

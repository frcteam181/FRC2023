package frc.robot.subsystems;

import com.ctre.phoenix.sensors.WPI_Pigeon2;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
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
  private Field2d m_field;
  private DifferentialDriveOdometry m_odometry;

  // Simulation

  public DriveBase() {

    m_pdh = new PowerDistribution(k_PDH, ModuleType.kRev);

    m_ph = new PneumaticHub(k_PH);

    m_ph.enableCompressorAnalog(55, 60);

    m_pigeon = new WPI_Pigeon2(k_PIGEON);

    m_leftLeader = new CANSparkMax(k_LEFT_LEADER, MotorType.kBrushless);
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

    m_leftLeader.setInverted(false);
    m_rightLeader.setInverted(true);

    m_diffDrive = new DifferentialDrive(m_leftLeader, m_rightLeader);

    m_field = new Field2d();

    m_odometry = new DifferentialDriveOdometry(getRotation2d(), getLeftPosition(), getRightPosition());

    // Simulation

  }

  @Override
  public void periodic() {
      updateOdometry();
      m_field.setRobotPose(getPoseMeters());
  }

  public Field2d getField() {
    return m_field;
  }

  public void updateOdometry() {
    m_odometry.update(getRotation2d(), getLeftPosition(), getRightPosition());
  }

  public Rotation2d getRotation2d() {
    return m_pigeon.getRotation2d();
  }

  public double getYaw() {
    return m_pigeon.getYaw();
  }

  public Pose2d getPoseMeters() {
    return m_odometry.getPoseMeters();
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

  public double deadband(double value) {

    if (Math.abs(value) >= k_DriverDeadband) {
      return value;
    } else {
      return 0;
    }

  }

  public void teleopDrive(double speedValue, double rotationValue) {
    m_diffDrive.arcadeDrive(deadband(speedValue), deadband(rotationValue));
  }

  public RelativeEncoder getLeftEncoder() {
    return m_leftEncoder;
  }

  public RelativeEncoder getRightEncoder() {
    return m_rightEncoder;
  }

  public double getLeftPosition() {
    return m_leftEncoder.getPosition();
  }


  public double getRightPosition() {
    return m_rightEncoder.getPosition();
  }

  public PowerDistribution getPDH() {
    return m_pdh;
  }

  public double getChannel7() {
    return m_pdh.getCurrent(k_PDH_CHANNEL7);
  }

  public double getChannel12() {
    return m_pdh.getCurrent(k_PDH_CHANNEL7);
  }

  //Beginning of code Aiden 2/2/2023

  /*
  public void stabilize() {
  
  if(getYaw() >= 5 || getYaw() <= -5) {
    m_diffDrive.tankDrive(getYaw(), getYaw());
  }

  }
*/
}

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.WPI_Pigeon2;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.ImplicitModelFollower;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.*;

public class DriveBase extends SubsystemBase {
  
  private CANSparkMax m_leftLeader, m_leftFollower, m_rightLeader, m_rightFollower;
  private RelativeEncoder m_leftEncoder, m_rightEncoder;
  private SparkMaxPIDController m_leftPID, m_rightPID;
  private DifferentialDrive m_diffDrive;
  private DifferentialDrivePoseEstimator m_odometry;
  private Field2d m_field;
  private PowerDistribution m_pdh;
  private PneumaticHub m_ph;
  private WPI_Pigeon2 m_pigeon;
  private boolean m_isTuning;
  private ShuffleboardTab m_tab;

  private ImplicitModelFollower m_implicitModelFollower;

  //private ChassisSpeeds m_chassisSpeed;
  //private DifferentialDriveWheelSpeeds m_wheelSpeed;
  private RamseteController m_RAMController;

  //#Simulation
  //#endregion

  public DriveBase() {

    m_leftLeader = new CANSparkMax(k_LEFT_LEADER, MotorType.kBrushless);
    m_leftFollower = new CANSparkMax(k_LEFT_FOLLOWER, MotorType.kBrushless);
    m_rightLeader = new CANSparkMax(k_RIGHT_LEADER, MotorType.kBrushless);
    m_rightFollower = new CANSparkMax(k_RIGHT_FOLLOWER, MotorType.kBrushless);

    m_leftLeader.restoreFactoryDefaults();
    m_leftFollower.restoreFactoryDefaults();
    m_rightLeader.restoreFactoryDefaults();
    m_rightLeader.restoreFactoryDefaults();

    m_leftFollower.follow(m_leftLeader);
    m_rightFollower.follow(m_rightLeader);

    m_leftLeader.setInverted(false);
    m_rightLeader.setInverted(true);

    m_diffDrive = new DifferentialDrive(m_leftLeader, m_rightLeader);

    m_leftEncoder = m_leftLeader.getEncoder();
    m_rightEncoder = m_rightLeader.getEncoder();

    m_leftPID = m_leftLeader.getPIDController();
    m_rightPID = m_rightLeader.getPIDController();

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

    m_pdh = new PowerDistribution(k_PDH, ModuleType.kRev);

    m_ph = new PneumaticHub(k_PH);

    m_ph.enableCompressorAnalog(100, 120);

    m_pigeon = new WPI_Pigeon2(k_PIGEON);

    m_field = new Field2d();

    m_odometry = new DifferentialDrivePoseEstimator(k_driveKinematics, getRotation2d(), 0, 0, new Pose2d(5, 10, getRotation2d()));

    m_RAMController = new RamseteController(kRamseteB, kRamseteZeta);
    /*
    m_chassisSpeed = m_RAMController.calculate(getPose(), null);
    m_wheelSpeed = k_driveKinematics.toWheelSpeeds(m_chassisSpeed);
    var left = m_wheelSpeed.leftMetersPerSecond;
    var right = m_wheelSpeed.rightMetersPerSecond;*/

    //m_implicitModelFollower = new ImplicitModelFollower<>(k_botPlant, k_refPlant);
    //m_implicitModelFollower.calculate(null, null);

    /* Tuning */
    m_isTuning = false;
    if(m_isTuning){tune();}
  }

  @Override
  public void periodic() {
      updateOdometry();
      m_field.setRobotPose(getPose());
      if(m_isTuning){tuningPeriodic();}
  }

  public void teleopDrive(double speedValue, double rotationValue) {
    m_diffDrive.arcadeDrive(deadband(speedValue), deadband(rotationValue));
  }

  public void followPath(double leftReferenceSpeed, double rightReferenceSpeed) {
    m_leftPID.setReference(leftReferenceSpeed, ControlType.kSmartVelocity, k_DRIVE_SLOT_ID);
    m_rightPID.setReference(rightReferenceSpeed, ControlType.kSmartVelocity, k_DRIVE_SLOT_ID);
  }

  public Field2d getField() {
    return m_field;
  }

  public Pose2d getPose() {
    return m_odometry.getEstimatedPosition();
  }

  public void updateOdometry() {
    m_odometry.update(getRotation2d(), getLeftPosition(), getRightPosition());
  }

  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    m_odometry.resetPosition(getRotation2d(), getLeftPosition(), getRightPosition(), pose);
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(getLeftSpeed(), getRightSpeed());
  }

  public SparkMaxPIDController getLeftPID() {
    return m_leftPID;
  }

  public SparkMaxPIDController getRightPID() {
    return m_rightPID;
  }

  public void resetHeading() {
    m_pigeon.reset();
  }

  public double getHeading() {
    return m_pigeon.getRotation2d().getDegrees();
  }

  public double getRoll() {
    return m_pigeon.getRoll();
  }

  public double getPitch(){
    return m_pigeon.getPitch();
  }

  public double getYaw() {
    return m_pigeon.getYaw();
  }

  public double getTurnRate() {
    return m_pigeon.getRate();
  }

  public Rotation2d getRotation2d() {
    return m_pigeon.getRotation2d();
  }

  public RelativeEncoder getLeftEncoder() {
    return m_leftEncoder;
  }

  public RelativeEncoder getRightEncoder() {
    return m_rightEncoder;
  }

  public void resetEncoders() {
    m_leftEncoder.setPosition(0);
    m_rightEncoder.setPosition(0);
  }

  public double getLeftPosition() {
    return m_leftEncoder.getPosition();
  }

  public double getRightPosition() {
    return m_rightEncoder.getPosition();
  }

  public double getLeftSpeed() {
    return m_leftEncoder.getVelocity();
  }

  public double getRightSpeed() {
    return m_rightEncoder.getVelocity();
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

  public double getChannel() {
    return m_pdh.getCurrent(12);
  }

  public RamseteController getRAMController() {
    return m_RAMController;
  }

  //#region Utilities



  
  public double deadband(double value) {
    if (Math.abs(value) >= k_DriverDeadband) {
      return value;
    } else {
      return 0;
    }
  }

  //#endregion Utilities

  //#region Tuning

  public void tune() {

    m_tab = Shuffleboard.getTab("Drive Base Tuning");

  }

  public void tuningPeriodic() {

  }

  //#endregion Tuning
}

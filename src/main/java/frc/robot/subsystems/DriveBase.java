package frc.robot.subsystems;

import com.ctre.phoenix.sensors.WPI_Pigeon2;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.DifferentialDriveFeedforward;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
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
  private WPI_Pigeon2 m_pigeon;
  private boolean m_isTuning;
  private ShuffleboardTab m_tab;
  private double[] m_leftResponse, m_rightResponse;
  private double m_driveFF, m_driveEffort, m_driveKp, m_driveKi, m_driveKd, m_driveFFKv, m_driveFFKa, m_angularFFKv, m_angularFFKa, m_leftSetpointVel, m_rightSetpointVel, m_driveOpenLoopRamp;
  private GenericEntry m_driveKpEntry, m_driveKiEntry, m_driveKdEntry, m_driveFFKvEntry, m_driveFFKaEntry, m_angularFFKvEntry, m_angularFFKaEntry, m_driveOpenLoopRampEntry, m_driveFFEntry;
  private RamseteController m_RAMController;
  private DifferentialDriveFeedforward m_diffDriveFF;

  // Trapezoid Profile
  private TrapezoidProfile.Constraints m_constraints;
  private double m_period;
  private TrapezoidProfile.State m_state, m_goal;
  private boolean m_enabled;

  //#Simulation
  //#endregion

  public DriveBase(boolean isTuning) {

    TrapezoidProfileSubsystem(new TrapezoidProfile.Constraints(1.5, 1), 0, 0.02);

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
    m_diffDrive.setDeadband(k_DriverDeadband);

    m_leftEncoder = m_leftLeader.getEncoder();
    m_leftEncoder.setPositionConversionFactor(k_drivePosFac);
    m_leftEncoder.setVelocityConversionFactor(k_driveVelFac);

    m_rightEncoder = m_rightLeader.getEncoder();
    m_rightEncoder.setPositionConversionFactor(k_drivePosFac);
    m_rightEncoder.setVelocityConversionFactor(k_driveVelFac);

    m_leftPID = m_leftLeader.getPIDController();
    m_rightPID = m_rightLeader.getPIDController();

    m_leftPID.setP(k_DriveGains.kP, k_DRIVE_SLOT_ID);
    m_leftPID.setI(k_DriveGains.kI, k_DRIVE_SLOT_ID);
    m_leftPID.setD(k_DriveGains.kD, k_DRIVE_SLOT_ID);
    m_leftPID.setIZone(k_DriveGains.kIzone, k_DRIVE_SLOT_ID);
    m_leftPID.setFF(k_DriveGains.kFF, k_DRIVE_SLOT_ID);
    m_leftPID.setOutputRange(k_DriveGains.kMinOutput, k_DriveGains.kMaxOutput, k_DRIVE_SLOT_ID);

    m_driveOpenLoopRamp = k_driveOpenLoopRamp;

    m_leftLeader.setOpenLoopRampRate(k_driveOpenLoopRamp);

    m_rightPID.setP(k_DriveGains.kP, k_DRIVE_SLOT_ID);
    m_rightPID.setI(k_DriveGains.kI, k_DRIVE_SLOT_ID);
    m_rightPID.setD(k_DriveGains.kD, k_DRIVE_SLOT_ID);
    m_rightPID.setIZone(k_DriveGains.kIzone, k_DRIVE_SLOT_ID);
    m_rightPID.setFF(k_DriveGains.kFF, k_DRIVE_SLOT_ID);
    m_rightPID.setOutputRange(k_DriveGains.kMinOutput, k_DriveGains.kMaxOutput, k_DRIVE_SLOT_ID);

    m_rightLeader.setOpenLoopRampRate(k_driveOpenLoopRamp);

    //m_diffDriveFF = new DifferentialDriveFeedforward(k_driveFFKv, k_driveFFKa, k_angularFFKv, k_angularFFKa, k_trackWidthMeters);

    m_pdh = new PowerDistribution(k_PDH, ModuleType.kRev);

    m_pigeon = new WPI_Pigeon2(k_PIGEON);

    m_field = new Field2d();

    m_odometry = new DifferentialDrivePoseEstimator(k_driveKinematics, getRotation2d(), getLeftPosition(), getRightPosition(), new Pose2d(0, 0, getRotation2d()));

    m_RAMController = k_RAMController;

    m_driveEffort = 0;//k_driveEffort;

    disable();
    enableBreakers();

    /* Tuning */
    m_isTuning = isTuning;
    if(m_isTuning){tune();}
  }

  @Override
  public void periodic() {

    updateOdometry();

    m_field.setRobotPose(getPose());
    m_diffDrive.feedWatchdog();

    if(m_isTuning){tuningPeriodic();}

    var profile = new TrapezoidProfile(m_constraints, m_goal, m_state);
    m_state = profile.calculate(m_period);
    if (m_enabled) {
        //System.out.println("About to use state");
        useState(m_state);
    }

  }

  public void enableBreakers() {
    m_leftLeader.setIdleMode(IdleMode.kBrake);
    m_leftFollower.setIdleMode(IdleMode.kBrake);
    m_rightLeader.setIdleMode(IdleMode.kBrake);
    m_rightFollower.setIdleMode(IdleMode.kBrake);
  }

  public void teleopDrive(double speedValue, double rotationValue) {
    m_diffDrive.arcadeDrive(speedValue * 0.75, rotationValue * 0.62, true);
  }

  public void teleopTankDrive(double leftSpeed, double rightSpeed) {
    m_diffDrive.tankDrive(-leftSpeed, -rightSpeed, true);
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

  public void resetPose(Pose2d pose) {
    resetEncoders();
    resetHeading();
    m_odometry.resetPosition(getRotation2d(), getLeftPosition(), getRightPosition(), pose);
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(getLeftSpeed(), getRightSpeed());
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

  public double getPitchRad() {
    return Units.degreesToRadians(getPitch());
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

  public double getLeftPositionIn() {
    return Units.metersToInches(m_leftEncoder.getPosition());
  }

  public double getRightPositionIn() {
    return Units.metersToInches(m_rightEncoder.getPosition());
  }

  public double getLeftSpeed() {
    return m_leftEncoder.getVelocity();
  }

  public double getRightSpeed() {
    return m_rightEncoder.getVelocity();
  }

  public double getLeftSetpointVel() {
    return m_leftSetpointVel;
  }

  public double getRightSetpointVel() {
    return m_rightSetpointVel;
  }

  public double[] getLeftVelResponse() {
    m_leftResponse[0] = getLeftSetpointVel();
    m_leftResponse[1] = getLeftSpeed();
    return m_leftResponse;
  }

  public double[] getRightVelResponse() {
    m_rightResponse[0] = getRightSetpointVel();
    m_rightResponse[1] = getRightSpeed();
    return m_rightResponse;
  }

  public double getChannel() {
    return m_pdh.getCurrent(12);
  }

  public RamseteController getRAMController() {
    return m_RAMController;
  }

  //#region Path Following

  public void followPath(double leftSetpointSpeed, double rightSetpointSpeed) {
    System.out.println("Follow Path Called");
    m_leftSetpointVel = leftSetpointSpeed;
    m_rightSetpointVel = rightSetpointSpeed;
    m_leftPID.setReference(leftSetpointSpeed, CANSparkMax.ControlType.kVelocity, k_DRIVE_SLOT_ID);
    m_rightPID.setReference(rightSetpointSpeed, CANSparkMax.ControlType.kVelocity, k_DRIVE_SLOT_ID);
  }

  public double getLeftCurrent() {
    return m_pdh.getCurrent(3);
  }

  public double getRightCurrent() {
    return m_pdh.getCurrent(2);
  }

  //#endregion Path Following

  //#region Tuning

  public void tune() {

    m_tab = Shuffleboard.getTab("Drive Base Tuning");

    // Velocity PID
    m_driveKp = k_DriveGains.kP;
    m_driveKi = k_DriveGains.kI;
    m_driveKd = k_DriveGains.kD;

    // Drive Train FF
    m_driveFFKv = k_driveFFKv;
    m_driveFFKa = k_driveFFKa;
    m_angularFFKv = k_angularFFKv;
    m_angularFFKa = k_angularFFKa;

    m_leftSetpointVel = 0;
    m_rightSetpointVel = 0;
    
    m_leftResponse = new double[2];
    m_rightResponse = new double[2];

    // Intake Current
    //Shuffleboard.getTab("Claw Tuning").addNumber("Left Current", this::getLeftCurrent).withPosition(4, 1);
    //Shuffleboard.getTab("Claw Tuning").addNumber("Right Current", this::getRightCurrent).withPosition(5, 1);

    m_driveKpEntry = m_tab.add("Drive Kp", m_driveKp).withPosition(0, 0).getEntry();
    m_driveKiEntry = m_tab.add("Drive Ki", m_driveKi).withPosition(0, 1).getEntry();
    m_driveKdEntry = m_tab.add("Drive Kd", m_driveKd).withPosition(0, 2).getEntry();
    m_driveFFEntry = m_tab.add("Drive FF", m_driveFF).withPosition(0, 3).getEntry();

    m_tab.addNumber("Heading", this::getHeading).withPosition(4, 1);
    m_tab.addNumber("Left Pos (meters)", this::getLeftPosition).withPosition(1, 0);
    m_tab.addNumber("Right Pos (meters)", this::getRightPosition).withPosition(7, 0);
    m_tab.addNumber("Left Pos (in)", this::getLeftPositionIn).withPosition(2, 0);
    m_tab.addNumber("Right Pos (in)", this::getRightPositionIn).withPosition(6, 0);

    m_driveFFKvEntry = m_tab.add("Drive Kv", m_driveFFKv).withPosition(8, 0).getEntry();
    m_driveFFKaEntry = m_tab.add("Drive Ka", m_driveFFKa).withPosition(8, 1).getEntry();
    m_angularFFKvEntry = m_tab.add("Angular Kv", m_angularFFKv).withPosition(8, 2).getEntry();
    m_angularFFKaEntry = m_tab.add("Angular Ka", m_angularFFKa).withPosition(8, 3).getEntry();
    m_driveOpenLoopRampEntry = m_tab.add("Open Loop Ramp", m_driveOpenLoopRamp).withPosition(4, 2).getEntry();

    m_tab.addDoubleArray("Left Vel Response", this::getLeftVelResponse).withPosition(1, 1).withSize(3, 3).withWidget(BuiltInWidgets.kGraph);
    m_tab.addDoubleArray("Right Vel Response", this::getRightVelResponse).withPosition(5, 1).withSize(3, 3).withWidget(BuiltInWidgets.kGraph);

  }

  public void tuningPeriodic() {

    // PID Gains
    var driveKp = m_driveKpEntry.getDouble(k_DriveGains.kP);
    var driveKi = m_driveKiEntry.getDouble(k_DriveGains.kI);
    var driveKd = m_driveKdEntry.getDouble(k_DriveGains.kD);
    var driveFF = m_driveFFEntry.getDouble(k_DriveGains.kFF);

    // Left PID
    if(driveKp != m_driveKp) {m_leftPID.setP(driveKp, k_DRIVE_SLOT_ID); m_rightPID.setP(driveKp, k_DRIVE_SLOT_ID); m_driveKp = driveKp;}
    if(driveKi != m_driveKi) {m_leftPID.setI(driveKi, k_DRIVE_SLOT_ID); m_rightPID.setI(driveKi, k_DRIVE_SLOT_ID); m_driveKi = driveKi;}
    if(driveKd != m_driveKd) {m_leftPID.setD(driveKd, k_DRIVE_SLOT_ID); m_rightPID.setD(driveKd, k_DRIVE_SLOT_ID); m_driveKd = driveKd;}
    if(driveFF != m_driveFF) {m_leftPID.setFF(driveFF, k_DRIVE_SLOT_ID); m_rightPID.setFF(driveFF, k_DRIVE_SLOT_ID); m_driveFF = driveFF;}

    var driveOpenLoopRamp = m_driveOpenLoopRampEntry.getDouble(k_driveOpenLoopRamp);
    if(driveOpenLoopRamp != m_driveOpenLoopRamp) {m_leftLeader.setOpenLoopRampRate(driveOpenLoopRamp); m_rightLeader.setOpenLoopRampRate(driveOpenLoopRamp); m_driveOpenLoopRamp = driveOpenLoopRamp;}
    
    // FF Gains
    var driveFFKv = m_driveFFKvEntry.getDouble(k_driveFFKv);
    var driveFFKa = m_driveFFKaEntry.getDouble(k_driveFFKa);
    var angularFFKv = m_angularFFKvEntry.getDouble(k_angularFFKv);
    var angularFFKa = m_angularFFKaEntry.getDouble(k_angularFFKa);
    
    // Feedforward
    if(driveFFKv != m_driveFFKv || driveFFKa != m_driveFFKa || angularFFKv != m_angularFFKv || angularFFKa != m_angularFFKa) {
      m_diffDriveFF = new DifferentialDriveFeedforward(driveFFKv, driveFFKa, angularFFKv, angularFFKa, k_refTrackWidthMeters);
      m_driveFFKv = driveFFKv;
      m_driveFFKa = driveFFKa;
      m_angularFFKv = angularFFKv;
      m_angularFFKa = angularFFKa;
    }

  }

  public boolean isTuning() {
    return m_isTuning;
  }

  //#endregion Tuning


  public void autoBalance() {

    if(Math.abs(getPitch()) < 1) {

      m_leftLeader.setVoltage(0);
      m_rightLeader.setVoltage(0);

    } else {

      var driveEffort = calcDriveEffort();

      m_leftLeader.setVoltage(driveEffort);
      m_rightLeader.setVoltage(driveEffort);

    }

  }

  public double calcDriveEffort() {
    return (Math.sin(getPitchRad())); //* m_maxDriveEffort);
  }

  public void driveForward(double speed) {
    m_leftLeader.set(speed);
    m_rightLeader.set(speed);
  }

  // Trapezoid Profile

  public void TrapezoidProfileSubsystem(TrapezoidProfile.Constraints constraints, double initialPosition, double period) {
    m_constraints = constraints;
    m_state = new TrapezoidProfile.State(initialPosition, 0);
    setGoal(initialPosition);
    m_period = period;
}

public void setGoal(TrapezoidProfile.State goal) {
  //System.out.println("Goal Updated");
  m_goal = goal;
}

public Command setDriveGoal(double goal) {
  //System.out.println("Drive Goal Set");
  return Commands.runOnce(() -> setGoal(goal), this);
}

public void setGoal(double goal) {
  setGoal(new TrapezoidProfile.State(goal, 0));
}

protected void useState(TrapezoidProfile.State setpoint) {
  m_leftSetpointVel = setpoint.velocity;
  m_rightSetpointVel = setpoint.velocity;
  m_leftPID.setReference(setpoint.velocity, CANSparkMax.ControlType.kVelocity, k_PIVOT_SLOT_ID);
  m_rightPID.setReference(setpoint.velocity, CANSparkMax.ControlType.kVelocity, k_PIVOT_SLOT_ID);
}

public void enable() {
  m_enabled = true;
}

public void disable() {
  m_enabled = false;
}

public void updateState(double currentState) {
  m_state = new TrapezoidProfile.State(currentState, 0);
}

}
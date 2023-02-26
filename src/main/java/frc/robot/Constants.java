package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.util.Units;

public final class Constants {

    /* Physical Robot Variables */

    public static final double k_wheelDiameterMeters = Units.inchesToMeters(6);
    public static final double k_trackWidthMeters = Units.inchesToMeters(22.65);
    public static final DifferentialDriveKinematics k_driveKinematics = new DifferentialDriveKinematics(k_trackWidthMeters);
    public static final double k_robotWeightKillograms = Units.lbsToKilograms(100);
    public static final double k_robotLengthMeters = Units.inchesToMeters(32.375);
    public static final double k_robotWidthMeters = Units.inchesToMeters(27.75);
    public static final double k_robotMomentOfInnertia = k_robotLengthMeters * k_robotWidthMeters * k_robotWeightKillograms * (1/6);
    public static final double k_gearRatio = 1;
    public static final double k_maxRPM = 5676;
    //public static final LinearSystem<N2,N2,N2> k_botPlant = LinearSystemId.createDrivetrainVelocitySystem(DCMotor.getNEO(2), k_robotWeightKillograms, k_wheelDiameterMeters/2, k_trackWidthMeters/2, k_robotMomentOfInnertia, k_gearRatio);

    /* Reference Robot Variables */

    public static final double k_refWheelDiameterMeters = Units.inchesToMeters(6);
    public static final double k_refTrackWidthMeters = 0.57531; // Provided by SysId
    public static final DifferentialDriveKinematics k_refDriveKinematics = new DifferentialDriveKinematics(k_trackWidthMeters);
    public static final double k_refRobotWeightKillograms = Units.lbsToKilograms(100);
    public static final double k_refRobotLengthMeters = Units.inchesToMeters(32.375);
    public static final double k_refRobotWidthMeters = Units.inchesToMeters(27.75);
    public static final double k_refRobotMomentOfInnertia = k_refRobotLengthMeters * k_refRobotWidthMeters * k_refRobotWeightKillograms * (1/6);
    public static final double k_refGearRatio = 1;
    public static final double k_refMaxRPM = 5676;
    //public static final LinearSystem<N2,N2,N2> k_refPlant = LinearSystemId.createDrivetrainVelocitySystem(DCMotor.getNEO(2), k_refRobotWeightKillograms, k_refWheelDiameterMeters/2, k_refTrackWidthMeters/2, k_refRobotMomentOfInnertia, k_refGearRatio);

    /* Drive Base Gains */

    public static final int k_DRIVE_SLOT_ID = 0;
    public static final int k_TURN_SLOT_ID = 1;
    public static final Gains k_DriveGains = new Gains(0, 0, 0, 0, 0, 0, 0);
    public static final Gains k_TurnGains = new Gains(0, 0, 0, 0, 0, 0, 0);

    // SysId variables for drivebase
    public static final double k_FFKs = 1.069; // Volts
    public static final double k_FFKv = 2.9807; // (Volts * Seconds) / Meter
    public static final double k_FFKa = 0.93873; // (Volts * Seconds^2) / Meter
    public static final double k_kPDriveVel = 3.1519;
    public static final double k_AngularKv = 0.93873;
    public static final double k_AngularKa = 0.93873;
    public static final double k_MaxSpeedMetersPerSecond = 3;
    public static final double k_MaxAccelerationMetersPerSecondSquared = 1;

    // Reasonable baseline values for a RAMSETE follower in units of meters and seconds
    public static final double kRamseteB = 2;
    public static final double kRamseteZeta = 0.7;

    /* Arm Gains */

    // Pivot
    public static final int k_PIVOT_SLOT_ID = 0;
    public static final double k_pivotKG_m_Gain = 0;
    public static final double k_pivotKG_b_Gain = 1;
    public static final double k_pivotOffset = Units.degreesToRadians(-62.6);
    public static final double k_maxPivotVel = 0.2;
    public static final double k_maxPivotAcc = 0.5;
    public static final double k_pivotPosFac = Units.degreesToRadians(4.5);
    public static final double k_pivotVelFac = 1;
    public static final double k_pivotKs = 0;
    public static final double k_pivotKg = 6;
    public static final double k_pivotKv = 0;
    public static final double k_pivotKa = 0;
    public static final Gains k_PivotGains = new Gains(0, 0, 0, 0, 0, -1, 1);

    // Spool
    public static final int k_SPOOL_SLOT_ID = 0;
    public static final double k_spoolPosFac = 1;
    public static final Gains k_SpoolGains = new Gains(0, 0, 0, 0, 0, -1, 1);

    // Claw

    // Beams

    /* Turret Gains */

    public static final int k_TURRET_SLOT_ID = 0;
    public static final Gains k_TurretGains = new Gains(0, 0, 0, 0, 0, -1, 1);
    public static final double k_maxTurretVel = 3;
    public static final double k_maxTurretAcc = 1.5;
    public static final double k_turretOffset = 0;
    public static final double k_turretPosFac = Units.degreesToRadians(0.77922077922);
    public static final double k_turretVelFac = Units.degreesToRadians(1);
    public static final double k_turretKs = 0;
    public static final double k_turretKv = 0;
    public static final double k_turretKa = 0;

    /* Intake Gains */

    public static final int k_INTAKE_SLOT_ID = 0;
    public static final Gains k_IntakeGains = new Gains(0.11, 0, 0, 0, 0, 0, 0);

    /* Controller Ports */

    public static final int k_DRIVER_CONTROLLER = 0;
    public static final int k_OERATOR_CONTROLLER = 1;

    /* Controller Deadbands */

    public static final double k_DriverDeadband = 0.2;
    public static final double k_OperatorDeadband = 0.2;

    /* Camera Pipelines */

    public static final int k_REFLECTIVE_PIPELINE = 0;
    public static final int k_APRIL_TAG_PIPELINE = 1;

    // Do not change anything after this line unless you rewire the robot and
    // update the spreadsheet!
    // Port assignments should match up with the spreadsheet here:
    // https://docs.google.com/spreadsheets/d/1yODdikPXsuiSYJ6ICP2WU1t-p1ZJTm6qR6IG0DP6S7M/edit#gid=0

    /* CAN IDs */

    public static final int k_PDH = 1;
    public static final int k_PH = 2;
    public static final int k_LEFT_FOLLOWER = 3;
    public static final int k_LEFT_LEADER = 4;
    public static final int k_RIGHT_FOLLOWER = 5;
    public static final int k_RIGHT_LEADER = 6;
    public static final int k_PIGEON = 7;
    public static final int k_RIGHT_PIVOT = 8;
    public static final int k_LEFT_PIVOT = 9;
    public static final int k_SPOOL = 10;
    public static final int k_TALON_PIVOT = 11;
    public static final int k_TURRET = 12;
    public static final int k_LEFT_INTAKE = 13;
    public static final int k_RIGHT_INTAKE = 14;

    /* Digital Input Output IDs */

    public static final int k_PIVOT_HOME_BEAM_ID = 0;
    public static final int k_PIVOT_ZERO_BEAM_ID = 1;
    public static final int k_SPOOL_HOME_BEAM_ID = 2;
    public static final int k_SPOOL_MAX_BEAM_ID = 3;

    /* Pneumatics Ports */

    public static final int k_STORED_PSI = 0;
    public static final int k_WORKING_PSI = 1;
    //public static final int k_INTAKE_PISTON = 0; //Not used may delete soon
    public static final int k_CLAW_GRAB = 0;
    public static final int k_CLAW_RELEASE = 1;

    // PDH Ports

}
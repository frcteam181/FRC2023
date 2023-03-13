package frc.robot;

import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;

public final class Constants {

    /* Physical Robot Variables */

    public static final double k_wheelDiameterMeters = Units.inchesToMeters(6.0);
    public static final double k_trackWidthMeters = Units.inchesToMeters(22.65); //22.65
    public static final DifferentialDriveKinematics k_driveKinematics = new DifferentialDriveKinematics(k_trackWidthMeters);
    public static final double k_robotWeightKillograms = Units.lbsToKilograms(100);
    public static final double k_robotLengthMeters = Units.inchesToMeters(32.375);
    public static final double k_robotWidthMeters = Units.inchesToMeters(27.75);
    public static final double k_robotMomentOfInnertia = k_robotLengthMeters * k_robotWidthMeters * k_robotWeightKillograms * (1.0/6.0);
    public static final double k_gearRatio = 8.45;
    public static final double k_maxRPM = 5676.0;
    public static final double k_drivePosFac = (Math.PI * k_wheelDiameterMeters * (1.0/k_gearRatio)); // rev -> meter
    public static final double k_driveVelFac = (Math.PI * k_wheelDiameterMeters * (1.0/(60 * k_gearRatio))); // rev/min -> m/s

    /* Reference Robot Variables */

    public static final double k_refWheelDiameterMeters = Units.inchesToMeters(6.0);
    // Trackwidth Provided by SysId
    public static final DifferentialDriveKinematics k_refDriveKinematics = new DifferentialDriveKinematics(k_trackWidthMeters);
    public static final double k_refRobotWeightKillograms = Units.lbsToKilograms(100.0);
    public static final double k_refRobotLengthMeters = Units.inchesToMeters(32.375);
    public static final double k_refRobotWidthMeters = Units.inchesToMeters(27.75);
    public static final double k_refRobotMomentOfInnertia = k_refRobotLengthMeters * k_refRobotWidthMeters * k_refRobotWeightKillograms * (1/6);
    public static final double k_refGearRatio = 1.0;
    public static final double k_refMaxRPM = 5676.0;

    /* Drive Base Gains */

    public static final int k_DRIVE_SLOT_ID = 0;
    public static final int k_TURN_SLOT_ID = 1;
    public static final double k_driveOpenLoopRamp = 0.85;

    // SysId variables for drivebase
    public static final Gains k_DriveGains = new Gains(0.5, 0, 0, 0, 0, -1, 1);
    public static final double k_driveFFKs = 0; // (Volts * Seconds) / Meter
    public static final double k_driveFFKv = 0;
    public static final double k_driveFFKa = 0; // (Volts * Seconds^2) / Meter
    public static final double k_angularFFKv = 0;
    public static final double k_angularFFKa = 0;
    public static final double k_MaxSpeedMetersPerSecond = 1.5;
    public static final double k_MaxAccelerationMetersPerSecondSquared = 0.8;
    public static final double k_refTrackWidthMeters = 0;
    //public static final LinearSystem<N2,N2,N2> k_refPlant = LinearSystemId.identifyDrivetrainSystem(k_driveFFKv, k_driveFFKa, k_angularFFKv, k_angularFFKa, k_refTrackWidthMeters);

    // Reasonable baseline values for a RAMSETE follower in units of meters and seconds // 2, 0.7
    public static final RamseteController k_RAMController = new RamseteController(2, 0.7);

    /* Arm Gains */

    // Pivot
    public static final int k_PIVOT_SLOT_ID = 0;
    public static final double k_pivotKG_m_Gain = 0;
    public static final double k_pivotKG_b_Gain = 0.5;
    public static final double k_maxPivotVel = 1.2;
    public static final double k_maxPivotAcc = 1;
    public static final double k_pivotGearRatio = 1.0;
    public static final double k_pivotPosFacRad = Units.degreesToRadians(360/k_pivotGearRatio);
    public static final double k_pivotVelFacRadPerSec = k_pivotPosFacRad/60.0;
    public static final Gains k_PivotGains = new Gains(0, 0, 0, 0, 0, -1, 1);

    // Spool
    public static final int k_SPOOL_SLOT_ID = 0;
    public static final double k_spoolGearRatio = 80.0;
    public static final double k_spoolDiameterMeter = Units.inchesToMeters(1);
    public static final double k_spoolPosFacMeter = (k_spoolDiameterMeter * Math.PI)/k_spoolGearRatio; //meter per rev
    public static final double k_spoolVelFacMeterPerSec = k_spoolPosFacMeter/60.0; // m/s
    public static final double k_maxSpoolExtentionMeter = Units.inchesToMeters(30);
    public static final Gains k_SpoolGains = new Gains(15, 0, 0, 0, 0, -1, 1);

    // Claw
    public static final int k_CLAW_SLOT_ID = 0;
    public static final double k_clawWheelDiameterMeters = Units.inchesToMeters(4.0);
    public static final double k_clawGearRatio = 4.0;
    public static final double k_clawPosFacMeter = (k_clawWheelDiameterMeters * Math.PI)/k_clawGearRatio;
    public static final double k_clawVelFacMeterPerSec = k_clawPosFacMeter/60.0;
    public static final double k_maxClawVelMeter = 4.0; // m/s
    public static final Gains k_ClawGains = new Gains(0, 0, 0, 0, 0, -1, 1);
    

    // Pneumatic Values
    public static final int k_minPressure = 100;
    public static final int k_maxPressure = 120;

    /* Turret Gains */

    public static final int k_TURRET_SLOT_ID = 0;
    public static final Gains k_TurretGains = new Gains(1, 0, 0, 0, 0, -1, 1);
    public static final double k_maxTurretVel = 3;
    public static final double k_maxTurretAcc = 1.5;
    public static final double k_turretOffset = 0;
    public static final double k_turretPosFacRad = Units.degreesToRadians(0.77922077922);
    public static final double k_turretVelFacRadPerSec = k_turretPosFacRad/60.0;
    public static final double k_turretKs = 2;
    public static final double k_turretKv = 0;
    public static final double k_turretKa = 0;

    /* Intake Gains */

    public static final int k_INTAKE_SLOT_ID = 0;
    public static final double k_intakeSpeed = 2;
    public static final Gains k_IntakeGains = new Gains(0, 0, 0, 0, 0, -1, 1);

    /* Controller Ports */

    public static final int k_DRIVER_CONTROLLER = 0;
    public static final int k_OERATOR_CONTROLLER = 1;
    public static final int k_DRIVER_TEST_CONTROLLER = 2;
    public static final int k_OPERATOR_TEST_CONTROLLER = 3;

    /* Controller Deadbands */

    public static final double k_DriverDeadband = 0.2;
    public static final double k_OperatorDeadband = 0.2;

    /* Camera Pipelines */

    public static final int k_REFLECTIVE_PIPELINE = 0;
    public static final int k_APRIL_TAG_PIPELINE = 1;

    /* Through bore encoder */

    public static final int k_throughBoreEncoderCPR = 8192;

    /* Presets */ /*-----------------------------------------------------------------------------*/ /* Presets */

    // Angle Presets
    // public static final double k_addedAngleDeg = 71;
    // public static final double k_addedAngleRad = Units.degreesToRadians(k_addedAngleDeg);
    // public static final double k_pivotHomeRad = Units.degreesToRadians(-66 + k_addedAngleDeg);
    // public static final double k_maxPivotAngleRad = Units.degreesToRadians(20 + k_addedAngleDeg);
    // public static final double k_pivotHighScoreRad = Units.degreesToRadians(11 + k_addedAngleDeg);
    // public static final double k_pivotHorizontalRad = Units.degreesToRadians(0 + k_addedAngleDeg);
    // public static final double k_pivotSafetyAngleRad = Units.degreesToRadians(-45 + k_addedAngleDeg);
    // public static final double k_pivotGroudIntake = Units.degreesToRadians(-45 + k_addedAngleDeg);

    public static final double k_addedAngleDeg = 71;
    public static final double k_addedAngleRad = Units.degreesToRadians(k_addedAngleDeg);
    public static final double k_pivotHomeRad = Units.degreesToRadians(122);
    public static final double k_maxPivotAngleRad = Units.degreesToRadians(210);
    public static final double k_pivotHighScoreRad = Units.degreesToRadians(202);
    public static final double k_pivotHorizontalRad = Units.degreesToRadians(188);
    public static final double k_pivotSafetyAngleRad = Units.degreesToRadians(150);
    public static final double k_pivotGroudIntake = Units.degreesToRadians(140);
    public static final double k_pivotOffsetRad = Units.degreesToRadians(122); // Offs0et to ZERO 0 deg a.k.a. parallel to floor //66
    //public static final double k_pivotThroughboreOffsetRad = Units.degreesToRadians(122 + 66);

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
    public static final int k_TURRET = 12;
    public static final int k_RIGHT_CLAW = 13;
    public static final int k_LEFT_CLAW = 14;

    /* Digital Input Output IDs */

    public static final int k_PIVOT_HOME_BEAM_ID = 0;
    public static final int k_PIVOT_ZERO_BEAM_ID = 1;
    public static final int k_SPOOL_HOME_BEAM_ID = 2;
    public static final int k_SPOOL_MAX_BEAM_ID = 3;

    /* Pneumatics Ports */

    // Analog Sensor Port
    public static final int k_STORED_PSI = 0;
    public static final int k_WORKING_PSI = 1;

    // Claw Channels
    public static final int k_CLAW_CLOSE = 0;
    public static final int k_CLAW_OPEN = 1;

}
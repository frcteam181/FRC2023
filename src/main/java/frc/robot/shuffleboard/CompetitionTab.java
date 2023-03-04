package frc.robot.shuffleboard;

import java.util.List;
import java.util.Map;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveKinematicsConstraint;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.commands.auto_commands.doNothing;
import frc.robot.subsystems.DriveBase;

import static frc.robot.Constants.*;

public class CompetitionTab {

    private ShuffleboardTab m_competitionTab;
    private SendableChooser<Command> m_autoChooser;

    private DriveBase m_driveBase;
    
    public CompetitionTab(DriveBase driveBase) {

        ShuffleboardTab m_competitionTab = Shuffleboard.getTab("Competition");

        //Initialize used objects
        m_autoChooser = new SendableChooser<Command>();

        //Used Subsystems
        m_driveBase = driveBase;

        /* Set Default Auto */
        m_autoChooser.setDefaultOption("Do Nothing", new doNothing());
        // Add auto options here:
        m_autoChooser.addOption("Do Nothing", new doNothing());
        m_competitionTab.add("Choose Auto", m_autoChooser).withWidget(BuiltInWidgets.kComboBoxChooser).withPosition(4, 4).withSize(3, 1);

        /* (Column , Row) starting from index #0  15x7 */
        m_competitionTab.add("Field", m_driveBase.getField()).withPosition(0, 0).withSize(7, 4);
        m_competitionTab.addDouble("Stored Pressure", m_driveBase::getStoredPSI).withWidget(BuiltInWidgets.kDial).withProperties(Map.of("min", 0, "max", 120)).withPosition(9, 4).withSize(2, 2);
        m_competitionTab.addDouble("Working Pressure", m_driveBase::getWorkingPSI).withWidget(BuiltInWidgets.kDial).withProperties(Map.of("min", 0, "max", 60)).withPosition(11, 4).withSize(2, 2);
        m_competitionTab.addBoolean("Compressor Satus", m_driveBase::getCompressorStatus).withPosition(4, 5).withSize(3, 1);

    }

    public Command getAuto() {
        return m_autoChooser.getSelected();
    }

    public Command getAdvancedAuto() {

        // Create config for trajectory
        TrajectoryConfig config = new TrajectoryConfig(k_MaxSpeedMetersPerSecond, k_MaxAccelerationMetersPerSecondSquared).setKinematics(k_driveKinematics).addConstraint(new DifferentialDriveKinematicsConstraint(k_driveKinematics, 2));

        // An example trajectory to follow.  All units in meters.
        Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(new Pose2d(0, 0, new Rotation2d(0)), List.of(new Translation2d(1, 1), new Translation2d(2, -1)), new Pose2d(3, 0, new Rotation2d(0)),config);

        RamseteCommand ramseteCommand = new RamseteCommand(exampleTrajectory, m_driveBase::getPose, m_driveBase.getRAMController(), k_driveKinematics, m_driveBase::followPath, m_driveBase);

        // Reset odometry to the starting pose of the trajectory.
        m_driveBase.resetPose(exampleTrajectory.getInitialPose());

        // Run path following command, then stop at the end.
        return ramseteCommand.andThen(() -> m_driveBase.teleopDrive(0, 0));
    }

}
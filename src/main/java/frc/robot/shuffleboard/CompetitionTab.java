package frc.robot.shuffleboard;

import java.util.HashMap;
import java.util.List;
import java.util.Map;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.RamseteAutoBuilder;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.DriveBase;

import static frc.robot.Constants.*;

public class CompetitionTab {

    private SendableChooser<String> m_pathChooser;
    private RamseteAutoBuilder m_autoBuilder;
    private List<PathPlannerTrajectory> pathGroup;
    private HashMap<String, Command> m_eventMap;

    private DriveBase m_driveBase;
    private Claw m_claw;
    
    public CompetitionTab(DriveBase driveBase, Claw claw, HashMap<String, Command> eventMap) {

        ShuffleboardTab m_competitionTab = Shuffleboard.getTab("Competition");

        //Initialize used objects
        m_pathChooser = new SendableChooser<String>();
        m_eventMap = eventMap;

        //Used Subsystems
        m_driveBase = driveBase;
        m_claw = claw;

        /* (Column , Row) starting from index #0  15x7 */
        m_competitionTab.add("Field", m_driveBase.getField()).withPosition(0, 0).withSize(7, 4);
        m_competitionTab.addDouble("Stored Pressure", m_claw::getStoredPSI).withWidget(BuiltInWidgets.kDial).withProperties(Map.of("min", 0, "max", 120)).withPosition(9, 4).withSize(2, 2);
        m_competitionTab.addDouble("Working Pressure", m_claw::getWorkingPSI).withWidget(BuiltInWidgets.kDial).withProperties(Map.of("min", 0, "max", 60)).withPosition(11, 4).withSize(2, 2);
        m_competitionTab.addBoolean("Compressor Satus", m_claw::getCompressorStatus).withPosition(4, 5).withSize(3, 1);

        /* Path Planner */

        m_pathChooser.setDefaultOption("Taxi Reversed", "Taxi Reversed");
        m_pathChooser.addOption("Taxi", "Taxi");
        m_pathChooser.addOption("Taxi Reversed", "Taxi Reversed");
        m_pathChooser.addOption("Taxi Turn Reversed", "Taxi Turn Reversed");
        m_pathChooser.addOption("Cone Taxi", "Cone Taxi");
        m_pathChooser.addOption("Loading Taxi", "Loading Taxi");

        // Remove this once tuning is finished
        Shuffleboard.getTab("Drive Base Tuning").add("Choose Path", m_pathChooser).withWidget(BuiltInWidgets.kComboBoxChooser).withPosition(3, 0).withSize(3, 1);
        //m_competitionTab.add("Choose Path", m_pathChooser).withWidget(BuiltInWidgets.kComboBoxChooser).withPosition(3, 0).withSize(3, 1);

        m_autoBuilder = new RamseteAutoBuilder(m_driveBase::getPose, m_driveBase::resetPose, k_RAMController, k_driveKinematics, m_driveBase::followPath, m_eventMap, true, m_driveBase);

    }

    public Command getPlannedAuto() {
        pathGroup = PathPlanner.loadPathGroup(m_pathChooser.getSelected(), false, new PathConstraints(2, 1), new PathConstraints(2, 1), new PathConstraints(2, 1), new PathConstraints(2, 1));
        return m_autoBuilder.fullAuto(pathGroup);
    }

}
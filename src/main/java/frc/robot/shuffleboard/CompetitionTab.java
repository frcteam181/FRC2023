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
import frc.robot.commands.autoDefault.doNothingAuto;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.DriveBase;

import static frc.robot.Constants.*;

public class CompetitionTab {

    private SendableChooser<String> m_pathChooser;
    private SendableChooser<Command> m_autoChooser;
    private RamseteAutoBuilder m_autoBuilder;
    private List<PathPlannerTrajectory> pathGroup;
    private HashMap<String, Command> m_eventMap;

    private DriveBase m_driveBase;
    private Claw m_claw;
    
    public CompetitionTab(DriveBase driveBase, Claw claw, HashMap<String, Command> eventMap) {

        ShuffleboardTab m_competitionTab = Shuffleboard.getTab("Competition");

        //Initialize used objects
        m_autoChooser = new SendableChooser<Command>();
        m_pathChooser = new SendableChooser<String>();
        m_eventMap = eventMap;

        //Used Subsystems
        m_driveBase = driveBase;
        m_claw = claw;

        /* (Column , Row) starting from index #0  15x7 */
        m_competitionTab.add("Field", m_driveBase.getField()).withPosition(0, 0).withSize(7, 4);
        m_competitionTab.addDouble("Stored Pressure", m_claw::getStoredPSI).withProperties(Map.of("min", 0, "max", 120)).withPosition(7, 0).withSize(2, 1);
        m_competitionTab.addDouble("Working Pressure", m_claw::getWorkingPSI).withProperties(Map.of("min", 0, "max", 60)).withPosition(7, 1).withSize(2, 1);
        m_competitionTab.addBoolean("Compressor Satus", m_claw::getCompressorStatus).withPosition(7, 2).withSize(1, 1);
        m_competitionTab.addBoolean("Has Game Piece", m_claw::hasPiece).withPosition(8, 2).withSize(1, 1);

        /* Path Planner */

        m_pathChooser.setDefaultOption("Taxi Reversed", "Taxi Reversed");
        m_pathChooser.addOption("Taxi", "Taxi");
        m_pathChooser.addOption("Taxi Reversed", "Taxi Reversed");
        m_pathChooser.addOption("Taxi Turn Reversed", "Taxi Turn Reversed");
        m_pathChooser.addOption("Cone Taxi", "Cone Taxi");
        m_pathChooser.addOption("Loading Taxi", "Loading Taxi");

        m_autoChooser.addOption("Do Nothing", new doNothingAuto(m_driveBase));
        m_autoChooser.setDefaultOption("Do Nothing", new doNothingAuto(driveBase));
        m_competitionTab.add("Auto", m_autoChooser).withWidget(BuiltInWidgets.kComboBoxChooser).withPosition(7, 3).withSize(3, 1);
       
        if(m_driveBase.isTuning()) {
            //Shuffleboard.getTab("Drive Base Tuning").add("Choose Path", m_pathChooser).withWidget(BuiltInWidgets.kComboBoxChooser).withPosition(3, 0).withSize(3, 1);
        } else {
            //m_competitionTab.add("Choose Path", m_pathChooser).withWidget(BuiltInWidgets.kComboBoxChooser).withPosition(7, 3).withSize(2, 1);
        }

        m_autoBuilder = new RamseteAutoBuilder(m_driveBase::getPose, m_driveBase::resetPose, k_RAMController, k_driveKinematics, m_driveBase::followPath, m_eventMap, true, m_driveBase);

    }

    public Command getPlannedAuto() {
        pathGroup = PathPlanner.loadPathGroup(m_pathChooser.getSelected(), false, new PathConstraints(2, 1), new PathConstraints(2, 1), new PathConstraints(2, 1), new PathConstraints(2, 1));
        return m_autoBuilder.fullAuto(pathGroup);
    }

    public Command getAuto() {
        return m_autoChooser.getSelected();
    }

}
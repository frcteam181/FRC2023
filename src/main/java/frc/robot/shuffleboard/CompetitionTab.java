package frc.robot.shuffleboard;

import java.util.HashMap;
import java.util.List;
import java.util.Map;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.RamseteAutoBuilder;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.autos.doNothing;
import frc.robot.commands.autos.midScore;
import frc.robot.commands.autos.scoreCubeTaxi;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.DriveBase;
import frc.robot.subsystems.Spool;

import static frc.robot.Constants.*;

public class CompetitionTab {

    private SendableChooser<String> m_pathChooser;
    private SendableChooser<Command> m_autoChooser;
    private RamseteAutoBuilder m_autoBuilder;
    private List<PathPlannerTrajectory> m_pathGroup;
    private PathPlannerTrajectory m_path;
    private HashMap<String, Command> m_eventMap;
    private UsbCamera m_usbCamera;
    private boolean m_isPlannedAuto;

    private DriveBase m_driveBase;
    private Claw m_claw;
    private Arm m_arm;
    private Spool m_spool;
    
    public CompetitionTab(DriveBase driveBase, Arm arm, Spool spool, Claw claw, HashMap<String, Command> eventMap, boolean isPlannedAuto) {

        ShuffleboardTab m_competitionTab = Shuffleboard.getTab("Competition");

        //Used Subsystems
        m_driveBase = driveBase;
        m_claw = claw;
        m_spool = spool;
        m_arm = arm;

        //Initialize used objects
        m_isPlannedAuto = isPlannedAuto;
        
        if(m_isPlannedAuto) {

            m_pathChooser = new SendableChooser<String>();

            /* Path Planner */
            // m_pathChooser.setDefaultOption("Taxi Reversed", "Taxi Reversed");
            // m_pathChooser.addOption("Taxi", "Taxi");
            // m_pathChooser.addOption("Taxi Reversed", "Taxi Reversed");
            // m_pathChooser.addOption("Taxi Turn Reversed", "Taxi Turn Reversed");
            // m_pathChooser.addOption("Cone Taxi", "Cone Taxi");
            // m_pathChooser.addOption("Loading Taxi", "Loading Taxi");
            m_pathChooser.addOption("Taxi Red", "R_TaxiLong");
            m_pathChooser.addOption("Taxi Blue", "B_TaxiLong");
            m_pathChooser.addOption("Spit Cube Only Blue", "B_SpitCube");
            m_pathChooser.addOption("Spit Cube Only Red", "R_SpitCube");

        } else {

            m_autoChooser = new SendableChooser<Command>();

            m_autoChooser.setDefaultOption("Do Nothing", new doNothing(m_driveBase));
            m_autoChooser.addOption("Do Nothing", new doNothing(m_driveBase));        
            m_autoChooser.addOption("Cube and Taxi", new scoreCubeTaxi(m_driveBase));
            m_autoChooser.addOption("Mid Cone", new midScore(m_arm, m_spool, m_claw));

        }

        /* (Column , Row) starting from index #0  15x7 */
        //m_competitionTab.add("Field", m_driveBase.getField()).withPosition(0, 0).withSize(7, 4);
        m_competitionTab.addDouble("Stored Pressure", m_claw::getStoredPSI).withProperties(Map.of("min", 0, "max", 120)).withPosition(7, 0).withSize(2, 1);
        m_competitionTab.addDouble("Working Pressure", m_claw::getWorkingPSI).withProperties(Map.of("min", 0, "max", 60)).withPosition(7, 1).withSize(2, 1);
        m_competitionTab.addBoolean("Compressor Satus", m_claw::getCompressorStatus).withPosition(7, 2).withSize(1, 1);
        m_competitionTab.addBoolean("Has Game Piece", m_claw::hasPiece).withPosition(8, 2).withSize(1, 1);
        
        m_usbCamera = CameraServer.startAutomaticCapture();
        m_competitionTab.add(m_usbCamera).withPosition(0, 0).withSize(7, 4);
       
        if(m_driveBase.isTuning()) {
            if(m_isPlannedAuto) {
                Shuffleboard.getTab("Drive Base Tuning").add("Choose Path", m_pathChooser).withWidget(BuiltInWidgets.kComboBoxChooser).withPosition(3, 0).withSize(3, 1);
            } else {
                Shuffleboard.getTab("Drive Base Tuning").add("Auto", m_autoChooser).withWidget(BuiltInWidgets.kComboBoxChooser).withPosition(3, 0).withSize(3, 1);
            }
        } else {
            if(m_isPlannedAuto) {
                m_competitionTab.add("Choose Path", m_pathChooser).withWidget(BuiltInWidgets.kComboBoxChooser).withPosition(7, 3).withSize(2, 1);
            } else {
                m_competitionTab.add("Auto", m_autoChooser).withWidget(BuiltInWidgets.kComboBoxChooser).withPosition(7, 3).withSize(2, 1);
            }
        }

        m_eventMap = eventMap;
        m_autoBuilder = new RamseteAutoBuilder(m_driveBase::getPose, m_driveBase::resetPose, k_RAMController, k_driveKinematics, m_driveBase::followPath, m_eventMap, true, m_driveBase);

    }

    public Command getPlannedAuto() {
        //m_pathGroup = PathPlanner.loadPathGroup(m_pathChooser.getSelected(), true, new PathConstraints(2, 1));
        m_path = PathPlanner.loadPath(m_pathChooser.getSelected(), new PathConstraints(2, 0.5), false);
        //return m_autoBuilder.fullAuto(m_pathGroup);
        return m_autoBuilder.followPathWithEvents(m_path);
    }

    public Command getAuto() {
        return m_autoChooser.getSelected();
    }

}
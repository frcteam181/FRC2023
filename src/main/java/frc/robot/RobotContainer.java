package frc.robot;

import com.pathplanner.lib.server.PathPlannerServer;

import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.controllers.DriverController;
import frc.robot.controllers.OperatorController;
import frc.robot.shuffleboard.CompetitionTab;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.DriveBase;
import frc.robot.subsystems.Spool;

import static frc.robot.Constants.*;

import java.util.HashMap;

public class RobotContainer {

  //Subsystems
  private DriveBase m_driveBase;
  private Arm m_arm;
  private Spool m_spool;
  private Claw m_claw;

  //Controllers
  private DriverController m_driverController;
  private OperatorController m_operatorController;
  
  //Shuffleboard Tabs
  private final CompetitionTab m_competitionTab;

  //Path Planning
  private boolean m_isTuning, m_isPlannedAuto;

  private HashMap<String, Command> m_eventMap;

  private UsbCamera m_usbCam;

  public RobotContainer() {

    // Path Planning Tuning
    m_isTuning = false;
    if(m_isTuning) {PathPlannerServer.startServer(5811);}
    m_eventMap = new HashMap<String, Command>();

    // Auto Type
    m_isPlannedAuto = false;

    //Subsystems
    m_driveBase = new DriveBase(false, false);
    m_arm = new Arm(false);
    m_spool = new Spool(false);
    m_claw = new Claw(false);

    //Controllers
    m_driverController = new DriverController(m_driveBase, m_claw, false);
    m_operatorController = new OperatorController(m_arm, m_spool, m_claw);

    //Shuffleboard Tabs
    m_competitionTab = new CompetitionTab(m_driveBase, m_arm, m_spool, m_claw, getEventMap(), m_isPlannedAuto);
  
  }

  public Command getAutonomousCommand() {
    if(m_isPlannedAuto) {
      return m_competitionTab.getPlannedAuto();
    } else {
      return m_competitionTab.getAuto();
    }
  } 

  public HashMap<String, Command> getEventMap() {
    m_eventMap.put("spitCube", m_claw.spitCubeCommand());
    m_eventMap.put("stopIntake", m_claw.stopIntakeCommand());
    return m_eventMap;
  }
  
}
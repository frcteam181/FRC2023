package frc.robot;

import com.pathplanner.lib.server.PathPlannerServer;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.controllers.DriverController;
import frc.robot.controllers.OperatorController;
import frc.robot.shuffleboard.CompetitionTab;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.DriveBase;

import static frc.robot.Constants.*;

import java.util.HashMap;

public class RobotContainer {

  //Subsystems
  private DriveBase m_driveBase;
  private Claw m_claw;

  //Controllers
  private DriverController m_driverController;
  private OperatorController m_operatorController;
  
  //Shuffleboard Tabs
  private final CompetitionTab m_competitionTab;

  //Path Planning
  private boolean m_isTuning;

  private HashMap<String, Command> m_eventMap;

  private UsbCamera m_usbCam;

  public RobotContainer() {

    //m_usbCam = CameraServer.startAutomaticCapture();

    // Path Planning Tuning
    m_isTuning = false;
    if(m_isTuning) {PathPlannerServer.startServer(5811);}
    m_eventMap = new HashMap<String, Command>();

    //Subsystems
    m_driveBase = new DriveBase(false);
    m_claw = new Claw(false);

    //Controllers
    m_driverController = new DriverController(m_driveBase);
    m_operatorController = new OperatorController(m_claw);

    //Shuffleboard Tabs
    m_competitionTab = new CompetitionTab(m_driveBase, m_claw, getEventMap());
  
  }

  public Command getAutonomousCommand() {
    //return m_competitionTab.getAuto();
    return m_competitionTab.getPlannedAuto();
  } 

  public HashMap<String, Command> getEventMap() {
    m_eventMap.put("spitCube", m_claw.spitCubeCommand());
    m_eventMap.put("stopIntake", m_claw.stopIntakeCommand());
    return m_eventMap;
  }
  
}
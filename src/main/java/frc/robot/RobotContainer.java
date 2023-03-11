package frc.robot;

import com.pathplanner.lib.server.PathPlannerServer;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.controllers.DriverController;
import frc.robot.controllers.DriverTestController;
import frc.robot.controllers.OperatorController;
import frc.robot.controllers.OperatorTestController;
import frc.robot.shuffleboard.CompetitionTab;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.DriveBase;
import frc.robot.subsystems.Spool;
import frc.robot.subsystems.Turret;

import static frc.robot.Constants.*;

import java.util.HashMap;

public class RobotContainer {

  //Subsystems
  private DriveBase m_driveBase;
  private Turret m_turret;
  private Arm m_arm;
  private Spool m_spool;
  private Claw m_claw;

  //Controllers
  private DriverController m_driverController;
  private OperatorController m_operatorController;
  private DriverTestController m_driverTestController;
  private OperatorTestController m_operatorTestController;
  
  //Shuffleboard Tabs
  private final CompetitionTab m_competitionTab;

  //Path Planning
  private boolean m_isTuning;

  private HashMap<String, Command> m_eventMap;

  public RobotContainer() {

    // Path Planning Tuning
    m_isTuning = true;
    if(m_isTuning) {PathPlannerServer.startServer(5811);}
    m_eventMap = new HashMap<String, Command>();

    //Subsystems
    m_driveBase = new DriveBase(false);
    m_turret = new Turret(false);
    m_arm = new Arm(false);
    m_spool = new Spool(false);
    m_claw = new Claw(false);

    //Controllers
    m_driverController = new DriverController(m_driveBase, m_claw);
    m_operatorController = new OperatorController(m_arm, m_spool, m_turret, m_claw);
    //m_driverTestController = new DriverTestController(m_driveBase, m_claw);
    //m_operatorTestController = new OperatorTestController(m_arm, m_spool, m_turret, m_claw);

    //Shuffleboard Tabs
    m_competitionTab = new CompetitionTab(m_driveBase, m_claw, getEventMap());
  
  }

  public Command getAutonomousCommand() {
    return m_competitionTab.getAuto();
    //return m_competitionTab.getAdvancedAuto();
    //return m_competitionTab.getPlannedAuto();
  } 

  public HashMap<String, Command> getEventMap() {
    //m_eventMap.put("pivotMidPeg", m_arm.setPivotGoal(0));
    //m_eventMap.put("pivotHome", m_arm.setPivotGoal(k_pivotOffsetRad));
    return m_eventMap;
  }
  
}
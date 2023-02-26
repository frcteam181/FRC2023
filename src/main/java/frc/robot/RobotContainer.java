package frc.robot;

import com.pathplanner.lib.server.PathPlannerServer;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.controllers.DriverController;
import frc.robot.controllers.OperatorController;
import frc.robot.shuffleboard.CompetitionTab;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.DriveBase;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Turret;

public class RobotContainer {

  //Subsystems
  private DriveBase m_driveBase;
  private Arm m_arm;
  private Turret m_turret;
  private Intake m_intake;

  //Controllers
  private DriverController m_driverController;
  private OperatorController m_operatorController;
  
  //Shuffleboard Tabs
  private final CompetitionTab m_competitionTab;

  public RobotContainer() {

    PathPlannerServer.startServer(5811);

    //Subsystems
    m_driveBase = new DriveBase();
    m_arm = new Arm();
    m_turret = new Turret();
    m_intake = new Intake();

    //Controllers
    m_driverController = new DriverController(m_driveBase);
    m_operatorController = new OperatorController(m_arm, m_turret, m_intake);

    //Shuffleboard Tabs
    m_competitionTab = new CompetitionTab(m_driveBase);
  
  }

  public Command getAutonomousCommand() {
    
    return m_competitionTab.getAuto();
  }
}
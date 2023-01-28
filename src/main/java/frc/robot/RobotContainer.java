package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.controllers.DriverController;
import frc.robot.controllers.OperatorController;
import frc.robot.shuffleboard.CompetitionTab;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.DriveBase;

public class RobotContainer {

  //Subsystems
  private DriveBase m_driveBase;
  private Arm m_arm;

  //Controllers
  private DriverController m_driverController;
  private OperatorController m_operatorController;
  
  //Shuffleboard Tabs
  private final CompetitionTab m_competitionTab;

  public RobotContainer() {

    //Subsystems
    m_driveBase = new DriveBase();
    m_arm = new Arm();

    //Controllers
    m_driverController = new DriverController(m_driveBase);
    m_operatorController = new OperatorController(m_arm);

    //Shuffleboard Tabs
    m_competitionTab = new CompetitionTab(m_driveBase, m_arm);
  
  }

  public Command getAutonomousCommand() {
    
    return m_competitionTab.getAuto();
  }
}
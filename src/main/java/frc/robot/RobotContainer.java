package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.controllers.DriverController;
import frc.robot.shuffleboard.CompetitionTab;
import frc.robot.subsystems.DriveBase;

public class RobotContainer {

  //Subsystems
  private DriveBase m_driveBase;

  //Controllers
  private DriverController m_driverController;
  
  //Shuffleboard Tabs
  private final CompetitionTab m_competitionTab;

  public RobotContainer() {

    //Subsystems
    m_driveBase = new DriveBase();

    //Controllers
    m_driverController = new DriverController(m_driveBase);

    //Shuffleboard Tabs
    m_competitionTab = new CompetitionTab(m_driveBase);
  
  }

  public Command getAutonomousCommand() {
    
    return m_competitionTab.getAuto();
  }
}
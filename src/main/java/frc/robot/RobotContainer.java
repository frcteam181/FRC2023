package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.controllers.DriverController;
import frc.robot.shuffleboard.CompetitionTab;

public class RobotContainer {

  //Subsystems
  //Controllers
  private DriverController m_driverController;
  
  //Shuffleboard Tabs
  private final CompetitionTab m_competitionTab;

  public RobotContainer() {

    //Subsystems
    //Controllers
    m_driverController = new DriverController(/*Send used subsystems*/);

    //Shuffleboard Tabs
    m_competitionTab = new CompetitionTab(/*Send used subsystems*/);
  
  }

  public Command getAutonomousCommand() {
    
    return m_competitionTab.getAuto();
  }
}
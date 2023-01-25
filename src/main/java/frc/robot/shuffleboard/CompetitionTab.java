package frc.robot.shuffleboard;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;

public class CompetitionTab {

    private ShuffleboardTab m_competitionTab;
    private SendableChooser<Command> m_autoChooser;
    
    public CompetitionTab(/*Used subsystems*/) {

        ShuffleboardTab m_competitionTab = Shuffleboard.getTab("Competition");

        //Auto Chooser
        m_autoChooser = new SendableChooser<Command>();

        //Used Subsystems

        //Set Default Auto
        //m_autoChooser.setDefaultOption("Taxi Only", new TaxiOnly(m_driveTrain));
        
        // Add autos here:
        //m_autoChooser.addOption("Taxi Only", new TaxiOnly(m_driveTrain));
    }

    public Command getAuto() {
        return m_autoChooser.getSelected();
    }
}
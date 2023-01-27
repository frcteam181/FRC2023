package frc.robot.shuffleboard;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveBase;

public class CompetitionTab {

    private ShuffleboardTab m_competitionTab;
    private SendableChooser<Command> m_autoChooser;

    private DriveBase m_driveBase;
    
    public CompetitionTab(DriveBase driveBase) {

        ShuffleboardTab m_competitionTab = Shuffleboard.getTab("Competition");

        //Auto Chooser
        m_autoChooser = new SendableChooser<Command>();

        //Used Subsystems
        m_driveBase = driveBase;

        //Set Default Auto
        //m_autoChooser.setDefaultOption("Taxi Only", new TaxiOnly(m_driveTrain));
        
        // Add autos here:
        //m_autoChooser.addOption("Taxi Only", new TaxiOnly(m_driveTrain));
        //
    }

    public Command getAuto() {
        return m_autoChooser.getSelected();
    }
}
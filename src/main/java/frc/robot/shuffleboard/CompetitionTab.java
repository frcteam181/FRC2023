package frc.robot.shuffleboard;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.DriveBase;

public class CompetitionTab {

    private ShuffleboardTab m_competitionTab;
    private SendableChooser<Command> m_autoChooser;

    private DriveBase m_driveBase;
    private Arm m_arm;
    
    public CompetitionTab(DriveBase driveBase, Arm arm) {

        ShuffleboardTab m_competitionTab = Shuffleboard.getTab("Competition");

        //Initialize used objects
        m_autoChooser = new SendableChooser<Command>();

        //Used Subsystems
        m_driveBase = driveBase;
        m_arm = arm;

        configureAuto();

        /* (Column , Row) starting from index #0  15x7 */
        m_competitionTab.add("Field", m_driveBase.getField()).withPosition(8, 0).withSize(7, 4);

    }

    public void configureAuto(/* May need to pass objects here. Meybe it'll be better to not do this method at all `\'-'/` */) {

        //Set Default Auto
        //m_autoChooser.setDefaultOption("Taxi Only", new TaxiOnly(m_driveTrain));

        // Add auto options here:
        //m_autoChooser.addOption("Taxi Only", new TaxiOnly(m_driveTrain));

        //m_competitionTab.add("Choose Auto", m_autoChooser).withWidget(BuiltInWidgets.kComboBoxChooser).withPosition(4, 3).withSize(2, 1);

    }

    public Command getAuto() {
        return m_autoChooser.getSelected();
    }

}
package frc.robot.shuffleboard;

import java.util.Map;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.auto_commands.doNothing;
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

        configureAuto(m_autoChooser, m_competitionTab);

        /* (Column , Row) starting from index #0  15x7 */
        m_competitionTab.add("Field", m_driveBase.getField()).withPosition(7, 0).withSize(7, 4);
        m_competitionTab.addDouble("Stored Pressure", m_driveBase::getStoredPSI).withWidget(BuiltInWidgets.kDial).withProperties(Map.of("min", 0, "max", 120)).withPosition(9, 4).withSize(2, 2);
        m_competitionTab.addDouble("Working Pressure", m_driveBase::getWorkingPSI).withWidget(BuiltInWidgets.kDial).withProperties(Map.of("min", 0, "max", 60)).withPosition(11, 4).withSize(2, 2);
        m_competitionTab.addBoolean("Compressor Satus", m_driveBase::getCompressorStatus).withPosition(4, 5).withSize(3, 1);
        //m_competitionTab.add("Pigeon 2", m_driveBase.getYaw()).withWidget(BuiltInWidgets.kGyro).withPosition(7, 4).withSize(2, 2);
        //m_competitionTab.add("Left Encoder", m_driveBase.getLeftEncoder()).withWidget(BuiltInWidgets.kEncoder).withPosition(0, 5).withSize(2, 2);
        //m_competitionTab.add("Right Encoder", m_driveBase.getRightEncoder()).withWidget(BuiltInWidgets.kEncoder).withPosition(2, 5).withSize(2, 2);
        //m_competitionTab.add("Power Distribution Hub", m_driveBase.getPDH()).withWidget(BuiltInWidgets.kPowerDistribution).withPosition(4, 0).withSize(3, 4);
        m_competitionTab.addDouble("Channel 7 (amps)", m_driveBase::getChannel7).withPosition(7, 4).withSize(1, 1);
        m_competitionTab.addDouble("Channel 12 (amps)", m_driveBase::getChannel12).withPosition(8, 4).withSize(1, 1);
    }
    

    public void configureAuto(SendableChooser chooser, ShuffleboardTab tab) {

        //Set Default Auto
        chooser.setDefaultOption("Do Nothing", new doNothing());

        // Add auto options here:
        chooser.addOption("Do Nothing", new doNothing());

        tab.add("Choose Auto", chooser).withWidget(BuiltInWidgets.kComboBoxChooser).withPosition(4, 4).withSize(3, 1);

    }

    public Command getAuto() {
        return m_autoChooser.getSelected();
    }

}
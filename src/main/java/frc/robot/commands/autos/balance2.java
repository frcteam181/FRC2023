package frc.robot.commands.autos;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveBase;

public class balance2 extends CommandBase {

    private DriveBase m_driveBase;

    private boolean m_phaseOne, m_phaseTwo, m_phaseThree;

    public balance2(DriveBase driveBase) {

        m_driveBase = driveBase;

        addRequirements(m_driveBase);

    }

    @Override
    public void initialize() {

        m_phaseOne = true;
        m_phaseTwo = false;
        m_phaseThree = false;
        
    }

    @Override
    public void execute() {

        if (m_driveBase.getPitch() < 5 && m_phaseOne) {
            m_driveBase.driveSpeed(0);
            if(m_driveBase.getPitch() >= 5) {
                m_phaseOne = false;
                m_phaseTwo = true;
                m_driveBase.driveSpeed(0);
            }
        }

        if(m_phaseTwo) {
            m_driveBase.autoBalance();
        }
       
    }

    @Override
    public void end(boolean interrupted) {
       
    }
    
}

package frc.robot.commands.autos;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveBase;

public class balance2 extends CommandBase {

    private DriveBase m_driveBase;

    private boolean m_pahseOne, m_phaseTwo, m_phaseThree;

    public balance2(DriveBase driveBase) {

        m_driveBase = driveBase;

        addRequirements(m_driveBase);

    }

    @Override
    public void initialize() {

        m_pahseOne = true;
        m_phaseTwo = false;
        m_phaseThree = false;
        
    }

    @Override
    public void execute() {

        if (m_driveBase.getPitch() < 5 && m_pahseOne) {
            m_driveBase.driveForward(0.2);
            if(m_driveBase.getPitch() >= 5) {
                m_pahseOne = false;
                m_phaseTwo = true;
                m_driveBase.driveForward(0);
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

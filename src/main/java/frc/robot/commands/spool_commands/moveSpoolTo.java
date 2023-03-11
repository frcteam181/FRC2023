package frc.robot.commands.spool_commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Spool;

public class moveSpoolTo extends CommandBase {

    private Spool m_spool;
    private double m_goalMeter;
    
    public moveSpoolTo(Spool spool, double goalMeters) {

        m_spool = spool;
        m_goalMeter = goalMeters;

        addRequirements(m_spool);

    }

    @Override
    public void initialize() {
        m_spool.moveTo(m_goalMeter);
    }

}

package frc.robot.commands.spool_commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Spool;

public class extendSpool extends CommandBase {

    private Spool m_spool;

    public extendSpool(Spool spool) {

        m_spool = spool;

        addRequirements(m_spool);
    }

    @Override
    public void initialize() {
        m_spool.extend();
    }

    @Override
    public void end(boolean interrupted) {
        m_spool.stop();
    }
    
}

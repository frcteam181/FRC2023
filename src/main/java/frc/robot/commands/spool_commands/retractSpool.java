package frc.robot.commands.spool_commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Spool;

public class retractSpool extends CommandBase {

    private Spool m_spool;

    public retractSpool(Spool spool) {

        m_spool = spool;

        addRequirements(m_spool);

    }

    @Override
    public void initialize() {
        m_spool.retract();
    }

    @Override
    public void end(boolean interrupted) {
        m_spool.stop();
    }
    
}

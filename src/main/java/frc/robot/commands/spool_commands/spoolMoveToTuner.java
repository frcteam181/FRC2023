package frc.robot.commands.spool_commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Spool;

public class spoolMoveToTuner extends CommandBase {

    private final Spool m_spool;;

    public spoolMoveToTuner(Spool spool) {
        
        m_spool = spool;

        addRequirements(m_spool);
    }

    @Override
    public void execute() {
        m_spool.moveToTuning();
    }
    
}
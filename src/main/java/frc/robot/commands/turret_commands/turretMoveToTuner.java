package frc.robot.commands.turret_commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Turret;

public class turretMoveToTuner extends CommandBase {

    private final Turret m_turret;

    public turretMoveToTuner (Turret turret) {
        
        m_turret = turret;

        addRequirements(m_turret);
    }

    @Override
    public void execute() {
        m_turret.moveTurretToTuning();;
    }
    
}
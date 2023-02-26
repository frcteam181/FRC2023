package frc.robot.commands.turret_commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Turret;

public class moveTurret extends CommandBase {

    public Turret m_turret;
    public double m_turretValue;

    public moveTurret(Turret turret, double turretValue) {

        m_turret = turret;
        m_turretValue = turretValue;

        addRequirements(m_turret);

    }

    @Override
    public void execute() {
        m_turret.move(m_turretValue);
    }

    @Override
    public void end(boolean interrupted) {
        m_turret.stop();
    }
    
}

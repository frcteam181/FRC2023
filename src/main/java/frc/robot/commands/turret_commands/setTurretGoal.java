package frc.robot.commands.turret_commands;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Turret;

public class setTurretGoal extends CommandBase {

    public Turret m_turret;
    public double m_turretGoal;
    public TrapezoidProfile.State m_goal;

    public setTurretGoal(Turret turret, double turretGoal) {

        m_turret = turret;
        m_turretGoal = turretGoal;

        addRequirements(m_turret);

    }

    @Override
    public void initialize() {
        if (m_turretGoal == Units.degreesToRadians(180) && m_turret.getAngle() <= 0) {
            m_turretGoal = Units.degreesToRadians(-180);
        }
        m_goal = new TrapezoidProfile.State(m_turretGoal, 0);
        m_turret.setGoal(m_goal);
    }
    
}

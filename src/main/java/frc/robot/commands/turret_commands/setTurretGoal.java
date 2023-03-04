package frc.robot.commands.turret_commands;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Turret;

public class setTurretGoal extends CommandBase {

    public Turret m_turret;
    public double m_turretGoalRad;
    public TrapezoidProfile.State m_goal;

    public setTurretGoal(Turret turret, double turretGoalRad) {

        m_turret = turret;
        m_turretGoalRad = turretGoalRad;

        addRequirements(m_turret);

    }

    @Override
    public void initialize() {
        if (Units.radiansToDegrees(m_turretGoalRad) == 180 && m_turret.getAngleDeg() <= 0) {
            m_turretGoalRad = Units.degreesToRadians(-180);
        }
        m_goal = new TrapezoidProfile.State(m_turretGoalRad, 0);
        m_turret.setGoal(m_goal);
    }
    
}

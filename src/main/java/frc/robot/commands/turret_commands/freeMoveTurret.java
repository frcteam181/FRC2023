package frc.robot.commands.turret_commands;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Turret;

public class freeMoveTurret extends CommandBase {

    private Turret m_turret;
    private boolean m_isMovingRight;

    public freeMoveTurret(Turret turret, boolean isMovingRight) {
        
        m_turret = turret;
        m_isMovingRight = isMovingRight;

        addRequirements(m_turret);
    }

    @Override
    public void initialize() {
        m_turret.disable();
    }

    @Override
    public void execute() {
        System.out.println("executing");
        if (m_isMovingRight) {
            if(m_turret.getAngleRad() >= Units.degreesToRadians(180)) {
                m_turret.move(0);
            } else {
                m_turret.move(-0.3);
            }
        } else {
            if(m_turret.getAngleRad() <= Units.degreesToRadians(-180)) {
                m_turret.move(0);
            } else {
                m_turret.move(0.3);
            }
        }
        m_turret.updateState();
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("Ended");
        m_turret.setGoal(m_turret.getAngleRad());
        m_turret.enable();
    }

}

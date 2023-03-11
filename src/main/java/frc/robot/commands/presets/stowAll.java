package frc.robot.commands.presets;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Turret;

public class stowAll extends CommandBase {

    private Turret m_turret;
    private Arm m_arm;
    private Claw m_claw;

    public stowAll(Turret turret, Arm arm, Claw claw) {

        m_turret = turret;
        m_arm = arm;
        m_claw = claw;

        addRequirements(m_turret, m_arm, m_claw);

    }

    
    
}

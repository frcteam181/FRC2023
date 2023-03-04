package frc.robot.commands.arm_commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;

public class spoolMoveToTuner extends CommandBase {

    private final Arm m_arm;;

    public spoolMoveToTuner(Arm arm) {
        
        m_arm = arm;

        addRequirements(m_arm);
    }

    @Override
    public void execute() {
        m_arm.movePivotToTuning();
    }
    
}
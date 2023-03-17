package frc.robot.commands.presets;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.arm_commands.freeMovePivot;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Spool;

import static frc.robot.Constants.*;

public class midScorePreset extends SequentialCommandGroup {

    private Arm m_arm;
    private Spool m_spool;

    public midScorePreset(Arm arm, Spool spool) {

        m_arm = arm;
        m_spool = spool;

        addCommands(
            new freeMovePivot(m_arm, true).raceWith(new WaitCommand(0.001)),
            m_arm.setPivotGoal(k_pivotMidScoreRad),
            new WaitCommand(2),
            m_spool.moveToCommand(k_spoolMidScoreMeter)
        );

        addRequirements(m_arm, m_spool);

    }
    
}

package frc.robot.commands.autos;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.arm_commands.freeMovePivot;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Spool;

import static frc.robot.Constants.*;

public class midScore extends SequentialCommandGroup {

    private Arm m_arm;
    private Spool m_spool;
    private Claw m_claw;

    public midScore(Arm arm, Spool spool, Claw claw) {

        m_arm = arm;
        m_spool = spool;
        m_claw = claw;

        addCommands(
            new freeMovePivot(m_arm, true).raceWith(new WaitCommand(0.001)),
            m_arm.setPivotGoal(k_pivotMidScoreRad).raceWith(new WaitCommand(2)),
            m_spool.moveToCommand(k_spoolMidScoreMeter).raceWith(new WaitCommand(2)),
          //  new WaitCommand(2),
            m_claw.switchModesCommand()
        );

        addRequirements(m_arm, m_spool, m_claw);

    }
    
}

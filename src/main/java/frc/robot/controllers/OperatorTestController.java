package frc.robot.controllers;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.commands.arm_commands.freeMovePivot;
import frc.robot.commands.claw_commands.clawSetSpeedTuner;
import frc.robot.commands.claw_commands.closeClaw;
import frc.robot.commands.claw_commands.intake;
import frc.robot.commands.claw_commands.openClaw;
import frc.robot.commands.claw_commands.outake;
import frc.robot.commands.claw_commands.stopIntake;
import frc.robot.commands.claw_commands.switchMode;
import frc.robot.commands.default_commands.ArmDefaultCommand;
import frc.robot.commands.spool_commands.extendSpool;
import frc.robot.commands.spool_commands.moveSpoolTo;
import frc.robot.commands.spool_commands.retractSpool;
import frc.robot.commands.spool_commands.spoolMoveToTuner;
import frc.robot.commands.turret_commands.freeMoveTurret;
import frc.robot.commands.turret_commands.setTurretGoal;
import frc.robot.commands.turret_commands.turretMoveToTuner;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Spool;
import frc.robot.subsystems.Turret;

import static frc.robot.Constants.*;

public class OperatorTestController {

    private XboxController m_operatorController;
    private JoystickButton m_a, m_b, m_x, m_y, m_lb, m_rb, m_sl, m_st, m_ls, m_rs;
    private POVButton m_up, m_upr, m_r, m_dwr, m_dw, m_dwl, m_l, m_upl;

    // Used Subsystems //
    private Arm m_arm;
    private Spool m_spool;
    private Turret m_turret;
    private Claw m_claw;

    public OperatorTestController(Arm arm, Spool spool, Turret turret, Claw claw) {
        
        // Used Subsystems Instances //
        m_arm = arm;
        m_turret = turret;
        m_claw = claw;
        m_spool = spool;

        // Xbox Controllers
        m_operatorController = new XboxController(k_OPERATOR_TEST_CONTROLLER);

        // Xbox Controller Buttons
        m_st = new JoystickButton(m_operatorController, XboxController.Button.kStart.value);
        m_sl = new JoystickButton(m_operatorController, XboxController.Button.kBack.value);
        m_lb = new JoystickButton(m_operatorController, XboxController.Button.kLeftBumper.value);
        m_rb = new JoystickButton(m_operatorController, XboxController.Button.kRightBumper.value);
        m_ls = new JoystickButton(m_operatorController, XboxController.Button.kLeftStick.value);
        m_rs = new JoystickButton(m_operatorController, XboxController.Button.kRightStick.value);
        m_a = new JoystickButton(m_operatorController, XboxController.Button.kA.value);
        m_b = new JoystickButton(m_operatorController, XboxController.Button.kB.value);
        m_x = new JoystickButton(m_operatorController, XboxController.Button.kX.value);
        m_y = new JoystickButton(m_operatorController, XboxController.Button.kY.value);
        m_up = new POVButton(m_operatorController, 0);
        m_upr = new POVButton(m_operatorController, 45);
        m_r = new POVButton(m_operatorController, 90);
        m_dwr = new POVButton(m_operatorController, 135);
        m_dw = new POVButton(m_operatorController, 180);
        m_dwl = new POVButton(m_operatorController, 225);
        m_l = new POVButton(m_operatorController, 270);
        m_upl = new POVButton(m_operatorController, 315);

        // Default Command
        m_arm.setDefaultCommand(new ArmDefaultCommand(m_arm, m_spool, m_turret, m_claw, m_operatorController));

        bindButtons();

    }

    public void bindButtons() {

        /* Pivot */
        m_rb.whileTrue(new freeMovePivot(m_arm, true)).onFalse(m_arm.setFinalPivotGoal());
        m_lb.whileTrue(new freeMovePivot(m_arm, false)).onFalse(m_arm.setFinalPivotGoal());
        //m_b.onTrue(m_arm.setPivotGoal(0));
        //m_a.onTrue(m_arm.setPivotGoal(k_pivotOffsetRad));
        //m_y.onTrue(m_arm.setTuningPivotGoal());

        /* Spool */
        //m_x.onTrue(new spoolMoveToTuner(m_arm));
        //m_lb.whileTrue(new retractSpool(m_arm));
        //m_rb.whileTrue(new extendSpool(m_arm));


        /* Claw */
 
        // Switch Game Piece Mode
        //m_st.onTrue(new switchMode(m_claw));
        //m_x.onTrue(new openClaw(m_claw));
        //m_y.onTrue(new closeClaw(m_claw));

        // Intaking
        //m_st.onTrue(new intake(m_claw));
        //m_sl.onTrue(new stopIntake(m_claw));
        //m_x.onTrue(new clawSetSpeedTuner(m_claw));
        //m_x.whileTrue(new intake(m_claw));
        //m_y.onTrue(new outake(m_claw));
        //m_st.onTrue(new stopIntake(m_claw));

        /* Turret */
        //m_rb.whileTrue(new freeMoveTurret(m_turret, true));
        //m_lb.whileTrue(new freeMoveTurret(m_turret, false));
        m_up.onTrue(m_turret.setTurretGoal(Units.degreesToRadians(0)));
        m_l.onTrue(m_turret.setTurretGoal(Units.degreesToRadians(90)));
        m_r.onTrue(m_turret.setTurretGoal(Units.degreesToRadians(-90)));
        m_dw.onTrue(m_turret.setTurretGoal(Units.degreesToRadians(180)));
        //m_x.onTrue(new turretMoveToTuner(m_turret));
        m_sl.whileTrue(m_spool.retractCommand()).onFalse(m_spool.stopCommand());
        m_st.whileTrue(m_spool.extendCommand()).onFalse(m_spool.stopCommand());

        /* Presets */
        m_x.onTrue(m_spool.moveToCommand(Units.inchesToMeters(0)).andThen(m_arm.setPivotGoal(k_pivotOffsetRad)));
        m_b.onTrue(m_arm.setPivotGoal(0).andThen(m_spool.moveToCommand(Units.inchesToMeters(10))));
        m_y.onTrue(m_arm.setPivotGoal(k_pivotHighScoreRad).andThen(m_spool.moveToCommand(Units.inchesToMeters(20))));
        m_a.onTrue(m_arm.setPivotGoal(k_pivotGroudIntake).andThen(m_spool.moveToCommand(Units.inchesToMeters(14.173))));
    }

}
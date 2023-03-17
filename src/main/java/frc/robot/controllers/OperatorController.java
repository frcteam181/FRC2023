package frc.robot.controllers;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.commands.arm_commands.freeMovePivot;
import frc.robot.commands.claw_commands.intake;
import frc.robot.commands.claw_commands.outake;
import frc.robot.commands.claw_commands.slowOutake;
import frc.robot.commands.claw_commands.stopIntake;
import frc.robot.commands.default_commands.ClawDefaultCommand;
import frc.robot.commands.presets.groundPickupPreset;
import frc.robot.commands.presets.highScorePreset;
import frc.robot.commands.presets.homePreset;
import frc.robot.commands.presets.midScorePreset;
import frc.robot.commands.presets.safetyPreset;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Spool;

import static frc.robot.Constants.*;

public class OperatorController {

    private XboxController m_operatorController;
    private JoystickButton m_a, m_b, m_x, m_y, m_lb, m_rb, m_sl, m_st, m_ls, m_rs;
    private POVButton m_up, m_upr, m_r, m_dwr, m_dw, m_dwl, m_l, m_upl;

    // Used Subsystems //
    private Arm m_arm;
    private Spool m_spool;
    private Claw m_claw;

    public OperatorController(Arm arm, Spool spool, Claw claw) {
        
        // Used Subsystems Instances //
        m_arm = arm;
        m_spool = spool;
        m_claw = claw;

        // Xbox Controllers
        m_operatorController = new XboxController(k_OERATOR_CONTROLLER);

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
        m_claw.setDefaultCommand(new ClawDefaultCommand(m_arm, m_spool, m_claw, m_operatorController));

        bindButtons();

    }

    public void bindButtons() {

        m_lb.whileTrue(new freeMovePivot(m_arm, false)).onFalse(m_arm.setFinalPivotGoal());
        m_rb.whileTrue(new freeMovePivot(m_arm, true)).onFalse(m_arm.setFinalPivotGoal());

        m_a.onTrue(new groundPickupPreset(m_arm, m_spool));
        m_b.onTrue(new midScorePreset(m_arm, m_spool));
        m_y.onTrue(new highScorePreset(m_arm, m_spool));
        m_x.onTrue(new safetyPreset(m_arm, m_spool));

        //m_a.whileTrue(new intake(m_claw)).onFalse(new stopIntake(m_claw));
        //m_b.whileTrue(new outake(m_claw)).onFalse(new stopIntake(m_claw));
        //m_y.whileTrue(new slowOutake(m_claw)).onFalse(new stopIntake(m_claw));
        //m_x.onTrue(m_claw.switchModesCommand());

        m_up.whileTrue(m_spool.extendCommand()).onFalse(m_spool.stopCommand());
        m_dw.whileTrue(m_spool.retractCommand()).onFalse(m_spool.stopCommand());

        //m_st.onTrue(m_arm.setTuningPivotGoal());
        m_sl.onTrue(m_spool.moveToTuning());

    }

}
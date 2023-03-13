package frc.robot.controllers;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.commands.claw_commands.intake;
import frc.robot.commands.claw_commands.outake;
import frc.robot.commands.claw_commands.slowOutake;
import frc.robot.commands.claw_commands.stopIntake;
import frc.robot.commands.default_commands.ClawDefaultCommand;
import frc.robot.subsystems.Claw;

import static frc.robot.Constants.*;

public class OperatorController {

    private XboxController m_operatorController;
    private JoystickButton m_a, m_b, m_x, m_y, m_lb, m_rb, m_sl, m_st, m_ls, m_rs;
    private POVButton m_up, m_upr, m_r, m_dwr, m_dw, m_dwl, m_l, m_upl;

    // Used Subsystems //
    private Claw m_claw;

    public OperatorController(Claw claw) {
        
        // Used Subsystems Instances //
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
        m_claw.setDefaultCommand(new ClawDefaultCommand(m_claw, m_operatorController));

        bindButtons();

    }

    public void bindButtons() {

        m_rb.whileTrue(new intake(m_claw)).onFalse(new stopIntake(m_claw));
        m_lb.whileTrue(new outake(m_claw)).onFalse(new stopIntake(m_claw));
        m_a.whileTrue(new slowOutake(m_claw)).onFalse(new stopIntake(m_claw));

        m_x.onTrue(m_claw.switchModesCommand());

    }

}
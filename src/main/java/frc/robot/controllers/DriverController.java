package frc.robot.controllers;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.Joystick.ButtonType;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.commands.claw_commands.intake;
import frc.robot.commands.claw_commands.outake;
import frc.robot.commands.claw_commands.slowOutake;
import frc.robot.commands.claw_commands.stopIntake;
import frc.robot.commands.default_commands.DriveBaseDefaultCommand;
import frc.robot.commands.default_commands.DriveBaseXboxDefaultCommand;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.DriveBase;

import static frc.robot.Constants.*;

public class DriverController {
    
    // Controllers && Joysticks
    private Joystick m_controller;
    private XboxController m_xController;

    private JoystickButton m_a, m_b, m_x, m_y, m_lb, m_rb, m_sl, m_st, m_ls, m_rs;
    private POVButton m_up, m_upr, m_r, m_dwr, m_dw, m_dwl, m_l, m_upl;

    // Buttons
    private JoystickButton m_tgJ, m_tbJ, m_nineJ, m_tenJ;

    // POV Buttons
    private POVButton m_upJ, m_dnJ, m_lJ, m_rJ;

    // Used Subsystems
    private DriveBase m_driveBase;
    private Claw m_claw;

    private boolean m_isXboxController;

    public DriverController(DriveBase driveBase, Claw claw, boolean isXboxController) {

        m_isXboxController = isXboxController;

        // Controller && Joysticks
        if(m_isXboxController) {
            m_xController = new XboxController(k_DRIVER_CONTROLLER);
        } else {
            m_controller = new Joystick(k_DRIVER_CONTROLLER);
        }

        // Used Subsystems - Instances
        m_driveBase = driveBase;
        m_claw = claw;

        // Buttons

        if(m_isXboxController) {
            // Xbox Controller Buttons
            m_st = new JoystickButton(m_xController, XboxController.Button.kStart.value);
            m_sl = new JoystickButton(m_xController, XboxController.Button.kBack.value);
            m_lb = new JoystickButton(m_xController, XboxController.Button.kLeftBumper.value);
            m_rb = new JoystickButton(m_xController, XboxController.Button.kRightBumper.value);
            m_ls = new JoystickButton(m_xController, XboxController.Button.kLeftStick.value);
            m_rs = new JoystickButton(m_xController, XboxController.Button.kRightStick.value);
            m_a = new JoystickButton(m_xController, XboxController.Button.kA.value);
            m_b = new JoystickButton(m_xController, XboxController.Button.kB.value);
            m_x = new JoystickButton(m_xController, XboxController.Button.kX.value);
            m_y = new JoystickButton(m_xController, XboxController.Button.kY.value);
            m_up = new POVButton(m_xController, 0);
            m_upr = new POVButton(m_xController, 45);
            m_r = new POVButton(m_xController, 90);
            m_dwr = new POVButton(m_xController, 135);
            m_dw = new POVButton(m_xController, 180);
            m_dwl = new POVButton(m_xController, 225);
            m_l = new POVButton(m_xController, 270);
            m_upl = new POVButton(m_xController, 315);
        } else {
            m_tgJ = new JoystickButton(m_controller, ButtonType.kTrigger.value);
            m_tbJ = new JoystickButton(m_controller, ButtonType.kTop.value);
            m_nineJ = new JoystickButton(m_controller, 9);
            m_tenJ = new JoystickButton(m_controller, 10);
        }

        // POV BUttons

        // Default Command

        if(m_isXboxController) {
            m_driveBase.setDefaultCommand(new DriveBaseXboxDefaultCommand(driveBase, claw, m_xController));
            bindXboxButtons();
        } else {
            m_driveBase.setDefaultCommand(new DriveBaseDefaultCommand(m_driveBase, m_claw, m_controller));
            bindJoystickButtons();
        }
    }

    public void bindJoystickButtons() {

        m_tbJ.onTrue(m_claw.switchModesCommand());
        m_tgJ.whileTrue(new intake(m_claw)).onFalse(new stopIntake(m_claw));
        m_nineJ.whileTrue(new outake(m_claw)).onFalse(new stopIntake(m_claw));
        m_tenJ.whileTrue(new slowOutake(m_claw)).onFalse(new stopIntake(m_claw));
        //m_y.whileTrue(new slowOutake(m_claw)).onFalse(new stopIntake(m_claw));

        //m_a.whileTrue(new intake(m_claw)).onFalse(new stopIntake(m_claw));
        //m_b.whileTrue(new outake(m_claw)).onFalse(new stopIntake(m_claw));
        //m_y.whileTrue(new slowOutake(m_claw)).onFalse(new stopIntake(m_claw));
        //m_x.onTrue(m_claw.switchModesCommand());
        
    }

    public void bindXboxButtons() {
        m_a.onTrue(m_claw.switchModesCommand());
        m_rb.whileTrue(new intake(m_claw)).onFalse(new stopIntake(m_claw));
        m_lb.whileTrue(new outake(m_claw)).onFalse(new stopIntake(m_claw));
        m_x.whileTrue(new slowOutake(m_claw)).onFalse(new stopIntake(m_claw));

    }

}
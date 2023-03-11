package frc.robot.controllers;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Joystick.ButtonType;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.commands.claw_commands.intake;
import frc.robot.commands.claw_commands.stopIntake;
import frc.robot.commands.default_commands.DriveBaseDefaultCommand;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.DriveBase;

import static frc.robot.Constants.*;

public class DriverTestController {
    
    // Controllers && Joysticks
    private Joystick m_controller;

    // Buttons
    private JoystickButton m_tg;

    // POV Buttons
    private POVButton m_up, m_dn, m_l, m_r;

    // Used Subsystems
    private DriveBase m_driveBase;
    private Claw m_claw;

    public DriverTestController(DriveBase driveBase, Claw claw) {

        // Controller && Joysticks
        m_controller = new Joystick(k_DRIVER_TEST_CONTROLLER);

        // Used Subsystems - Instances
        m_driveBase = driveBase;
        m_claw = claw;

        // Buttons

        m_tg = new JoystickButton(m_controller, ButtonType.kTrigger.value);

        // POV BUttons

        // Default Command

        //m_driveBase.setDefaultCommand(new DriveBaseDefaultCommand(m_driveBase, m_controller));

        bindButtons();
    }

    public void bindButtons() {

        m_tg.whileTrue(new intake(m_claw)).onFalse(new stopIntake(m_claw));
        
    }

}
package frc.robot.controllers;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Joystick.ButtonType;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.commands.default_commands.DriveBaseDefaultCommand;
import frc.robot.subsystems.DriveBase;

import static frc.robot.Constants.*;

public class DriverController {
    
    // Controllers && Joysticks
    private Joystick m_controller;

    // Buttons
    private JoystickButton m_tg;

    // POV Buttons
    private POVButton m_up, m_dn, m_l, m_r;

    // Used Subsystems
    private DriveBase m_driveBase;

    public DriverController(DriveBase driveBase) {

        // Controller && Joysticks
        m_controller = new Joystick(k_DRIVER_CONTROLLER);

        // Used Subsystems - Instances
        m_driveBase = driveBase;

        // Buttons

        m_tg = new JoystickButton(m_controller, ButtonType.kTrigger.value);

        // POV BUttons

        // Default Command

        m_driveBase.setDefaultCommand(new DriveBaseDefaultCommand(m_driveBase, m_controller));

        bindButtons();
    }

    public void bindButtons() {

        
        
    }

}
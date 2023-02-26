package frc.robot.controllers;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.commands.arm_commands.movePivotTo;
import frc.robot.commands.arm_commands.pivotDown;
import frc.robot.commands.arm_commands.pivotUp;
import frc.robot.commands.default_commands.ArmDefaultCommand;
import frc.robot.commands.intake_commands.intakeSpeedControl;
import frc.robot.commands.turret_commands.setTurretGoal;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Turret;

import static frc.robot.Constants.*;

public class OperatorController {

    private XboxController m_operatorController;
    private JoystickButton m_a, m_b, m_x, m_y, m_lb, m_rb, m_sl, m_st, m_ls, m_rs;
    private POVButton m_up, m_upr, m_r, m_dwr, m_dw, m_dwl, m_l, m_upl;
    private boolean m_isDefaultBinding;

    // Used Subsystems //
    private Arm m_arm;
    private Turret m_turret;
    private Intake m_intake;

    public OperatorController(Arm arm, Turret turret, Intake intake) {
        
        // Used Subsystems Instances //
        m_arm = arm;
        m_turret = turret;
        m_intake = intake;

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
        m_arm.setDefaultCommand(new ArmDefaultCommand(m_arm, m_turret, m_operatorController));

        bindButtons();

    }

    public void bindButtons() {

        /* Pivot */
        m_rb.whileTrue(new pivotDown(m_arm));
        m_lb.whileTrue(new pivotUp(m_arm));
        m_b.onTrue(m_arm.setPivotGoal(0));
        m_a.onTrue(m_arm.setPivotGoal(-62.6));
        m_y.onTrue(new movePivotTo(m_arm, 0));
        m_x.onTrue(new movePivotTo(m_arm, k_pivotOffset));

        /* Spool */

        /* Claw */

        /* Turret */
        m_up.onTrue(new setTurretGoal(m_turret, 0));
        m_l.onTrue(new setTurretGoal(m_turret, Units.degreesToRadians(90)));
        m_r.onTrue(new setTurretGoal(m_turret, Units.degreesToRadians(-90)));
        m_dw.onTrue(new setTurretGoal(m_turret, Units.degreesToRadians(180)));

        /* Intake */
        m_x.whileTrue(new intakeSpeedControl(m_intake, 0.25)); // Intake
        m_x.whileTrue(new intakeSpeedControl(m_intake, -0.25)); // Out take

    }

}
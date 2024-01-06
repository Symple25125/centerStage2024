package org.firstinspires.ftc.teamcode;


import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.commands.arm.ArmByJoystickCommand;
import org.firstinspires.ftc.teamcode.commands.claw.CloseClawCommand;
import org.firstinspires.ftc.teamcode.commands.claw.OpenClawCommand;
import org.firstinspires.ftc.teamcode.commands.drivebase.ArcadeDriveCommand;
import org.firstinspires.ftc.teamcode.commands.joint.MoveJointToPosition;
import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ClawSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveBaseSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.JointSubSystem;

import java.util.Collections;

@TeleOp(name ="test1")
public class Main extends CommandOpMode {
    private DriveBaseSubsystem driveBase;
    private ClawSubsystem clawSubsystem;
    private ArmSubsystem armSubsystem;
    private JointSubSystem jointSubsystem;

    private GamepadEx driverController;
    private GamepadEx actionController;

    @Override
    public void initialize() {
        this.driverController = new GamepadEx(gamepad1);
        this.actionController = new GamepadEx(gamepad2);

        this.driveBase = new DriveBaseSubsystem(hardwareMap);
//        this.clawSubsystem = new ClawSubsystem(hardwareMap);
        this.armSubsystem = new ArmSubsystem(hardwareMap, telemetry);
        this.jointSubsystem = new JointSubSystem(hardwareMap);


        initDefaultCommands();
        initButtons();
    }

    private double _tmp;

    @Override
    public void run() {
        super.run();
        telemetry.addData("DEG", String.valueOf(this.armSubsystem.getCurrentPositionDeg()));
        telemetry.update();
        this.armSubsystem.moveMotor(_tmp);
    }

    private void initDefaultCommands() {
        this.driveBase.setDefaultCommand(new ArcadeDriveCommand(this.driveBase, this.driverController));
//        this.armSubsystem.setDefaultCommand(new ArmByJoystickCommand(this.armSubsystem, this.actionController));
    }

    private void initButtons() {
        actionController.getGamepadButton(GamepadKeys.Button.A)
                .whenPressed(() -> _tmp -= 0.01);

        actionController.getGamepadButton(GamepadKeys.Button.Y)
                .whenPressed(() -> _tmp += 0.01);

        actionController.getGamepadButton(GamepadKeys.Button.X)
                .whenPressed(() -> _tmp += 0.001);

        actionController.getGamepadButton(GamepadKeys.Button.B)
                .whenPressed(() -> _tmp -= 0.001);

        actionController.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whenPressed(() -> _tmp = 0);

        actionController.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whenPressed(() -> _tmp = 1);

        // remember to change to action controller
        driverController.getGamepadButton(GamepadKeys.Button.A)
                .toggleWhenPressed(
                        new MoveJointToPosition(this.jointSubsystem, JointSubSystem.JointPositions.PICKUP),
                        new MoveJointToPosition(this.jointSubsystem, JointSubSystem.JointPositions.PUT)
                );

//        actionController.getGamepadButton(GamepadKeys.Button.A)
//                .toggleWhenPressed(
//                        new CloseClawCommand(clawSubsystem),
//                        new OpenClawCommand(clawSubsystem)
//                );
    }
}
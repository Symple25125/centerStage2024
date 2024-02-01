package org.firstinspires.ftc.teamcode;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commands.arm.ArmByJoystickCommand;
import org.firstinspires.ftc.teamcode.commands.arm.MoveArmToPointWithPID;
import org.firstinspires.ftc.teamcode.commands.claw.ClawStartPositionCommand;
import org.firstinspires.ftc.teamcode.commands.claw.CloseClawCommand;
import org.firstinspires.ftc.teamcode.commands.claw.MoveClawToPositionSlowCommand;
import org.firstinspires.ftc.teamcode.commands.claw.OpenBigClawCommand;
import org.firstinspires.ftc.teamcode.commands.claw.OpenClawCommand;
import org.firstinspires.ftc.teamcode.commands.drivebase.ArcadeDriveCommand;
import org.firstinspires.ftc.teamcode.commands.drivebase.MoveUnderCommand;
import org.firstinspires.ftc.teamcode.commands.hook.JoyStickHookCommand;
import org.firstinspires.ftc.teamcode.commands.hook.MoveToHookPositionCommand;
import org.firstinspires.ftc.teamcode.commands.joint.MoveJointToPosition;
import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ClawSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveBaseSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.HookSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.JointSubSystem;

@TeleOp(name="Driver Mode")
public class DriverOpMode extends CommandOpMode {
    private DriveBaseSubsystem driveBase;
    private ClawSubsystem clawSubsystem;
    private ArmSubsystem armSubsystem;
    private JointSubSystem jointSubsystem;
    private HookSubsystem hookSubsystem;

    private GamepadEx driverController;
    private GamepadEx actionController;

    @Override
    public void initialize() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        this.driverController = new GamepadEx(gamepad1);
        this.actionController = new GamepadEx(gamepad2);

        this.driveBase = new DriveBaseSubsystem(hardwareMap);
        this.clawSubsystem = new ClawSubsystem(hardwareMap);
        this.jointSubsystem = new JointSubSystem(hardwareMap);
        this.armSubsystem = new ArmSubsystem(hardwareMap, jointSubsystem);
        this.hookSubsystem = new HookSubsystem(hardwareMap);


        initDefaultCommands();
        initButtons();
    }

    @Override
    public void run() {
        super.run();
        telemetry.addData("ARM ANGLE", String.valueOf(this.armSubsystem.getCurrentPositionDeg()));
        telemetry.addData("CLAW ANGLE", String.valueOf(this.jointSubsystem.getClawJointAngle()));
        telemetry.update();
    }

    private void initDefaultCommands() {
        this.driveBase.setDefaultCommand(new ArcadeDriveCommand(this.driveBase, this.driverController));
        this.armSubsystem.setDefaultCommand(new ArmByJoystickCommand(this.armSubsystem, this.actionController));
        this.hookSubsystem.setDefaultCommand(new JoyStickHookCommand(this.hookSubsystem, this.actionController));

        // claw starting pos
//        new ClawStartPositionCommand(this.clawSubsystem).schedule();
//        this.jointSubsystem.moveServo(JointSubSystem.JointPositions.PICKUP);
    }

    private void initButtons() {
        this.driverController.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .toggleWhenPressed(
                        new InstantCommand(() -> this.driveBase.changeSpeedModifier(DriveBaseSubsystem.SLOW_SPEED_MODIFIER)),
                        new InstantCommand(() -> this.driveBase.changeSpeedModifier(DriveBaseSubsystem.NORMAL_SPEED_MODIFIER))
                );

        this.driverController.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .toggleWhenPressed(
                        new InstantCommand(() -> this.driveBase.setInverted(true)),
                        new InstantCommand(() -> this.driveBase.setInverted(false))
                );

        this.driverController.getGamepadButton(GamepadKeys.Button.X)
                .whenHeld(new MoveUnderCommand(this.armSubsystem, this.driveBase, this.jointSubsystem));

        this.driverController.getGamepadButton(GamepadKeys.Button.B)
                .whenPressed(new InstantCommand(() -> this.armSubsystem.resetPos()));

        this.actionController.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whenHeld(new ParallelCommandGroup(
                    new MoveArmToPointWithPID(this.armSubsystem, ArmSubsystem.ArmPositions.TAKE),
                    new MoveJointToPosition(this.jointSubsystem, JointSubSystem.JointPositions.PICKUP)
                ));

        this.actionController.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whenHeld(new ParallelCommandGroup(
                    new MoveArmToPointWithPID(this.armSubsystem, ArmSubsystem.ArmPositions.PLACE),
                    new MoveJointToPosition(this.jointSubsystem, JointSubSystem.JointPositions.PUT)
                ));

        this.actionController.getGamepadButton(GamepadKeys.Button.A)
                .toggleWhenPressed(
                        new CloseClawCommand(this.clawSubsystem),
                        new OpenClawCommand(this.clawSubsystem)
                );

        this.actionController.getGamepadButton(GamepadKeys.Button.Y)
                .whenPressed(new MoveToHookPositionCommand(this.armSubsystem, this.jointSubsystem));

        this.actionController.getGamepadButton(GamepadKeys.Button.B)
                .toggleWhenPressed(
                        new OpenBigClawCommand(this.clawSubsystem, ClawSubsystem.ClawPositions.BIG_OPEN),
                        new MoveClawToPositionSlowCommand(this.clawSubsystem, ClawSubsystem.ClawPositions.CLOSE)
                );

        new Trigger(() -> Math.abs(this.actionController.getRightY()) > 0.1).whileActiveOnce(new ArmByJoystickCommand(this.armSubsystem, this.actionController));
    }
}
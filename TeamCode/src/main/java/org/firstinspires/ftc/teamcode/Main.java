package org.firstinspires.ftc.teamcode;


import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commands.arm.ArmByJoystickCommand;
import org.firstinspires.ftc.teamcode.commands.arm.MoveArmToPointWithPID;
import org.firstinspires.ftc.teamcode.commands.claw.ClawStartPositionCommand;
import org.firstinspires.ftc.teamcode.commands.claw.CloseClawCommand;
import org.firstinspires.ftc.teamcode.commands.claw.OpenClawCommand;
import org.firstinspires.ftc.teamcode.commands.drivebase.ArcadeDriveCommand;
import org.firstinspires.ftc.teamcode.commands.drivebase.MoveUnderCommand;
import org.firstinspires.ftc.teamcode.commands.drivebase.SlowModeArcadeDriveCommand;
import org.firstinspires.ftc.teamcode.commands.joint.MoveJointToPosition;
import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ClawSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveBaseSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.JointSubSystem;

@TeleOp(name="Driver Mode")
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
        this.clawSubsystem = new ClawSubsystem(hardwareMap);
        this.jointSubsystem = new JointSubSystem(hardwareMap);
        this.armSubsystem = new ArmSubsystem(hardwareMap, jointSubsystem);


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

        // claw starting pos
        new ClawStartPositionCommand(this.clawSubsystem).schedule();
        this.jointSubsystem.moveServo(JointSubSystem.JointPositions.PICKUP);
    }

    private void initButtons() {
        this.driverController.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .toggleWhenPressed(
                        new ArcadeDriveCommand(this.driveBase, this.driverController),
                        new SlowModeArcadeDriveCommand(this.driveBase, this.driverController)
                );
        this.driverController.getGamepadButton(GamepadKeys.Button.X)
                .whenHeld(new MoveUnderCommand(this.armSubsystem, this.driveBase));

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
    }
}
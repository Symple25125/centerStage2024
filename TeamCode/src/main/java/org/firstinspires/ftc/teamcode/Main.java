package org.firstinspires.ftc.teamcode;


import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commands.arm.ArmByJoystickCommand;
import org.firstinspires.ftc.teamcode.commands.arm.MoveArmToPointWithPID;
import org.firstinspires.ftc.teamcode.commands.claw.ClawStartPositionCommand;
import org.firstinspires.ftc.teamcode.commands.claw.CloseClawCommand;
import org.firstinspires.ftc.teamcode.commands.claw.OpenClawCommand;
import org.firstinspires.ftc.teamcode.commands.drivebase.ArcadeDriveCommand;
import org.firstinspires.ftc.teamcode.commands.joint.MoveJointToPosition;
import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ClawSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveBaseSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.JointSubSystem;

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
        this.clawSubsystem = new ClawSubsystem(hardwareMap);
        this.jointSubsystem = new JointSubSystem(hardwareMap);
        this.armSubsystem = new ArmSubsystem(hardwareMap, jointSubsystem);


        initDefaultCommands();
        initButtons();
    }

    @Override
    public void run() {
        super.run();
        telemetry.addData("DEG", String.valueOf(this.armSubsystem.getCurrentPositionDeg()));
        telemetry.addData("CLAW ANGLE", String.valueOf(this.jointSubsystem.getClawJointAngle()));
        telemetry.addData("KC", String.valueOf(ArmSubsystem.KClawJoint));
        telemetry.update();
    }

    private void initDefaultCommands() {
        this.driveBase.setDefaultCommand(new ArcadeDriveCommand(this.driveBase, this.driverController));
        this.armSubsystem.setDefaultCommand(new ArmByJoystickCommand(this.armSubsystem, this.actionController));
        new ClawStartPositionCommand(this.clawSubsystem).schedule();
        this.jointSubsystem.moveServo(JointSubSystem.JointPositions.PICKUP);
    }

    private void initButtons() {
        // remember to change to action controller
        this.actionController.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whenHeld(new MoveArmToPointWithPID(this.armSubsystem, ArmSubsystem.ArmPositions.TAKE));

        this.actionController.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whenHeld(new MoveArmToPointWithPID(this.armSubsystem, ArmSubsystem.ArmPositions.PLACE));

        this.actionController.getGamepadButton(GamepadKeys.Button.A)
                .toggleWhenPressed(
                        new CloseClawCommand(this.clawSubsystem),
                        new OpenClawCommand(this.clawSubsystem)
                );

        this.actionController.getGamepadButton(GamepadKeys.Button.Y)
                .toggleWhenPressed(
                        new MoveJointToPosition(this.jointSubsystem, JointSubSystem.JointPositions.PICKUP),
                        new MoveJointToPosition(this.jointSubsystem, JointSubSystem.JointPositions.PUT)
                );
    }
}
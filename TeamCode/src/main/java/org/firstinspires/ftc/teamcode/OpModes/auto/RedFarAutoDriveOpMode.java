package org.firstinspires.ftc.teamcode.OpModes.auto;

import com.arcrobotics.ftclib.command.ParallelDeadlineGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.RobotController;
import org.firstinspires.ftc.teamcode.commands.arm.ArmGoToRestPositionCommand;
import org.firstinspires.ftc.teamcode.commands.arm.MoveArmToPositionCommand;
import org.firstinspires.ftc.teamcode.commands.claw.CloseClawCommand;
import org.firstinspires.ftc.teamcode.commands.claw.OpenClawCommand;
import org.firstinspires.ftc.teamcode.commands.drivebase.DriveDistanceDriveCommand;
import org.firstinspires.ftc.teamcode.commands.drivebase.RotateRobotByDegCommand;
import org.firstinspires.ftc.teamcode.commands.joint.MoveJointToPosition;
import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.JointSubsystem;
import org.firstinspires.ftc.teamcode.util.AutoOpMode;
import org.firstinspires.ftc.teamcode.util.OpModeType;
import org.firstinspires.ftc.teamcode.util.TeamColor;

@Autonomous(name = "red far Auto Drive")
public class RedFarAutoDriveOpMode extends AutoOpMode {
    private RobotController robotController;

    private static final long wait_time = 3000;

    @Override
    public void initialize() {
        this.robotController = new RobotController(OpModeType.Auto, hardwareMap, telemetry, gamepad1, gamepad2, TeamColor.RED);
        this.robotController.init();
    }

    @Override
    public void sympleStart() {
        this.robotController.sympleStart();
        new SequentialCommandGroup(
                new SequentialCommandGroup(
                        new ParallelDeadlineGroup(
                                new WaitCommand(500),
                                new MoveArmToPositionCommand(this.robotController.armSubsystem, ArmSubsystem.ArmPositions.PICKUP),
                                new MoveJointToPosition(this.robotController.jointSubsystem, JointSubsystem.JointPositions.PICKUP)
                        ),
                        new CloseClawCommand(this.robotController.clawSubsystem)
                ),
                new WaitCommand(3000),
                new ParallelDeadlineGroup(
                        new WaitCommand(wait_time),
                        new ArmGoToRestPositionCommand(this.robotController.armSubsystem),
                        new MoveJointToPosition(this.robotController.jointSubsystem, JointSubsystem.JointPositions.REST)
                ),
                new DriveDistanceDriveCommand(this.robotController.driveBase, 1.30f).withTimeout(7000),
                new RotateRobotByDegCommand(this.robotController.driveBase, -90).withTimeout(7000),
                new DriveDistanceDriveCommand(this.robotController.driveBase, 2.5f).withTimeout(7000),
                new WaitCommand(500),
                new OpenClawCommand(this.robotController.clawSubsystem),
                new WaitCommand(500),
                new DriveDistanceDriveCommand(this.robotController.driveBase, -0.35f)
        ).schedule();
    }
}

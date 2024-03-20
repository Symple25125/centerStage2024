package org.firstinspires.ftc.teamcode.OpModes.auto;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.RobotController;
import org.firstinspires.ftc.teamcode.commands.arm.MoveArmToPositionCommand;
import org.firstinspires.ftc.teamcode.commands.claw.CloseClawCommand;
import org.firstinspires.ftc.teamcode.commands.drivebase.DriveDistanceDriveCommand;
import org.firstinspires.ftc.teamcode.commands.joint.MoveJointToPosition;
import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.JointSubsystem;
import org.firstinspires.ftc.teamcode.util.AutoOpMode;
import org.firstinspires.ftc.teamcode.util.OpModeType;
import org.firstinspires.ftc.teamcode.util.TeamColor;

@Config
@Autonomous(name = "Test Front Drive", group = "test")
public class TestFrontDriveAutoOpMode extends AutoOpMode {
    private RobotController robotController;

    public static double METERS = 0.1f;


    @Override
    public void initialize() {
        this.robotController = new RobotController(OpModeType.Auto, hardwareMap, telemetry, gamepad1, gamepad2, TeamColor.RED);
    }

    @Override
    public void sympleStart() {
        new CloseClawCommand(this.robotController.clawSubsystem).schedule();

        new ParallelCommandGroup(
                new MoveArmToPositionCommand(this.robotController.armSubsystem, ArmSubsystem.ARM_STARTING_DEG + 15f),
                new MoveJointToPosition(this.robotController.jointSubsystem, JointSubsystem.JointPositions.PUT)
        ).schedule();

        new DriveDistanceDriveCommand(this.robotController.driveBase, METERS).schedule();
    }

    @Override
    public void run() {
        super.run();

        telemetry.addData("ARM ANGLE", String.valueOf(this.robotController.armSubsystem.getCurrentPos()));
        telemetry.addData("Arm Command", String.valueOf(this.robotController.armSubsystem.getCurrentCommand().getName()));

        telemetry.update();
    }
}

package org.firstinspires.ftc.teamcode.OpModes.auto;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.RobotController;
import org.firstinspires.ftc.teamcode.commands.arm.MoveArmToPositionCommand;
import org.firstinspires.ftc.teamcode.commands.claw.CloseClawCommand;
import org.firstinspires.ftc.teamcode.commands.drivebase.RotateRobotByDegCommand;
import org.firstinspires.ftc.teamcode.commands.joint.MoveJointToPosition;
import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.JointSubsystem;
import org.firstinspires.ftc.teamcode.util.AutoOpMode;
import org.firstinspires.ftc.teamcode.util.OpModeType;
import org.firstinspires.ftc.teamcode.util.TeamColor;

@Config
@Autonomous(name = "Test Rotate Drive", group = "test")
public class TestRotateDriveAutoOpMode extends AutoOpMode {
    private RobotController robotController;

    public static double DEGS = 180f;


    @Override
    public void initialize() {
        this.robotController = new RobotController(OpModeType.Auto, hardwareMap, telemetry, gamepad1, gamepad2, TeamColor.RED);
        this.robotController.init();
    }

    @Override
    public void sympleStart() {
        this.robotController.sympleStart();

        new CloseClawCommand(this.robotController.clawSubsystem).schedule();

        new ParallelCommandGroup(
                new MoveArmToPositionCommand(this.robotController.armSubsystem, ArmSubsystem.ARM_STARTING_DEG + 15f),
                new MoveJointToPosition(this.robotController.jointSubsystem, JointSubsystem.JointPositions.SCORE_UPPER)
        ).schedule();

        new RotateRobotByDegCommand(this.robotController.driveBase, DEGS).schedule();
    }

    @Override
    public void run() {
        super.run();

        telemetry.addData("ARM ANGLE", String.valueOf(this.robotController.armSubsystem.getCurrentPos()));
        telemetry.addData("Arm Command", String.valueOf(this.robotController.armSubsystem.getCurrentCommand().getName()));

        telemetry.update();
    }
}

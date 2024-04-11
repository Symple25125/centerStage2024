package org.firstinspires.ftc.teamcode.OpModes.auto;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.RobotController;
import org.firstinspires.ftc.teamcode.commands.arm.MoveArmToPositionCommand;
import org.firstinspires.ftc.teamcode.commands.claw.CloseClawCommand;
import org.firstinspires.ftc.teamcode.commands.drivebase.MoveMotorDiveCommand;
import org.firstinspires.ftc.teamcode.commands.joint.MoveJointToPosition;
import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.JointSubsystem;
import org.firstinspires.ftc.teamcode.util.AutoOpMode;
import org.firstinspires.ftc.teamcode.util.OpModeType;
import org.firstinspires.ftc.teamcode.util.TeamColor;

@Autonomous(name = "Back Auto Drive")
public class BackAutoDriveOpMode extends AutoOpMode {
    private RobotController robotController;

    private static final long drive_time = 1500;
    private static final long wait_time = 2000;
    private static final double motor_power = -0.7;


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
                new MoveArmToPositionCommand(this.robotController.armSubsystem, ArmSubsystem.ArmPositions.HOOK),
                new MoveJointToPosition(this.robotController.jointSubsystem, JointSubsystem.JointPositions.SCORE_UPPER)
        ).schedule();

        new WaitCommand(wait_time)
                .andThen(new MoveMotorDiveCommand(this.robotController.driveBase, drive_time, motor_power, motor_power))
                .schedule();
    }

    @Override
    public void run() {
        super.run();

        telemetry.addData("ARM ANGLE", String.valueOf(this.robotController.armSubsystem.getCurrentPos()));
        telemetry.addData("Arm Command", String.valueOf(this.robotController.armSubsystem.getCurrentCommand().getName()));
        telemetry.update();
    }
}

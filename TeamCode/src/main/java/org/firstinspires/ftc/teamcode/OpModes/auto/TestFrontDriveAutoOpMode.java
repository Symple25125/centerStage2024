package org.firstinspires.ftc.teamcode.OpModes.auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.commands.arm.MoveArmToPointWithPID;
import org.firstinspires.ftc.teamcode.commands.claw.CloseClawCommand;
import org.firstinspires.ftc.teamcode.commands.drivebase.DriveDistanceDriveCommand;
import org.firstinspires.ftc.teamcode.commands.drivebase.MoveMotorDiveCommand;
import org.firstinspires.ftc.teamcode.commands.joint.MoveJointToPosition;
import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ClawSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveBaseSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.JointSubSystem;

@Config
@Autonomous(name = "Test Front Drive", group = "test")
public class TestFrontDriveAutoOpMode extends CommandOpMode {
    private DriveBaseSubsystem driveBase;
    private JointSubSystem jointSubSystem;
    private ArmSubsystem armSubsystem;
    private ClawSubsystem clawSubsystem;

    public static double METERS = 0.1f;


    @Override
    public void initialize() {
        this.telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        this.driveBase = new DriveBaseSubsystem(hardwareMap);
        this.jointSubSystem = new JointSubSystem(hardwareMap);
        this.armSubsystem = new ArmSubsystem(hardwareMap);
        this.clawSubsystem = new ClawSubsystem(hardwareMap);

        new CloseClawCommand(this.clawSubsystem).schedule();

        new ParallelCommandGroup(
                new MoveArmToPointWithPID(this.armSubsystem, ArmSubsystem.ARM_STARTING_DEG + 15f),
                new MoveJointToPosition(this.jointSubSystem, JointSubSystem.JointPositions.PUT)
        ).schedule();

        new DriveDistanceDriveCommand(this.driveBase, METERS).schedule();
    }

    @Override
    public void run() {
        super.run();

        telemetry.addData("ARM ANGLE", String.valueOf(this.armSubsystem.getCurrentPos()));
        telemetry.addData("Arm Command", String.valueOf(this.armSubsystem.getCurrentCommand().getName()));

        telemetry.update();
    }
}

package org.firstinspires.ftc.teamcode.OpModes.auto;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.ParallelDeadlineGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.commands.arm.GoToPickupPositionCommand;
import org.firstinspires.ftc.teamcode.commands.arm.MoveArmToPointWithPID;
import org.firstinspires.ftc.teamcode.commands.claw.CloseClawCommand;
import org.firstinspires.ftc.teamcode.commands.claw.OpenClawCommand;
import org.firstinspires.ftc.teamcode.commands.drivebase.DriveDistanceDriveCommand;
import org.firstinspires.ftc.teamcode.commands.drivebase.MoveMotorDiveCommand;
import org.firstinspires.ftc.teamcode.commands.joint.EnableJointCommand;
import org.firstinspires.ftc.teamcode.commands.joint.MoveJointToPosition;
import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ClawSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveBaseSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.JointSubSystem;
import org.firstinspires.ftc.teamcode.util.AutoOpMode;

@Autonomous(name = "Front Auto Drive")
public class FrontAutoDriveOpMode extends AutoOpMode {
    private DriveBaseSubsystem driveBase;
    private JointSubSystem jointSubsystem;
    private ArmSubsystem armSubsystem;
    private ClawSubsystem clawSubsystem;

    private static final long drive_time = 3500;
    private static final long wait_time = 6000;
    private static final double motor_power = 0.5;

    @Override
    public void initialize() {
        this.driveBase = new DriveBaseSubsystem(hardwareMap);
        this.jointSubsystem = new JointSubSystem(hardwareMap);
        this.armSubsystem = new ArmSubsystem(hardwareMap);
        this.clawSubsystem = new ClawSubsystem(hardwareMap);
    }

    @Override
    public void sympleStart() {
        new EnableJointCommand(this.jointSubsystem).schedule();

        new SequentialCommandGroup(
                new SequentialCommandGroup(
                        new ParallelDeadlineGroup(
                                new WaitCommand(500),
                                new GoToPickupPositionCommand(this.armSubsystem),
                                new MoveJointToPosition(this.jointSubsystem, JointSubSystem.JointPositions.PICKUP)
                        ),
                        new CloseClawCommand(this.clawSubsystem)
                ),
                new WaitCommand(3000),
                new ParallelDeadlineGroup(
                        new WaitCommand(wait_time),
                        new MoveArmToPointWithPID(this.armSubsystem, ArmSubsystem.ArmPositions.REST),
                        new MoveJointToPosition(this.jointSubsystem, JointSubSystem.JointPositions.REST)
                ),
                new DriveDistanceDriveCommand(this.driveBase, 1.15f).withTimeout(7000),
                new WaitCommand(500),
                new OpenClawCommand(this.clawSubsystem),
                new WaitCommand(500),
                new DriveDistanceDriveCommand(this.driveBase, -0.35f)
        ).schedule();
    }


//    @Override
//    public void runOpMode() throws InterruptedException {
//        this.initialize();
//        this.waitForStart();
//        this.sympleStart();
//
//        while(!this.isStopRequested() && this.opModeIsActive()) {
//            this.run();
//        }
//
//        this.reset();
//    }
}

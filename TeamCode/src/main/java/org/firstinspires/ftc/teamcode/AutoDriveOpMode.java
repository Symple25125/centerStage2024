package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.ParallelDeadlineGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.util.Timing;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.commands.arm.MoveArmToPointWithPID;
import org.firstinspires.ftc.teamcode.commands.joint.MoveJointToPosition;
import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveBaseSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.JointSubSystem;

import java.util.concurrent.TimeUnit;

@Autonomous(name = "Auto Drive")
public class AutoDriveOpMode extends LinearOpMode {

    public static long time = 1000;

    @Override
    public void runOpMode() throws InterruptedException {
        DriveBaseSubsystem driveBase = new DriveBaseSubsystem(hardwareMap);
        JointSubSystem jointSubSystem = new JointSubSystem(hardwareMap);
        ArmSubsystem armSubsystem = new ArmSubsystem(hardwareMap, jointSubSystem);

        waitForStart();

        if (isStopRequested()) return;

        Timing.Timer timer = new Timing.Timer(time, TimeUnit.MILLISECONDS);

        timer.start();

        armSubsystem.setDefaultCommand(new MoveArmToPointWithPID(armSubsystem, armSubsystem.getCurrentPositionDeg() + 10));

        while(!timer.done()) {
            driveBase.moveMotors(-0.75, -0.75);
            telemetry.addData("status", "running");
            telemetry.update();
        }


        driveBase.moveMotors(0, 0);


        new SequentialCommandGroup(
                new MoveArmToPointWithPID(armSubsystem, 150, false, 0.65),
                new ParallelDeadlineGroup(
                        new MoveJointToPosition(jointSubSystem, JointSubSystem.JointPositions.HOOK),
                        new MoveArmToPointWithPID(armSubsystem, armSubsystem.getCurrentPositionDeg())
                ),
                new ParallelDeadlineGroup(
                    new MoveJointToPosition(jointSubSystem, JointSubSystem.JointPositions.PICKUP),
                    new MoveArmToPointWithPID(armSubsystem, ArmSubsystem.STARTING_DEG)
                )
        ).schedule();

        while (!isStopRequested() && opModeIsActive()) ;

        telemetry.addData("status", "stopped");
        telemetry.update();
    }
}

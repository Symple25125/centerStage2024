package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.util.Timing;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleTankDrive;
import org.firstinspires.ftc.teamcode.subsystems.DriveBaseSubsystem;

import java.sql.Time;
import java.util.concurrent.TimeUnit;

@Autonomous(name = "Auto Drive")
public class AutoDriveOpMode extends LinearOpMode {

    public static long time = 1500;

    @Override
    public void runOpMode() throws InterruptedException {
        DriveBaseSubsystem driveBase = new DriveBaseSubsystem(hardwareMap);

        waitForStart();

        if (isStopRequested()) return;

        Timing.Timer timer = new Timing.Timer(time, TimeUnit.MILLISECONDS);

        timer.start();

        while(!timer.done()) {
            driveBase.moveMotors(0.75, 0.75);
            telemetry.addData("status", "running");
            telemetry.update();
        }

        driveBase.moveMotors(0, 0);
        telemetry.addData("status", "stopped");
        telemetry.update();
    }
}

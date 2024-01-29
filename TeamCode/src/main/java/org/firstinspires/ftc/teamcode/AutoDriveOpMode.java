package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleTankDrive;

public class AutoDriveOpMode extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SampleTankDrive tankDrive = new SampleTankDrive(hardwareMap);
    }
}

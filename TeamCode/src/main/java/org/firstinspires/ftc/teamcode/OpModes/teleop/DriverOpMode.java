package org.firstinspires.ftc.teamcode.OpModes.teleop;


import com.arcrobotics.ftclib.command.CommandOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RobotController;

import org.firstinspires.ftc.teamcode.util.OpModeType;
import org.firstinspires.ftc.teamcode.util.TeamColor;

@TeleOp(name="Driver Mode")
public class DriverOpMode extends CommandOpMode {
    private RobotController robotController;

    @Override
    public void initialize() {
        robotController = new RobotController(OpModeType.TeleOP, hardwareMap, telemetry, gamepad1, gamepad2, TeamColor.RED);
    }

    @Override
    public void run() {
        super.run();
        telemetry.addData("ARM ANGLE", String.valueOf(this.robotController.armSubsystem.getCurrentPos()));
        telemetry.addData("CLAW JOINT ANGLE", String.valueOf(this.robotController.jointSubsystem.getClawJointAngle()));
        telemetry.addData("CLAW ANGLE", String.valueOf(this.robotController.clawSubsystem.getClawDeg()));
        telemetry.addData("Arm Command", String.valueOf(this.robotController.armSubsystem.getCurrentCommand().getName()));
        telemetry.update();
    }
}
package org.firstinspires.ftc.teamcode.OpModes.teleop;


import com.arcrobotics.ftclib.command.CommandOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RobotController;

import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.util.OpModeType;
import org.firstinspires.ftc.teamcode.util.SympleCommandOpMode;
import org.firstinspires.ftc.teamcode.util.TeamColor;

@TeleOp(name="Driver Mode")
public class DriverOpMode extends SympleCommandOpMode {
    private RobotController robotController;

    @Override
    public void initialize() {
        this.robotController = new RobotController(OpModeType.TeleOP, hardwareMap, telemetry, gamepad1, gamepad2, TeamColor.RED);
        this.robotController.init();
    }

    @Override
    public void sympleStart() {
        this.robotController.init();
    }

    @Override
    public void run() {
        super.run();
        robotController.getTelemetry().addData("ARM ANGLE", String.valueOf(this.robotController.armSubsystem.getCurrentPos()));
        robotController.getTelemetry().addData("CLAW JOINT ANGLE", String.valueOf(this.robotController.jointSubsystem.getClawJointAngle()));
        robotController.getTelemetry().addData("CLAW ANGLE", String.valueOf(this.robotController.clawSubsystem.getClawDeg()));
        robotController.getTelemetry().addData("Arm Command", String.valueOf(this.robotController.armSubsystem.getCurrentCommand().getName()));
        robotController.getTelemetry().addData("Arm State", ArmSubsystem.getState());
        robotController.getTelemetry().update();
    }
}
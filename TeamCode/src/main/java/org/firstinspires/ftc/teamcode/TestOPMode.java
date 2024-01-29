package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.util.Motors;

@TeleOp(name = "Robot Testing")
public class TestOPMode extends CommandOpMode {
    MotorEx motor = null;

    @Override
    public void initialize() {
        motor = new MotorEx(hardwareMap, Motors.CLAW_MOTOR.id);
    }

    @Override
    public void run() {
        motor.set(gamepad1.right_stick_y);
    }
}

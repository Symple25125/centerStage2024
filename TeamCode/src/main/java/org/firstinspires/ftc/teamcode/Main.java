package org.firstinspires.ftc.teamcode;


import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.commands.claw.CloseClawCommand;
import org.firstinspires.ftc.teamcode.commands.claw.OpenClawCommand;
import org.firstinspires.ftc.teamcode.subsystems.ClawSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveBaseSubsystem;

public class Main extends CommandOpMode {
    private DriveBaseSubsystem driveBase;
    private ClawSubsystem clawSubsystem;

    private GamepadEx driverController;
    private GamepadEx actionController;

    @Override
    public void initialize() {
        this.driverController = new GamepadEx(gamepad1);
        this.actionController = new GamepadEx(gamepad2);

        this.driveBase = new DriveBaseSubsystem(hardwareMap);
        this.clawSubsystem = new ClawSubsystem(hardwareMap);

        initButtons();
    }


    private void initButtons() {
        actionController.getGamepadButton(GamepadKeys.Button.A)
                .toggleWhenPressed(
                        new CloseClawCommand(clawSubsystem),
                        new OpenClawCommand(clawSubsystem)
                );
    }
}
package org.firstinspires.ftc.teamcode.commands.arm;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;

public class ArmByJoystickCommand extends CommandBase {
    private final GamepadEx gamepad;
    private final ArmSubsystem armSubsystem;

    public ArmByJoystickCommand(ArmSubsystem armSubsystem, GamepadEx gamepad) {
        addRequirements(armSubsystem);
        this.armSubsystem = armSubsystem;
        this.gamepad = gamepad;
    }

    @Override
    public void execute() {
        this.armSubsystem.moveMotor(-this.gamepad.getRightY());
    }
}

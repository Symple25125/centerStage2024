package org.firstinspires.ftc.teamcode.commands.arm;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;

public class RobotGoUpCommand extends MoveArmToPointWithPID {
    public RobotGoUpCommand(ArmSubsystem armSubsystem){
        super(armSubsystem, ArmSubsystem.ArmPositions.GRAB, 0.5f);
        super.MAX_POWER = 1;
    }
}


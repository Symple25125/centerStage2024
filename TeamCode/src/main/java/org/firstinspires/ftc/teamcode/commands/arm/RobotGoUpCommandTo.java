package org.firstinspires.ftc.teamcode.commands.arm;

import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;

public class RobotGoUpCommandTo extends MoveArmToPositionCommand {
    public RobotGoUpCommandTo(ArmSubsystem armSubsystem){
        super(armSubsystem, ArmSubsystem.ArmPositions.GRAB, 0.5f);
        super.MAX_POWER = 1;
    }
}


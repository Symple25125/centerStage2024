package org.firstinspires.ftc.teamcode.commands.joint;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.subsystems.JointSubsystem;

public class MoveJointToPosition extends InstantCommand {
    public MoveJointToPosition(JointSubsystem jointSubSystem, JointSubsystem.JointPositions jointPosition) {
        super(() -> jointSubSystem.moveServo(jointPosition));
    }
}
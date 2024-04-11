package org.firstinspires.ftc.teamcode.commands.arm;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.RobotController;
import org.firstinspires.ftc.teamcode.commands.claw.MoveClawToPosition;
import org.firstinspires.ftc.teamcode.commands.joint.MoveJointToPosition;
import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ClawSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.JointSubsystem;

public class PlacePurplePixelCommand extends SequentialCommandGroup {
    public PlacePurplePixelCommand(RobotController robotController) {
        super();

        addCommands(
                new WaitCommand(250),
                new MoveArmToPositionCommand(robotController.armSubsystem, ArmSubsystem.ArmPositions.PICKUP),
                new MoveJointToPosition(robotController.jointSubsystem, JointSubsystem.JointPositions.PICKUP),
                new WaitCommand(1000),
                new MoveClawToPosition(robotController.clawSubsystem, ClawSubsystem.ClawPositions.HALF_OPEN),
                new WaitCommand(1000),
                new MoveJointToPosition(robotController.jointSubsystem, JointSubsystem.JointPositions.REST),
                new ArmGoToRestPositionCommand(robotController.armSubsystem),
                new WaitCommand(250)
        );

        this.withTimeout(4000);
    }
}

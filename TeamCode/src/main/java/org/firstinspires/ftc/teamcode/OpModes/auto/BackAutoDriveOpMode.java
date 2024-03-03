package org.firstinspires.ftc.teamcode.OpModes.auto;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.arcrobotics.ftclib.util.Timing;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.commands.arm.MoveArmToPointWithPID;
import org.firstinspires.ftc.teamcode.commands.claw.CloseClawCommand;
import org.firstinspires.ftc.teamcode.commands.drivebase.MoveMotorDiveCommand;
import org.firstinspires.ftc.teamcode.commands.joint.MoveJointToPosition;
import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ClawSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveBaseSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.JointSubSystem;

import java.util.concurrent.TimeUnit;

@Autonomous(name = "Back Auto Drive")
public class BackAutoDriveOpMode extends CommandOpMode {
    private DriveBaseSubsystem driveBase;
    private JointSubSystem jointSubSystem;
    private ArmSubsystem armSubsystem;
    private ClawSubsystem clawSubsystem;

    private static final long drive_time = 1500;
    private static final long wait_time = 2000;
    private static final double motor_power = -0.7;


    @Override
    public void initialize() {
        this.driveBase = new DriveBaseSubsystem(hardwareMap);
        this.jointSubSystem = new JointSubSystem(hardwareMap);
        this.armSubsystem = new ArmSubsystem(hardwareMap);
        this.clawSubsystem = new ClawSubsystem(hardwareMap);

        new CloseClawCommand(this.clawSubsystem).schedule();

        new ParallelCommandGroup(
                new MoveArmToPointWithPID(this.armSubsystem, ArmSubsystem.ArmPositions.HOOK),
                new MoveJointToPosition(this.jointSubSystem, JointSubSystem.JointPositions.PUT)
        ).schedule();

        new WaitCommand(wait_time)
                .andThen(new MoveMotorDiveCommand(this.driveBase, drive_time, motor_power, motor_power))
                .schedule();
    }

    @Override
    public void run() {
        super.run();

        telemetry.addData("ARM ANGLE", String.valueOf(this.armSubsystem.getCurrentPos()));
        telemetry.addData("Arm Command", String.valueOf(this.armSubsystem.getCurrentCommand().getName()));
        telemetry.update();
    }
}

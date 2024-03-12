package org.firstinspires.ftc.teamcode.OpModes.auto;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.commands.arm.MoveArmToPointWithPID;
import org.firstinspires.ftc.teamcode.commands.claw.CloseClawCommand;
import org.firstinspires.ftc.teamcode.commands.claw.OpenClawCommand;
import org.firstinspires.ftc.teamcode.commands.drivebase.MoveMotorDiveCommand;
import org.firstinspires.ftc.teamcode.commands.joint.EnableJointCommand;
import org.firstinspires.ftc.teamcode.commands.joint.MoveJointToPosition;
import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ClawSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveBaseSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.JointSubSystem;

@Autonomous(name = "Front Auto Drive")
public class FrontAutoDriveOpMode extends OpMode {
    private DriveBaseSubsystem driveBase;
    private JointSubSystem jointSubsystem;
    private ArmSubsystem armSubsystem;
    private ClawSubsystem clawSubsystem;

    private static final long drive_time = 3500;
    private static final long wait_time = 6000;
    private static final double motor_power = 0.5;

    @Override
    public void init() {
        this.driveBase = new DriveBaseSubsystem(hardwareMap);
        this.jointSubsystem = new JointSubSystem(hardwareMap);
        this.armSubsystem = new ArmSubsystem(hardwareMap);
        this.clawSubsystem = new ClawSubsystem(hardwareMap);

    }

    @Override
    public void start() {
        new EnableJointCommand(this.jointSubsystem).schedule();

        new CloseClawCommand(this.clawSubsystem).schedule();


        new WaitCommand(wait_time/2)
                .andThen(new ParallelCommandGroup(
                        new MoveArmToPointWithPID(this.armSubsystem, ArmSubsystem.ArmPositions.REST),
                        new MoveJointToPosition(this.jointSubsystem, JointSubSystem.JointPositions.REST)
                )).schedule();

        new WaitCommand(wait_time)
                .andThen(new MoveMotorDiveCommand(this.driveBase, drive_time, motor_power, motor_power))
                .andThen(new OpenClawCommand(this.clawSubsystem))
                .andThen(new MoveMotorDiveCommand(this.driveBase, 350, -motor_power, -motor_power))
                .schedule();
        super.start();
    }

    @Override
    public void loop() {
        CommandScheduler.getInstance().run();

    }
}

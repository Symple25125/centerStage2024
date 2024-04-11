package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.commands.arm.ArmByJoystickCommand;
import org.firstinspires.ftc.teamcode.commands.arm.ArmGoToRestPositionCommand;
import org.firstinspires.ftc.teamcode.commands.arm.GoToScorePositionCommand;
import org.firstinspires.ftc.teamcode.commands.arm.HoldArmPositionWithPIDCommand;
import org.firstinspires.ftc.teamcode.commands.arm.MoveArmToPositionCommand;
import org.firstinspires.ftc.teamcode.commands.arm.RobotGoUpCommand;
import org.firstinspires.ftc.teamcode.commands.claw.CloseClawCommand;
import org.firstinspires.ftc.teamcode.commands.claw.OpenClawCommand;
import org.firstinspires.ftc.teamcode.commands.claw.ToggleClawCommand;
import org.firstinspires.ftc.teamcode.commands.drivebase.ArcadeDriveCommand;
import org.firstinspires.ftc.teamcode.commands.drone.LaunchDroneCommand;
import org.firstinspires.ftc.teamcode.commands.joint.DisableJointCommand;
import org.firstinspires.ftc.teamcode.commands.joint.EnableJointCommand;
import org.firstinspires.ftc.teamcode.commands.joint.MoveJointToPosition;
import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ClawSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveBaseSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DroneSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.JointSubsystem;
import org.firstinspires.ftc.teamcode.util.OpModeType;
import org.firstinspires.ftc.teamcode.util.ScorePositions;
import org.firstinspires.ftc.teamcode.util.TeamColor;
import org.firstinspires.ftc.teamcode.vision.TeamPropDetector;

public class RobotController {
    public DriveBaseSubsystem driveBase;
    public ClawSubsystem clawSubsystem;
    public ArmSubsystem armSubsystem;
    public JointSubsystem jointSubsystem;
    public DroneSubsystem droneSubsystem;

    public TeamPropDetector teamPropDetector;

    public final GamepadEx driverController;
    public final GamepadEx actionController;

    private final OpModeType opModeType;
    private final HardwareMap hardwareMap;
    private final MultipleTelemetry telemetry;
    private final TeamColor teamColor;

    public RobotController(OpModeType opModeType, HardwareMap hMap, Telemetry telemetry, Gamepad driverController, Gamepad actionController, TeamColor teamColor) {
        this.opModeType = opModeType;
        this.hardwareMap = hMap;
        this.teamColor = teamColor;
        this.telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        this.driverController = new GamepadEx(driverController);
        this.actionController = new GamepadEx(actionController);

        FtcDashboard.getInstance().stopCameraStream();
    }

    public void init() {
        initOpMode();
    }

    public void sympleStart() {
        enableJointServo();
    }

    public void initOpMode() {
        CommandScheduler.getInstance().reset();

        switch (this.opModeType) {
            case TeleOP: {
                initTeleOpMode();
                break;
            }
            case Auto: {
                initAutoOpMode();
                break;
            }
            case Testing: {
                initTestingOpMode();
                break;
            }
        }
    }

    public void initTeleOpMode() {
        initDriveBase();
        initArm();
        initJoint();
        initClaw();
        initDrone();

        initDriverButtons();
        initActionButtons();
    }

    public void initAutoOpMode() {
        initDriveBase();
        initArm();
        initJoint();
        initClaw();
        initTeamPropDetector();
    }

    public void initTestingOpMode() {
        initDriveBase();
        initArm();
        initJoint();
        initClaw();
        initDrone();
//
        initTestingButtons();
//        initDriverButtons();
//        initActionButtons();
    }

    public void initDriveBase() {
        this.driveBase = new DriveBaseSubsystem(hardwareMap);
        this.driveBase.setDefaultCommand(new ArcadeDriveCommand(this.driveBase, this.driverController));
    }

    public void initArm() {
        this.armSubsystem = new ArmSubsystem(hardwareMap);
        this.armSubsystem.setDefaultCommand(new HoldArmPositionWithPIDCommand(this.armSubsystem));
    }

    public void initJoint() {
        this.jointSubsystem = new JointSubsystem(hardwareMap);
    }

    public void enableJointServo() {
        new EnableJointCommand(this.jointSubsystem);
    }

    public void initClaw() {
        this.clawSubsystem = new ClawSubsystem(hardwareMap);
    }

    public void initDrone() {
        this.droneSubsystem = new DroneSubsystem(hardwareMap);
    }

    public void initTeamPropDetector() {
        this.teamPropDetector = new TeamPropDetector(hardwareMap, teamColor, telemetry);
    }

    public void initDriverButtons() {
        this.driverController.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .toggleWhenPressed(
                        new InstantCommand(() -> this.driveBase.setInverted(true)),
                        new InstantCommand(() -> this.driveBase.setInverted(false))
                );

        this.driverController.getGamepadButton(GamepadKeys.Button.Y)
                .whenPressed(
                        new LaunchDroneCommand(this.droneSubsystem)
                );
    }

    public void initActionButtons() {
        this.actionController.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whenPressed(
                        new GoToScorePositionCommand(this.clawSubsystem, this.armSubsystem, this.jointSubsystem, ScorePositions.LOWER)
                );

        this.actionController.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whenPressed(
                        new GoToScorePositionCommand(this.clawSubsystem, this.armSubsystem, this.jointSubsystem, ScorePositions.UPPER)
                );

        this.actionController.getGamepadButton(GamepadKeys.Button.A)
                .whenPressed(new ToggleClawCommand(this.clawSubsystem));

        this.actionController.getGamepadButton(GamepadKeys.Button.Y)
                .whenPressed(
                        new ParallelCommandGroup(
                                new MoveArmToPositionCommand(this.armSubsystem, ArmSubsystem.ArmPositions.HOOK),
                                new MoveJointToPosition(this.jointSubsystem, JointSubsystem.JointPositions.HOOK)
                        )
                );


        this.actionController.getGamepadButton(GamepadKeys.Button.X)
                .whenPressed(
                        new ParallelCommandGroup(
                                new DisableJointCommand(this.jointSubsystem),
                                new RobotGoUpCommand(this.armSubsystem)
                        )
                );


        new Trigger(() -> this.actionController.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.5f)
                .whenActive(
                        new SequentialCommandGroup(
                                new CloseClawCommand(this.clawSubsystem),
                                new MoveJointToPosition(this.jointSubsystem, JointSubsystem.JointPositions.PICKUP),
                                new MoveArmToPositionCommand(this.armSubsystem, ArmSubsystem.ArmPositions.PICKUP),
                                new WaitUntilCommand(() -> this.armSubsystem.isAtPosition(ArmSubsystem.ArmPositions.PICKUP, 5f)),
                                new OpenClawCommand(this.clawSubsystem)
                        )
                )
                .whenInactive(new SequentialCommandGroup(
                        new CloseClawCommand(this.clawSubsystem),
                        new WaitCommand(500),
                        new MoveJointToPosition(this.jointSubsystem, JointSubsystem.JointPositions.REST),
                        new ArmGoToRestPositionCommand(this.armSubsystem)
                ));
        new Trigger(() -> Math.abs(this.actionController.getRightY()) > ArmSubsystem.JOYSTICK_DEAD_AREA).whenActive(new ArmByJoystickCommand(this.armSubsystem, this.actionController));
    }

    public void initTestingButtons() {
        this.driverController.getGamepadButton(GamepadKeys.Button.B)
                .whenPressed(new MoveJointToPosition(this.jointSubsystem, JointSubsystem.JointPositions.ZERO));
    }

    public TeamColor getTeamColor() {
        return teamColor;
    }

    public MultipleTelemetry getTelemetry() {
        return telemetry;
    }
}

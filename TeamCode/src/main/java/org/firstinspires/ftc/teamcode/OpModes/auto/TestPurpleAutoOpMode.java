package org.firstinspires.ftc.teamcode.OpModes.auto;

import android.database.DatabaseErrorHandler;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.commands.arm.InitArmForAutoCommand;
import org.firstinspires.ftc.teamcode.commands.drivebase.DriveDistanceDriveCommand;
import org.firstinspires.ftc.teamcode.commands.joint.EnableJointCommand;
import org.firstinspires.ftc.teamcode.paths.ClosePurplePixelPath;
import org.firstinspires.ftc.teamcode.paths.Paths;
import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ClawSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveBaseSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.JointSubSystem;
import org.firstinspires.ftc.teamcode.util.AutoOpMode;
import org.firstinspires.ftc.teamcode.util.DetectionSide;
import org.firstinspires.ftc.teamcode.util.TeamColor;
import org.firstinspires.ftc.teamcode.vision.TeamPropDetector;

@Config
@Autonomous(name = "Test Purple Auto", group = "test")
public class TestPurpleAutoOpMode extends AutoOpMode {
    public static TeamColor TEAM_COLOR = TeamColor.RED;

    private TeamPropDetector teamPropDetector;

    private DriveBaseSubsystem driveBase;
    private JointSubSystem jointSubsystem;
    private ArmSubsystem armSubsystem;
    private ClawSubsystem clawSubsystem;

    private DetectionSide purplePixelSide;

    @Override
    public void initialize() {
        this.driveBase = new DriveBaseSubsystem(hardwareMap);
        this.jointSubsystem = new JointSubSystem(hardwareMap);
        this.armSubsystem = new ArmSubsystem(hardwareMap);
        this.clawSubsystem = new ClawSubsystem(hardwareMap);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        this.teamPropDetector = new TeamPropDetector(hardwareMap, TEAM_COLOR, (MultipleTelemetry) telemetry);
    }

    @Override
    public void runInit() {
        if (teamPropDetector.getTeamPropSide() != null) {
            purplePixelSide = teamPropDetector.getTeamPropSide();
        }

        logData();
    }

    @Override
    public void run() {
        super.run();

        logData();
    }

    @Override
    public void sympleStart() {
        SequentialCommandGroup pathCommands = Paths.generatePath(TEAM_COLOR, purplePixelSide, this.driveBase);

        new InitArmForAutoCommand(this.jointSubsystem, this.armSubsystem, this.clawSubsystem)
                .andThen(pathCommands)
                .schedule();
    }


    private void logData() {
        telemetry.addData("ARM ANGLE", String.valueOf(this.armSubsystem.getCurrentPos()));
//        telemetry.addData("Arm Command", String.valueOf(this.armSubsystem.getCurrentCommand().getName()));

        teamPropDetector.telemetry();

        telemetry.update();
    }
}

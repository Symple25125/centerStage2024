package org.firstinspires.ftc.teamcode.OpModes.auto;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.RobotController;
import org.firstinspires.ftc.teamcode.commands.arm.InitArmForAutoCommand;
import org.firstinspires.ftc.teamcode.paths.Paths;
import org.firstinspires.ftc.teamcode.util.AutoOpMode;
import org.firstinspires.ftc.teamcode.util.DetectionSide;
import org.firstinspires.ftc.teamcode.util.OpModeType;
import org.firstinspires.ftc.teamcode.util.TeamColor;

@Config
@Autonomous(name = "Test Purple Auto", group = "test")
public class TestPurpleAutoOpMode extends AutoOpMode {
    public static TeamColor TEAM_COLOR = TeamColor.RED;

    private RobotController robotController;

    private DetectionSide purplePixelSide;

    @Override
    public void initialize() {
        this.robotController = new RobotController(OpModeType.Auto, hardwareMap, telemetry, gamepad1, gamepad2, TEAM_COLOR);
    }

    @Override
    public void runInit() {
        if (this.robotController.teamPropDetector.getTeamPropSide() != null) {
            purplePixelSide = this.robotController.teamPropDetector.getTeamPropSide();
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
        SequentialCommandGroup pathCommands = Paths.generatePath(this.robotController, this.purplePixelSide);

        new InitArmForAutoCommand(this.robotController.jointSubsystem, this.robotController.armSubsystem, this.robotController.clawSubsystem)
                .andThen(pathCommands)
                .schedule();
    }


    private void logData() {
        telemetry.addData("ARM ANGLE", String.valueOf(this.robotController.armSubsystem.getCurrentPos()));
//        telemetry.addData("Arm Command", String.valueOf(this.armSubsystem.getCurrentCommand().getName()));

        telemetry.update();
    }
}

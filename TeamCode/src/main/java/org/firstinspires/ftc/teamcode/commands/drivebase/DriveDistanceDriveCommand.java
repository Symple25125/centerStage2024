package org.firstinspires.ftc.teamcode.commands.drivebase;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.controller.PController;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.DriveBaseSubsystem;

@Config
public class DriveDistanceDriveCommand extends CommandBase {
    private DriveBaseSubsystem driveBaseSubsystem;
    private PController pController;

    public static double Kp = 0.5;
    private static double Ks = 0.07;

    private double STARTING_ENCODER_POS;
    private final double finalPos;

    public DriveDistanceDriveCommand(DriveBaseSubsystem driveBaseSubsystem, double cmToMove) {
        addRequirements(driveBaseSubsystem);
        this.driveBaseSubsystem = driveBaseSubsystem;
        this.finalPos = cmToMove;

        this.pController = new PController(Kp);
        this.pController.setTolerance(1);
    }

    @Override
    public void initialize() {
        super.initialize();
        this.STARTING_ENCODER_POS = this.driveBaseSubsystem.getMotorEncoderPos();
        this.pController.setSetPoint(this.finalPos);
    }

    @Override
    public void execute() {
        super.execute();

        double driveDistance = this.driveBaseSubsystem.encoderTicksToCM(this.driveBaseSubsystem.getMotorEncoderPos() - this.STARTING_ENCODER_POS);

        double power = this.pController.calculate(driveDistance);
        power += Math.signum(power) * Ks;

        FtcDashboard.getInstance().getTelemetry().addData("power", power);
        FtcDashboard.getInstance().getTelemetry().addData("dist", driveDistance);
        FtcDashboard.getInstance().getTelemetry().addData("rel motor encoder", this.driveBaseSubsystem.getMotorEncoderPos() - this.STARTING_ENCODER_POS);
        FtcDashboard.getInstance().getTelemetry().addData("motor encoder", this.driveBaseSubsystem.getMotorEncoderPos());
        FtcDashboard.getInstance().getTelemetry().update();

        this.driveBaseSubsystem.moveMotors(power, power);
    }

    @Override
    public void end(boolean interrupted) {
        this.driveBaseSubsystem.moveMotors(0, 0);
    }

    @Override
    public boolean isFinished() {
        return this.pController.atSetPoint();
    }
}

package org.firstinspires.ftc.teamcode.util;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.qualcomm.robotcore.robocol.Command;

public abstract class SympleCommandOpMode extends CommandOpMode {
    public void sympleStart() { }

    @Override
    public void runOpMode() throws InterruptedException {
        initialize();

        waitForStart();

        sympleStart();

        // run the scheduler
        while (!isStopRequested() && opModeIsActive()) {
            run();
        }

        reset();
    }
}

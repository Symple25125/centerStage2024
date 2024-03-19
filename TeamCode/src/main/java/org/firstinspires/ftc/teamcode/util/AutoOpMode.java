package org.firstinspires.ftc.teamcode.util;

import com.arcrobotics.ftclib.command.CommandOpMode;

public abstract class AutoOpMode extends CommandOpMode {

    abstract public void sympleStart();

    public void runInit() { }

    @Override
    public void runOpMode() throws InterruptedException {
        // idle
        this.initialize();

        // run this when in init mod
        while (this.opModeInInit() && !this.isStopRequested()) {
            this.runInit();
        }

        this.waitForStart();

        // init
        this.sympleStart();


        //run mode
        while(!this.isStopRequested() && this.opModeIsActive()) {
            this.run();
        }

        this.reset();
    }
}

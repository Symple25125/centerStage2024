package org.firstinspires.ftc.teamcode.vision;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.util.Cameras;
import org.firstinspires.ftc.teamcode.util.DetectionSide;
import org.firstinspires.ftc.teamcode.util.TeamColor;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

public class TeamPropDetector extends OpenCvPipeline {
    public OpenCvCamera webcam;

    private DetectionSide teamPropSide;

    private final TeamColor teamColor;
    private final MultipleTelemetry telemetry;

    private final Rect centerRectangle;
    private final Rect farRectangle;

    private double farMean;
    private double centerMean;

    private final Mat YCrCb = new Mat();
    private final Mat blurredImage = new Mat();
    private final Mat currentColorChannel = new Mat();

    public TeamPropDetector(HardwareMap hardwareMap, TeamColor teamColor, MultipleTelemetry telemetry) {
        this.telemetry = telemetry;
        this.teamColor = teamColor;

//        if(teamColor == TeamColor.RED) {
            centerRectangle = new Rect(310,  279,270,200);
            farRectangle = new Rect(0, 279, 160, 200); //LEFT (RED)
//        }
//        else {
//            centerRectangle = new Rect(0,  279,300,200);
//            farRectangle = new Rect(445, 279, 160, 200); //RIGHT (BLUE)
//        }

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, Cameras.WEBCAM.id), cameraMonitorViewId);

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(640, 480, OpenCvCameraRotation.UPSIDE_DOWN);
            }

            @Override
            public void onError(int errorCode) {
            }
        });

        webcam.setPipeline(this);
    }

    @Override
    public Mat processFrame(Mat frame) {
        // blur image
        Imgproc.GaussianBlur(frame, blurredImage, new Size(15,15), 0.0);

        // change image format in order to separate the blue and red channels
        Imgproc.cvtColor(blurredImage, YCrCb, Imgproc.COLOR_RGB2YCrCb);

        // choose wanted channel
        switch (teamColor){
            case RED: {
                Core.extractChannel(YCrCb, currentColorChannel, 1);
                break;
            }

            case BLUE: {
                Core.extractChannel(YCrCb, currentColorChannel, 2);
                break;
            }
        }

        // Submats from the current channel
        Mat farMat = currentColorChannel.submat(farRectangle);
        Mat centerMat = currentColorChannel.submat(centerRectangle);


        // get the area specific means
        farMean = Core.mean(farMat).val[0];
        centerMean = Core.mean(centerMat).val[0];

        double centerTolerance = (teamColor == TeamColor.BLUE) ? Tolerance.BLUE_CENTER.tolerance : Tolerance.RED_CENTER.tolerance;
        double farTolerance = (teamColor == TeamColor.BLUE) ? Tolerance.BLUE_FAR.tolerance : Tolerance.RED_FAR.tolerance;

        // choose the most prominent side
        DetectionSide tempResult = DetectionSide.CLOSE;

        if(centerMean > centerTolerance) {
            tempResult = DetectionSide.CENTER;
        } else if(farMean > farTolerance) {
            tempResult = DetectionSide.FAR;
        }

        teamPropSide = tempResult;

        // visual indicator
        Imgproc.rectangle(frame, farRectangle, new Scalar(255, 0, 0), 5);
        Imgproc.rectangle(frame, centerRectangle, new Scalar(0, 0, 255), 5);

        //Camera Stream output
        return frame;
    }

    public DetectionSide getTeamPropSide() {
        if (teamPropSide == null) return null;
        return teamColor == TeamColor.RED ? teamPropSide : teamPropSide.mirror();
    }

    public void telemetry() {
        telemetry.addData("teamPropSide", getTeamPropSide());
        telemetry.addData("farMean", farMean);
        telemetry.addData("centerMean", centerMean);
    }



    public enum Tolerance {
        RED_CENTER(131),
        RED_FAR(134),
        BLUE_CENTER(129),
        BLUE_FAR(129);

        final double tolerance;

        Tolerance(double tolerance) {
            this.tolerance = tolerance;
        }
    }
}

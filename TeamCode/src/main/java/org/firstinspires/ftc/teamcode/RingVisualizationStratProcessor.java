package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.RingDeterminationPipeline;
import org.opencv.core.Mat;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

import java.util.Arrays;

public class RingVisualizationStratProcessor {

    OpenCvInternalCamera phoneCam;
    RingDeterminationPipeline pipeline;
    public LinearOpMode teleOp;
    RingDeterminationPipeline.RingPosition positionRing;
    OpenCvCamera webcam;

    public RingVisualizationStratProcessor(LinearOpMode teleOp) {
        this.teleOp=teleOp;
    }


    public long getThresholdValue(Mat img) {
        return 0;
    }

    public void init() {
        int cameraMonitorViewId = teleOp.hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", teleOp.hardwareMap.appContext.getPackageName());
        pipeline = new RingDeterminationPipeline();
        webcam = OpenCvCameraFactory.getInstance().createWebcam(teleOp.hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        webcam.setPipeline(new RingDeterminationPipeline());
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }
        });


//        int cameraMonitorViewId = getHardwareMap().appContext.getResources().getIdentifier("cameraMonitorViewId", "id", getHardwareMap().appContext.getPackageName());
//        phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
//        pipeline = new RingDeterminationPipeline();
//        phoneCam.setPipeline(pipeline);
//
//        phoneCam.setViewportRenderingPolicy(OpenCvCamera.ViewportRenderingPolicy.OPTIMIZE_VIEW);
//
//        phoneCam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
//            @Override
//            public void onOpened() {
//                phoneCam.startStreaming(320, 240, OpenCvCameraRotation.SIDEWAYS_LEFT);
//            }
//        });
        teleOp.telemetry.addData("ringPosition: ", getRingPosition());
        teleOp.telemetry.update();
    }

    public void process() {
        long startTime = System.currentTimeMillis();
        long currentTime = System.currentTimeMillis();
        while (currentTime-startTime<5000&&!teleOp.opModeIsActive()) {
            teleOp.telemetry.addData("Analysis_threshold                  ", Arrays.deepToString(pipeline.getThresholdDataAnalysis()));
            teleOp.telemetry.addData("Analysis_thresholdPercentage                  ", Arrays.deepToString(pipeline.getThresholdPercentageDataAnalysis()));
            teleOp.telemetry.addData("Position", pipeline.getPosition());
            teleOp.telemetry.update();
            currentTime = System.currentTimeMillis();
            setRingPosition(pipeline.position);
        }
        setRingPosition(pipeline.position);
    }

    public void processFor(long time) {
        long startTime = System.currentTimeMillis();
        long currentTime = System.currentTimeMillis();
        while (currentTime-startTime<time&&!teleOp.opModeIsActive()) {
            teleOp.telemetry.addData("Analysis_threshold                  ", Arrays.deepToString(pipeline.getThresholdDataAnalysis()));
            teleOp.telemetry.addData("Analysis_thresholdPercentage                  ", Arrays.deepToString(pipeline.getThresholdPercentageDataAnalysis()));
            teleOp.telemetry.addData("Position", pipeline.getPosition());
            teleOp.telemetry.update();
            currentTime = System.currentTimeMillis();
            setRingPosition(pipeline.position);
        }
        setRingPosition(pipeline.position);
    }

    public void setRingPosition(RingDeterminationPipeline.RingPosition position) {
        positionRing=position;
    }

    public RingDeterminationPipeline.RingPosition getRingPosition() {
        return positionRing;
    }

}

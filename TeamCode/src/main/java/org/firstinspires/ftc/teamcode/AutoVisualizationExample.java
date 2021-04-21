package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
@Autonomous(name="THISBETTERWORK", group="FinalAuto")
public class AutoVisualizationExample extends LinearOpMode {

    RingVisualizationStratProcessor processRings= new RingVisualizationStratProcessor(this);

    long time=3000;
    int up,right;

    @Override
    public void runOpMode() throws InterruptedException {
        processRings.init();
        //hardwareMap setUp
        //the random intialization of motors and servos
        processRings.process();
        telemetry.addData("ringPosition: ", processRings.getRingPosition());
        waitForStart();
//        if(opModeIsActive()){
            //if(processRings.getRingPosition() == RingDeterminationPipeline.RingPosition.BLANK || processRings.getRingPosition() == RingDeterminationPipeline.RingPosition.NONE) {
                //processRings.processFor(time);//time can be as long or short as neccessary
            //}
            //whatever you want here
        while (opModeIsActive()){
            processRings.processFor(time);
            if (processRings.getRingPosition() == RingDeterminationPipeline.RingPosition.FOUR) {
                up = 120;//10
                right = 36;//3
            } else if (processRings.getRingPosition() == RingDeterminationPipeline.RingPosition.ONE) {
                up = 96;//8
                right = 12;//1
            } else {
                up = 72;//6
                right = 36;//3
            }
            telemetry.addData("position:", processRings.getRingPosition());
            telemetry.addData("up:", up);
            telemetry.update();
        }
        }

    }


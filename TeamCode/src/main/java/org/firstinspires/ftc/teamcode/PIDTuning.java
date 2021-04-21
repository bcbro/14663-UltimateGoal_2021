package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;


@TeleOp(name="PIDTest", group="Iterative Opmode")

public class PIDTuning extends OpMode
{
    private ElapsedTime runtime = new ElapsedTime();
    private Servo wobbleGoalTres;

    private DcMotorEx outake2 = null;

    FtcDashboard dashboard = FtcDashboard.getInstance();


    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        outake2 = hardwareMap.get(DcMotorEx.class, "outake2");
        wobbleGoalTres = hardwareMap.get(Servo.class, "wobbleGoalTres");

        outake2.setDirection(DcMotorEx.Direction.FORWARD);

//        MotorConfigurationType GoBILDA5202Series = outake2.getMotorType().clone();
//        GoBILDA5202Series.setAchieveableMaxRPMFraction(0.5);
        outake2.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(50, 0, 3, 11));

        telemetry.addData("Status", "Initialized");

        wobbleGoalTres.setPosition(0.9);

    }


    @Override
    public void loop() {

        if(gamepad2.x) {
            outake2.setVelocity(1320);
        }

        if(gamepad2.y) {
            outake2.setVelocity(0);
        }


        if(gamepad2.a) {
            wobbleGoalTres.setPosition(0.5);

        }
        if(gamepad2.b) {
            wobbleGoalTres.setPosition(0.9);
        }

        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Velocity", ": " + outake2.getVelocity());
        telemetry.addData("PIDFCoeffs", ":" + outake2.getPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER));
        telemetry.update();
    }

    @Override
    public void stop() {
    }

}

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
//import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
//import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp(name="TeleOp Final Killabytez Solo︎︎︎", group="Iterative Opmode")

public class DriverSolo extends OpMode {
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotorEx spinner;
    private DcMotorEx spinner2;
    private Servo wobbleGoalTres;
    private Servo wobbleGoalUno;
    private DcMotorEx frontleft = null;
    private DcMotorEx frontright = null;
    private DcMotorEx backleft = null;
    private DcMotorEx backright = null;
    private DcMotorEx outake2 = null;
    // private DcMotorEx intake = null;
    private DcMotorEx arm = null;

    boolean autoStop=false;
    //private static final double velocity = 2000;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        frontleft = hardwareMap.get(DcMotorEx.class, "frontleft");
        frontright = hardwareMap.get(DcMotorEx.class, "frontright");
        backleft = hardwareMap.get(DcMotorEx.class, "backleft");
        backright = hardwareMap.get(DcMotorEx.class, "backright");
        outake2 = hardwareMap.get(DcMotorEx.class, "outake2");
        //too1 = hardwareMap.get(DcMotor.class, "too1");
        // intake = hardwareMap.get(DcMotorEx.class, "intake");
        wobbleGoalTres = hardwareMap.get(Servo.class, "wobbleGoalTres");
        arm = hardwareMap.get(DcMotorEx.class, "arm");
        spinner = hardwareMap.get(DcMotorEx.class, "spinner");
        spinner2 = hardwareMap.get(DcMotorEx.class, "spinner2");
        wobbleGoalUno = hardwareMap.get(Servo.class, "wobbleGoalUno");

        //outake1.setDirection(DcMotor.Direction.REVERSE);
        outake2.setDirection(DcMotorEx.Direction.FORWARD);
        //too1.setDirection(DcMotor.Direction.REVERSE);
        //  intake.setDirection(DcMotorEx.Direction.REVERSE);
        arm.setDirection(DcMotorEx.Direction.FORWARD);

        frontleft.setDirection(DcMotorEx.Direction.REVERSE);
        frontright.setDirection(DcMotorEx.Direction.FORWARD);
        backleft.setDirection(DcMotorEx.Direction.FORWARD);
        backright.setDirection(DcMotorEx.Direction.REVERSE);

//        frontleft.setDirection(DcMotor.Direction.REVERSE);
//        frontright.setDirection(DcMotor.Direction.REVERSE);
//        backleft.setDirection(DcMotor.Direction.REVERSE);
//        backright.setDirection(DcMotor.Direction.FORWARD);

        outake2.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(50, 0, 3, 11));
        //outake2.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(50, 0, 3, 11));


        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");

        outake2.setVelocity(0);

        wobbleGoalTres.setPosition(0.95);
        wobbleGoalUno.setPosition(0.0);

    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        // Setup a variable for each drive wheel to save power level for telemetry
//        outake2.setMotorEnable();
//        outake2.setVelocity(velocity);
        //.setPower(0.63569);
        if(!autoStop) {

            double sped = gamepad1.right_trigger;

            //too1.setPower(o/2);
            //intake.setPower(in);
            spinner.setPower(-1);
            spinner2.setPower(-1);
            arm.setPower(sped);
            outake2.setVelocity(1320);


            if (gamepad1.a) {
                wobbleGoalTres.setPosition(0.65);

            }
            if (gamepad1.b) {
                wobbleGoalTres.setPosition(0.95);
            }

            if (gamepad1.x) {
                wobbleGoalUno.setPosition(0.0);
            }

            if (gamepad1.y) {
                wobbleGoalUno.setPosition(0.8);
            }


            float y;
            float x;
            float z;
            y = gamepad1.left_stick_y;
            x = gamepad1.left_stick_x;
            z = gamepad1.right_stick_y;
            frontright.setPower((y + x - z));
            frontleft.setPower((y - x + z));
            backright.setPower((y - x - z));
            backleft.setPower((y + x + z));
        }

        if(gamepad1.left_bumper) autoStop=!autoStop;
        // Send calculated powerea to wheels
        // Show the elapsed game time and wheel power.

        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Velocity", ": " + outake2.getVelocity());
        telemetry.addData("Power", ": " + outake2.getPower());
        telemetry.addData("PIDFCoeffs", ":" + outake2.getPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER));
        telemetry.update();

    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }
}

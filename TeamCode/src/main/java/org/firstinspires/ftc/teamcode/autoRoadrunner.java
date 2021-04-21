package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.StandardTrackingWheelLocalizer;

@TeleOp(name="autoRoadRun", group="auto")
public class autoRoadrunner extends LinearOpMode {
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    double rotationCoefficient=0.25;
    private DcMotorEx spinner;
    private DcMotorEx spinner2;
    private Servo wobbleGoalTres;
    private Servo wobbleGoalUno;
    public Servo downSlapper;
    private DcMotorEx frontleft = null;
    private DcMotorEx frontright = null;
    private DcMotorEx backleft = null;
    private DcMotorEx backright = null;
    private DcMotorEx outake2 = null;
    public SampleMecanumDrive drive;
    double meccyBias = 0.9; //adjust strafing
    Double width = 18.0; //inches
    Integer cpr = 28; //counts per rotation
    Integer gearratio = 20;
    Double diameter = 4.0;
    Pose2d highGoalPose=new Pose2d();
    Double cpi = (cpr * gearratio)/(Math.PI * diameter);
    BNO055IMU imu;
    // private DcMotorEx intake = null;
    private DcMotorEx arm = null;

//    double targetHeading=0; double frontLeftTarget; double frontRightTarget; double backLeftTarget; double backRightTarget; double speed;

    boolean autoHighGoalPositionSet=false;

    boolean autoStop=false;

    long startTime=0;
    private boolean exit=false;


    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Status", "Initialized");
        initGyro();
        drive=new SampleMecanumDrive(hardwareMap);

//        StandardTrackingWheelLocalizer myLocalizer = new StandardTrackingWheelLocalizer(hardwareMap);

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        downSlapper=hardwareMap.get(Servo.class, "downSlapper");
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
        downSlapper.setPosition(0.9);
        while(opModeIsActive() && !drive.isBusy() && !isStopRequested()) {
//            myLocalizer.update();
            drive.update();
            drive.updatePoseEstimate();

            double in = gamepad2.left_stick_y;
            double o = gamepad2.right_stick_y;
            double sped = gamepad2.right_stick_x;

            spinner.setPower(-in);
            spinner2.setPower(-in);
            arm.setPower(3*sped/4);

            if(gamepad1.x) {
                if(!autoHighGoalPositionSet) {
                    autoHighGoalPositionSet=true;
                    startTime=System.currentTimeMillis();
                    highGoalPose=drive.getPoseEstimate();
                }
                if(autoHighGoalPositionSet && System.currentTimeMillis()-startTime>2000) {
                    Trajectory trajectoryHighGoal=drive.trajectoryBuilder(drive.getPoseEstimate(), true).splineToSplineHeading(highGoalPose, 180.0).build();//.lineTo(new Vector2d(highGoalPose.getX(), highGoalPose.getY())).build();

                    if(opModeIsActive()) drive.followTrajectoryAsync(trajectoryHighGoal);
                    startTime=System.currentTimeMillis();
                }
            }

            if (gamepad2.a) {
                wobbleGoalTres.setPosition(0.5);

            }
            if (gamepad2.b) {
                wobbleGoalTres.setPosition(0.95);
            }

            if (gamepad2.x) {
                outake2.setVelocity(1320);
            }

            if (gamepad2.y) {
                outake2.setVelocity(0);
            }

            if (gamepad2.right_bumper) {
                wobbleGoalUno.setPosition(0.0);
            }

            if (gamepad2.left_bumper) {
                wobbleGoalUno.setPosition(0.95);
            }

            if (gamepad2.dpad_down) downSlapper.setPosition(0.5);
            if (gamepad2.dpad_up) downSlapper.setPosition(0.9);

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

           /* Pose2d poseEstimate = drive.getPoseEstimate();

            Vector2d input = new Vector2d(
                    -gamepad1.left_stick_y,
                    -gamepad1.left_stick_x
            ).rotated(-poseEstimate.getHeading());

            drive.setWeightedDrivePower(
                    new Pose2d(
                            input.getX(),
                            input.getY(),
                            -gamepad1.right_stick_x
                    )
            );*/

            // Send calculated powerea to wheels
            // Show the elapsed game time and wheel power.

            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Velocity", ": " + outake2.getVelocity());
            telemetry.addData("Power", ": " + outake2.getPower());
            telemetry.addData("PIDFCoeffs", ":" + outake2.getPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER));
            telemetry.addData("HighGoalPose: ", "x="+highGoalPose.component1()+", y="+highGoalPose.component2()+", z="+highGoalPose.component3());
            telemetry.addData("CurrentPose: ", "x="+drive.getPoseEstimate().component1()+", y="+drive.getPoseEstimate().component2()+", z="+drive.getPoseEstimate().component3());
            telemetry.update();
        }
    }

    private void runToHighGoalPosition(Pose2d highGoalPose) {

    }
    public void initGyro(){
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
    }
}

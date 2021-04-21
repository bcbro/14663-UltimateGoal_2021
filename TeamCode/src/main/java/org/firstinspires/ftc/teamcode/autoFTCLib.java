//package org.firstinspires.ftc.teamcode;
//
//import com.arcrobotics.ftclib.command.MecanumControllerCommand;
//import com.arcrobotics.ftclib.drivebase.MecanumDrive;
//import com.arcrobotics.ftclib.geometry.Translation2d;
//import com.arcrobotics.ftclib.hardware.motors.Motor;
//import com.arcrobotics.ftclib.kinematics.wpilibkinematics.MecanumDriveKinematics;
//import com.arcrobotics.ftclib.kinematics.wpilibkinematics.MecanumDriveOdometry;
//import com.arcrobotics.ftclib.kinematics.wpilibkinematics.MecanumDriveWheelSpeeds;
//import com.arcrobotics.ftclib.trajectory.Trajectory;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.hardware.DcMotorEx;
//
//@Autonomous(name="autoFTCLib", group="auto")
//public class autoFTCLib extends LinearOpMode{
//
//    Motor frontLeft;
//    Motor frontRight;
//    Motor backLeft;
//    Motor backRight;
//
//    MecanumControllerCommand mecanumDrive;
//    Trajectory trajectory;
//
//    @Override
//    public void runOpMode() throws InterruptedException {
//        frontLeft = hardwareMap.get(Motor.class,"frontleft");
//        frontRight = hardwareMap.get(Motor.class, "frontright");
//        backLeft = hardwareMap.get(Motor.class, "backleft");
//        backRight = hardwareMap.get(Motor.class, "backright");
////
////        trajectory=new Trajectory()
////        mecanumDrive=new MecanumControllerCommand()
////        mecanumDrive.driveRobotCentric();
//    }
//}

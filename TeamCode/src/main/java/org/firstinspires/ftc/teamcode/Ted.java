package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveBase;
import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveREV;


@Autonomous(group = "drive")
public class Ted extends LinearOpMode {

    Servo grabServo;
    Servo footServoOne;
    Servo footServoTwo;
    DcMotor armMotor;
    SkystoneTracker vue;

    public void runOpMode() throws InterruptedException {
        SampleMecanumDriveBase drive = new SampleMecanumDriveREV(hardwareMap);
        vue = new SkystoneTracker(.25);
        vue.init();
        RobotLog.d("After VF init");
        grabServo = hardwareMap.get(Servo.class, "thehandofnod");
        armMotor = hardwareMap.get(DcMotor.class, "thestrengthofnod");
        footServoOne = hardwareMap.get(Servo.class, "footOfNodOne");
        footServoTwo = hardwareMap.get(Servo.class, "theFootOfNodTwo");
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setTargetPosition(0);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        waitForStart();
        if (isStopRequested()) return;
        drive.setPoseEstimate(new Pose2d(-46, -63, Math.PI / 2));
        drive.followTrajectorySync(
                drive.trajectoryBuilder().forward(48).build());

        while(!isStopRequested()) {
            telemetry.addData("F",  Double.toString(DriveConstants.getMotorVelocityF()));
            telemetry.update();
        }

    }
}
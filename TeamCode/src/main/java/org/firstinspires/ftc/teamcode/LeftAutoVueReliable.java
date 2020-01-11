package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveBase;
import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveREV;


@Autonomous(group = "drive")
public class LeftAutoVueReliable extends LinearOpMode {

    Servo grabServo;
    Servo footServoOne;
    Servo footServoTwo;
    DcMotor armMotor;
    SkystoneTracker vue;
    String path;

    public void runOpMode() throws InterruptedException {
        SampleMecanumDriveBase drive = new SampleMecanumDriveREV(hardwareMap);
        vue = new SkystoneTracker();
        vue.init(hardwareMap);
        RobotLog.d("After VF init");
        grabServo = hardwareMap.get(Servo.class, "thehandofnod");
        armMotor = hardwareMap.get(DcMotor.class, "thestrengthofnod");
        footServoOne = hardwareMap.get(Servo.class,"footOfNodOne");
        footServoTwo = hardwareMap.get(Servo.class,"theFootOfNodTwo" );
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setTargetPosition(0);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();
        if (isStopRequested()) return;
        drive.setPoseEstimate(new Pose2d(-46, -63, 0));
        footServoTwo.setPosition(.9);
        footServoOne.setPosition(.1);
        grabServo.setPosition(.1);
        path=vue.getPath();
        drive.followTrajectorySync(
                drive.trajectoryBuilder().forward(15).build());
        vue.update();
        vue.update();

        if (path=="L") {
            armMotor.setTargetPosition(2800);
            armMotor.setPower(.7);
            drive.followTrajectorySync(drive.trajectoryBuilder().forward(1).build());
            sleep(1500);
            grabServo.setPosition(.5);
            armMotor.setTargetPosition(2400);
            armMotor.setPower(-1);
            drive.followTrajectorySync(
                    drive.trajectoryBuilder().forward(5).build()
            );
            drive.turnSync(Math.PI/2+.1);
            drive.followTrajectorySync(
                    drive.trajectoryBuilder().back(78).build()
            );

        } else {
            drive.followTrajectorySync(
                    drive.trajectoryBuilder().strafeLeft(8).build()
            );
            if (path=="C") {
                armMotor.setTargetPosition(2800);
                armMotor.setPower(.7);
                sleep(1500);
                drive.followTrajectorySync(drive.trajectoryBuilder().forward(1).build());
                grabServo.setPosition(.5);
                armMotor.setTargetPosition(2400);
                armMotor.setPower(-1);
                drive.turnSync(Math.PI/2+.1);
                drive.followTrajectorySync(
                        drive.trajectoryBuilder().back(84).build()
                );
            } else {
                drive.followTrajectorySync(
                        drive.trajectoryBuilder().strafeLeft(8).build()
                );
                armMotor.setTargetPosition(2800);
                armMotor.setPower(.7);
                sleep(1500);
                drive.followTrajectorySync(drive.trajectoryBuilder().forward(1).build());
                grabServo.setPosition(.5);
                armMotor.setTargetPosition(2400);
                armMotor.setPower(-1);
                drive.turnSync(Math.PI/2+.1);
                drive.followTrajectorySync(
                        drive.trajectoryBuilder().back(90).build()
                );


            }
            drive.turnSync(-Math.PI/2);
            drive.followTrajectorySync(
                    drive.trajectoryBuilder().forward(15).build()
            );



            footServoOne.setPosition(.9);
            footServoTwo.setPosition(.1);
            sleep(1500);
            drive.followTrajectorySync(drive.trajectoryBuilder().back(29).build());
            grabServo.setPosition(.2);
            // drive.turnSync(-Math.PI/2);
            drive.turnSync(-Math.PI);
            drive.followTrajectorySync(drive.trajectoryBuilder().forward(29).build());
            //drive.followTrajectorySync(drive.trajectoryBuilder().strafeRight(24).build());
            footServoTwo.setPosition(.9);
            footServoOne.setPosition(.1);
            drive.followTrajectorySync(
                    drive.trajectoryBuilder().back(35).build()
            );

        }
    }
}
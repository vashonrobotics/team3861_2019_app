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
public class LeftAutoVue extends LinearOpMode {

    Servo grabServo;
    Servo footServoOne;
    Servo footServoTwo;
    DcMotor armMotor;
    SkystoneTracker vue;

    public void runOpMode() throws InterruptedException {
        SampleMecanumDriveBase drive = new SampleMecanumDriveREV(hardwareMap);
        vue = new SkystoneTracker();
        //vue.init();
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
        drive.setPoseEstimate(new Pose2d(-46, -63, Math.PI / 2));
       // vue.update();
        sleep(250);
        grabServo.setPosition(.1);
       // vue.update();
        drive.followTrajectorySync(
                drive.trajectoryBuilder().forward(9).build());
               sleep(1500);
        drive.followTrajectorySync(
                drive.trajectoryBuilder().forward(3).build());

      //  vue.update();
        sleep(250);
      //  vue.update();

            if (vue.isVisible()) {
                armMotor.setTargetPosition(2800);
                armMotor.setPower(.7);
                drive.followTrajectorySync(drive.trajectoryBuilder().forward(3).build());
                sleep(1000);
                grabServo.setPosition(.5);
                drive.followTrajectorySync(
                        drive.trajectoryBuilder().back(3).build()
                );
                armMotor.setTargetPosition(2200);
                armMotor.setPower(-1);
                sleep(2000);
                drive.turnSync(Math.PI/2);
                drive.followTrajectorySync(
                        drive.trajectoryBuilder().back(76).build()
                );
                drive.turnSync(-Math.PI/2);

            } else {
                drive.followTrajectorySync(
                        drive.trajectoryBuilder().strafeLeft(8).build()
                );
                sleep(700);
                vue.update();
                sleep(500);
                vue.update();
                if (vue.isVisible()) {
                    armMotor.setTargetPosition(2800);
                    armMotor.setPower(.7);
                    sleep(3500);
                    drive.followTrajectorySync(drive.trajectoryBuilder().forward(2).build());
                    grabServo.setPosition(.5);
                    drive.followTrajectorySync(
                            drive.trajectoryBuilder().back(3).build()
                    );
                    armMotor.setTargetPosition(2200);
                    armMotor.setPower(-1);
                    sleep(2000);
                    drive.turnSync(Math.PI/2);
                    drive.followTrajectorySync(
                            drive.trajectoryBuilder().back(84).build()
                    );
                    drive.turnSync(-Math.PI/2);
                } else {
                    drive.followTrajectorySync(
                            drive.trajectoryBuilder().strafeLeft(8).build()
                    );
                    armMotor.setTargetPosition(2800);
                    armMotor.setPower(.7);
                    sleep(3500);
                    drive.followTrajectorySync(drive.trajectoryBuilder().forward(2).build());
                    grabServo.setPosition(.5);
                    drive.followTrajectorySync(
                            drive.trajectoryBuilder().back(3).build()
                    );
                    armMotor.setTargetPosition(2200);
                    armMotor.setPower(-1);
                    sleep(2000);
                    drive.turnSync(Math.PI/2);
                    drive.followTrajectorySync(
                            drive.trajectoryBuilder().back(92).build()
                    );
                    drive.turnSync(-Math.PI/2);

                }
                drive.followTrajectorySync(
                        drive.trajectoryBuilder().forward(7).build()
                );
                footServoTwo.setPosition(.9);
                footServoOne.setPosition(.1);
                drive.followTrajectorySync(drive.trajectoryBuilder().forward(6).build());
                grabServo.setPosition(.2);

                footServoOne.setPosition(.9);
                footServoTwo.setPosition(.1);
                sleep(1500);
                drive.followTrajectorySync(drive.trajectoryBuilder().back(29).build());
                // drive.turnSync(-Math.PI/2);
                drive.turnSync(-Math.PI);
                drive.followTrajectorySync(drive.trajectoryBuilder().forward(26).build());
                //drive.followTrajectorySync(drive.trajectoryBuilder().strafeRight(24).build());
                footServoTwo.setPosition(.9);
                footServoOne.setPosition(.1);
                drive.followTrajectorySync(
                        drive.trajectoryBuilder().back(35).build()
                );

            }
        }
    }
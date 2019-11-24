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
    Servo footServo;
    DcMotor armMotor;
    SkystoneTracker vue;

    public void runOpMode() throws InterruptedException {
        SampleMecanumDriveBase drive = new SampleMecanumDriveREV(hardwareMap);
        vue = new SkystoneTracker(.25);
        vue.init();
        RobotLog.d("After VF init");
        grabServo = hardwareMap.get(Servo.class, "thehandofnod");
        armMotor = hardwareMap.get(DcMotor.class, "thestrengthofnod");
        footServo = hardwareMap.get(Servo.class, "thefootofnod");
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setTargetPosition(0);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();
        if (isStopRequested()) return;
        drive.setPoseEstimate(new Pose2d(-46, -63, Math.PI / 2));
        vue.update();
        sleep(1000);
        drive.followTrajectorySync(
                drive.trajectoryBuilder().strafeLeft(4).build()
        );
        vue.update();
        drive.followTrajectorySync(
                drive.trajectoryBuilder().strafeRight(4).build()
        );
        sleep(250);
        vue.update();
        drive.followTrajectorySync(
                drive.trajectoryBuilder().forward(11).build()
        );
        sleep(250);
        vue.update();


            sleep(1000);
            if (vue.isVisible()) {
                armMotor.setTargetPosition(2500);
                armMotor.setPower(.7);
                sleep(3500);
                grabServo.setPosition(.1);
                armMotor.setTargetPosition(400);
                armMotor.setPower(-.5);
                drive.followTrajectorySync(
                        drive.trajectoryBuilder().back(4).build()
                );
                drive.followTrajectorySync(
                        drive.trajectoryBuilder().strafeRight(72).build()
                );

            } else {
                drive.followTrajectorySync(
                        drive.trajectoryBuilder().strafeLeft(8).build()
                );
                sleep(500);
                vue.update();
                if (vue.isVisible()) {
                    armMotor.setTargetPosition(2500);
                    armMotor.setPower(.7);
                    sleep(3500);
                    grabServo.setPosition(.1);
                    armMotor.setTargetPosition(400);
                    armMotor.setPower(-.5);
                    drive.followTrajectorySync(
                            drive.trajectoryBuilder().back(4).build()
                    );
                    drive.followTrajectorySync(
                            drive.trajectoryBuilder().strafeRight(80).build()
                    );
                } else {
                    drive.followTrajectorySync(
                            drive.trajectoryBuilder().strafeLeft(8).build()
                    );
                    armMotor.setTargetPosition(2500);
                    armMotor.setPower(.7);
                    sleep(3500);
                    grabServo.setPosition(.1);
                    armMotor.setTargetPosition(400);
                    armMotor.setPower(-.5);
                    drive.followTrajectorySync(
                            drive.trajectoryBuilder().back(4).build()
                    );
                    drive.followTrajectorySync(
                            drive.trajectoryBuilder().strafeRight(88).build()
                    );


                }
                grabServo.setPosition(.2);
                drive.followTrajectorySync(
                        drive.trajectoryBuilder().strafeLeft(40).build()
                );

            }
        }
    }









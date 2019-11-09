package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveBase;
import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveREV;


@Autonomous(group = "drive")
public class LeftAutoUltra extends LinearOpMode {

    Servo grabServo;
    Servo footServo;
    DcMotor armMotor;
    public void runOpMode() throws InterruptedException {
        SampleMecanumDriveBase drive = new SampleMecanumDriveREV(hardwareMap);
        boolean isBrick = false;
        grabServo = hardwareMap.get(Servo.class, "thehandofnod");
        armMotor = hardwareMap.get(DcMotor.class, "thestrengthofnod");
        footServo = hardwareMap.get(Servo.class,"thefootofnod");
        armMotor.setTargetPosition(armMotor.getCurrentPosition());
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        int currentPosition = 0;
        waitForStart();
        if (isStopRequested()) return;




        drive.setPoseEstimate(new Pose2d(-46,-63,Math.PI/2));
        drive.followTrajectorySync(
                drive.trajectoryBuilder().forward(48).build()
        );
        footServo.setPosition(.1);
        sleep(1000);

        drive.followTrajectorySync(
                drive.trajectoryBuilder().back(46).build()
        );
        footServo.setPosition(.9);
        drive.followTrajectorySync(
                drive.trajectoryBuilder().strafeLeft(26).build()
        );



        /*
        followSplineTo(43,-28,-90,drive);
        if(isBrick){
            followSplineTo(43,48,-Math.PI/2,drive);
            followSplineTo(43,-52,-Math.PI/2,drive);
            followSplineTo(43,40,-Math.PI/2,drive);
            followSplineTo(43,0,-Math.PI/2,drive);
        }else{
            followSplineTo(434,-36,-Math.PI/2,drive);
            if(isBrick){
                followSplineTo(43,48,-Math.PI/2,drive);
                followSplineTo(43,-60,-Math.PI/2,drive);
                followSplineTo(43,40,-Math.PI/2,drive);
                followSplineTo(43,0,-Math.PI/2,drive);

            }else{
                followSplineTo(43,-44,-Math.PI/2,drive);
                followSplineTo(43,48,-Math.PI/2,drive);
                followSplineTo(43,0,-Math.PI/2,drive);

            }
        }
        */
    }

    public void followSplineTo(double x, double y, double heading,SampleMecanumDriveBase drive){
        drive.followTrajectorySync(drive.trajectoryBuilder().splineTo(new Pose2d(x,y,heading)).build());
        drive.updatePoseEstimate();
    }



}
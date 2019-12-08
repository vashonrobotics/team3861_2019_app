package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveBase;
import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveREV;


@Autonomous(group = "drive")
public class RightAutoUltra extends LinearOpMode {

    Servo grabServo;
    Servo footServoOne;
    Servo footServoTwo;
    DcMotor armMotor;
    public void runOpMode() throws InterruptedException {
        SampleMecanumDriveBase drive = new SampleMecanumDriveREV(hardwareMap);
        boolean isBrick = false;
        grabServo = hardwareMap.get(Servo.class, "thehandofnod");
        armMotor = hardwareMap.get(DcMotor.class, "thestrengthofnod");
        footServoOne = hardwareMap.get(Servo.class,"footOfNodOne");
        footServoTwo = hardwareMap.get(Servo.class,"theFootOfNodTwo" );
        armMotor.setTargetPosition(armMotor.getCurrentPosition());
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        waitForStart();
        if (isStopRequested()) return;



        footServoTwo.setPosition(.9);
        footServoOne.setPosition(.1);
        drive.setPoseEstimate(new Pose2d(-46,-63,Math.PI/2));
        drive.followTrajectorySync(
                drive.trajectoryBuilder().forward(27).build()
        );

        footServoOne.setPosition(.9);
        footServoTwo.setPosition(.1);
        sleep(1500);
        drive.followTrajectorySync(drive.trajectoryBuilder().back(22).build());
       // drive.turnSync(-Math.PI/2);
        drive.turnSync(Math.PI+.1);
        drive.followTrajectorySync(drive.trajectoryBuilder().forward(26).build());
        //drive.followTrajectorySync(drive.trajectoryBuilder().strafeRight(24).build());
        footServoTwo.setPosition(.9);
        footServoOne.setPosition(.1);
        drive.followTrajectorySync(
                drive.trajectoryBuilder().back(35).build()
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
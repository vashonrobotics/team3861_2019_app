package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveBase;
import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveREV;

/*
 * This is an example of a more complex path to really test the tuning.
 */
@Autonomous(group = "drive")
public class BlueAuto extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDriveBase drive = new SampleMecanumDriveREV(hardwareMap);
        Boolean isBrick = true;


        waitForStart();
        if (isStopRequested()) return;
        drive.setPoseEstimate(new Pose2d(63,-28,90));
        followSplineTo(-43,-28,90,drive);
        if(isBrick){
            followSplineTo(-43,48,90,drive);
            followSplineTo(-43,-52,90,drive);
            followSplineTo(-43,40,90,drive);
            followSplineTo(-43,0,90,drive);
        }else{
            followSplineTo(-43,-36,90,drive);
            if(isBrick){
                followSplineTo(-43,48,90,drive);
                followSplineTo(-43,-60,90,drive);
                followSplineTo(-43,40,90,drive);
                followSplineTo(-43,0,90,drive);

            }else{
                followSplineTo(-43,-44,90,drive);
                followSplineTo(-43,48,90,drive);
                followSplineTo(-43,0,90,drive);

            }
        }
    }

    public void followSplineTo(double x, double y, double heading,SampleMecanumDriveBase drive){
        drive.followTrajectorySync(drive.trajectoryBuilder().splineTo(new Pose2d(x,y,heading)).build());
    }
}

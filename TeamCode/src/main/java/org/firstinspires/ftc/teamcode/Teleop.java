/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.drive.DriveSignal;
import com.acmerobotics.roadrunner.drive.MecanumDrive;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveBase;
import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveREV;


/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 * <p>
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 * <p>
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name = "Teleop", group = "Linear Opmode")
public class Teleop extends LinearOpMode {
    private static final double ROTATION_SCALE = -0.3;
    private static final double VELOCITY_SCALE = -0.9;
    private static final double ARM_SCALE = 0.45;

    Servo grabServo;
    DcMotorEx armMotor;
    Servo footServoOne;
    Servo footServoTwo;

    // Declare OpMode members.
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDriveBase drive = new SampleMecanumDriveREV(hardwareMap);
        grabServo = hardwareMap.get(Servo.class, "thehandofnod");
        armMotor = hardwareMap.get(DcMotorEx.class, "thestrengthofnod");
        footServoOne = hardwareMap.get(Servo.class, "footOfNodOne");
        footServoTwo = hardwareMap.get(Servo.class, "theFootOfNodTwo");
        armMotor.setTargetPosition(armMotor.getCurrentPosition());
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        armMotor.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(
//                20, 5, 8, 4
//        ));
        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        int currentPosition = 0;
        waitForStart();

        while (!isStopRequested()) {
            double velocityScale = VELOCITY_SCALE;
            double rotationScale = ROTATION_SCALE;
            double armScale = ARM_SCALE;

            float triggerValue = gamepad1.right_trigger;
            telemetry.addLine(String.format("trigger value = %f", triggerValue));
            rotationScale *= Math.min(1.05 - triggerValue, 1.0);
            velocityScale *= Math.min(1.25 - triggerValue, 1.0);

            if (gamepad2.right_trigger > 0.5) {
                armScale *= .5;
            }

            telemetry.addLine(String.format("rScale = %f, vScale = %f", rotationScale, velocityScale));
            drive.setDrivePower(new Pose2d(
                    velocityScale * gamepad1.left_stick_y,
                    velocityScale * gamepad1.left_stick_x,
                    rotationScale * gamepad1.right_stick_x));

            if (gamepad2.right_bumper) {
                grabServo.setPosition(.5);
            }
            if (gamepad2.left_bumper) {
                grabServo.setPosition(.2);
            }
            if (gamepad2.x) {
                footServoOne.setPosition(.9);
                footServoTwo.setPosition(.1);
            }
            if (gamepad2.y) {
                footServoTwo.setPosition(.9);
                footServoOne.setPosition(.1);
            }
                /*
                else if(gamepad2.left_bumper){
                    grabServo.setPosition(.3);
                }
                */

            //currentPosition = armMotor.getCurrentPosition();
            // currentPosition += (int)(90*gamepad2.right_stick_y);
            //armMotor.setTargetPosition(currentPosition);

            float armPower = gamepad2.right_stick_y;
            if (Math.abs(armPower) < 0.01) {
                armPower = 0;
            }

            if (armPower > 0) {
                armPower *= 0.5;
            }

            armMotor.setPower(armScale * armPower);


            // drive.update();

            Pose2d poseEstimate = drive.getPoseEstimate();
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.update();
        }
    }
}


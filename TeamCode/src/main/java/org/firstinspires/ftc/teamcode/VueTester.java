package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="VueTest", group="Linear Opmode")
public class VueTester extends LinearOpMode {
    SkystoneTracker vue;
    public void runOpMode()throws InterruptedException{
        vue=new SkystoneTracker();
        vue.init(hardwareMap);
        while(true){
            vue.update();
            telemetry.addLine(String.format("%d %d %d", SkystoneTracker.getValLeft(),
                    SkystoneTracker.getValMid(), SkystoneTracker.getValRight()));
            if(vue.isVisible()){
                telemetry.addData("isVisible",1);
            }else{
                telemetry.addData("what the heck?!?!",1);
            }
            telemetry.update();
        }
    }
}

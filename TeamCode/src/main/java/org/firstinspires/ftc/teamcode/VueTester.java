package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="VueTest", group="Linear Opmode")
public class VueTester extends LinearOpMode {
    SkystoneTracker vue;
    public void runOpMode()throws InterruptedException{
        vue=new SkystoneTracker(.25);
        vue.init();
        while(true){
            vue.update();
            if(vue.isVisible()){
                telemetry.addData("isVisible",1);
            }else{
                telemetry.addData("what the heck?!?!",1);
            }
            telemetry.update();
        }
    }
}

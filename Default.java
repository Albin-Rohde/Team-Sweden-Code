package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="Default")
public class Default extends OpMode {
    private Shooting shooting = null;
    private Drive drive = null;

    @Override
    public void init(){
        telemetry.addData("init", "Robot initialized");
        drive.init();
        shooting.init();
    }

    @Override
    public void start(){
        shooting.start();
        drive.start();
    }

    @Override
    public void loop(){
        drive.run();
        shooting.run();
        if(gamepad1.a && gamepad1.b){

        }
    }

    public void stop(){
        telemetry.addData("Stopped", "Robot was stopped");
    }
}

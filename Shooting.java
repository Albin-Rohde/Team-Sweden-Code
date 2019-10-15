package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

public class Shooting extends Default{
    private DcMotor shootingMotor = null;
    private Servo shootingServo = null;
    private Config config = null;

    public void init(){
        shootingMotor = hardwareMap.get(DcMotor.class, "motor2");
        shootingServo = hardwareMap.get(Servo.class, "servo0");
    }

    public void start(){
        shootingMotor.setPower(config.getMax());
    }

    public void run(){
        if (gamepad2.right_trigger > 0){
            shootingServo.setPosition(0);
        } else {
            shootingServo.setPosition(200);
        }
    }
}
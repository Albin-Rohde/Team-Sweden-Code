package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;

public class Drive extends Default{
    private DcMotor leftMotor = null;
    private DcMotor rightMotor = null;
    private DcMotor collectorMotor = null;
    private Config config = null;

    public void init(){
        leftMotor = hardwareMap.get(DcMotor.class, "motor0");
        rightMotor = hardwareMap.get(DcMotor.class, "motor1");
        collectorMotor = hardwareMap.get(DcMotor.class, "motor3");

        leftMotor.setDirection(DcMotor.Direction.FORWARD);
        rightMotor.setDirection(DcMotor.Direction.REVERSE);
    }

    public void start(){
        collectorMotor.setPower(config.getMax());
    }

    public void run(){
        double leftStickInput = gamepad1.left_stick_y;
        double rightStickInput = gamepad1.right_stick_x;

        double multiplier;

        if (gamepad1.left_trigger > 0) multiplier = config.getSlow();
        else if (gamepad1.right_trigger > 0) multiplier = config.getMax();
        else multiplier = config.getMiddle();

        double power = leftStickInput * multiplier;
        double turn = rightStickInput * multiplier;

        leftMotor.setPower(power - turn);
        rightMotor.setPower(power + turn);
    }
}

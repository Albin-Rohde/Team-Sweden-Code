package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.robot.Robot;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="Soma")
public class Soma extends OpMode {
    private ElapsedTime elapsedTime = new ElapsedTime();
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    private DcMotor collectorMotor = null;
    private DcMotor shootingMotor = null;
    private Servo shootingServo = null;

    private double slow = 0.20;
    private double normal = 0.65;
    private double fast = 1.00;

    @Override
    public void init(){
        telemetry.addData("init", "Robot initialized");
        leftDrive = hardwareMap.get(DcMotor.class, "motor0");
        rightDrive = hardwareMap.get(DcMotor.class, "motor1");

        leftDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setDirection(DcMotor.Direction.REVERSE);

        collectorMotor = hardwareMap.get(DcMotor.class, "motor3");

        shootingMotor = hardwareMap.get(DcMotor.class, "motor2");
        shootingServo = hardwareMap.get(Servo.class, "servo0");
    }

    private void drive() {
        double leftStickInputY = gamepad1.left_stick_y;
        double leftStickInputX = gamepad1.left_stick_y;

        double multiplier;

        if (gamepad1.left_trigger == true) multiplier = slow;
        else if (gamepad1.right_trigger == true) multiplier = fast;
        else multiplier = normal;

        double power = leftStickInputY * multiplier;
        double turn = leftStickInputX * multiplier;

        double leftPower = power - turn;
        double rightPower = power + turn;

        leftDrive.setPower(leftPower);
        rightDrive.setPower(rightPower);
    }

    private void shooting() {
        double x = gamepad2.right_stick_y;
        if (gamepad2.right_trigger > 0){
            shootingServo.setPosition(0);
        } else {
            shootingServo.setPosition(200);
        }
        telemetry.addData("stick", x);
    }


    @Override
    public void start(){
        elapsedTime.reset();
        shootingMotor.setPower(1);
        collectorMotor.setPower(1);
    }

    @Override
    public void loop(){
        drive();
        shooting();
    }

    public void stop(){
        telemetry.addData("Stopped", "Robot was stopped");
    }
}

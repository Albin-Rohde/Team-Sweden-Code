package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.ColorSensor;

@TeleOp(name="Main", group="Linear Opmode")

public class MainControllerSchema extends LinearOpMode {

    //utils
    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime intervalTime = new ElapsedTime();
    private ElapsedTime ballSensorDelta = null;

    // Motors
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    private DcMotor collector = null;
    private DcMotor elevator = null;
    private DcMotor canonLift = null;
    private DcMotor canon = null;
    private Servo trigger = null;

    // Other devices
    private AnalogInput canonAngle = null;
    private DigitalChannel magnetSensor = null;
    private DistanceSensor rangeSensor = null;
    private ColorSensor canonSensor = null;

    /* Controller schema */

    // gamepad 1
    private final double turning = gamepad1.right_stick_x;
    private final double accelerator = gamepad1.left_stick_y;
    private final boolean slowMode = (gamepad1.left_trigger == 1.00);
    private final boolean speedMode = (gamepad1.right_trigger == 1.00);

    // gamepad 2
    private final boolean shootTrigger = (gamepad2.right_trigger == 1.00);
    private final boolean toggleElevator = gamepad2.b;
    private final boolean aimUp = gamepad2.dpad_up;
    private final boolean aimDown = gamepad2.dpad_down;
    private final boolean shooting2mPreset = gamepad2.y;
    private final boolean shooting7mPreset = gamepad2.a;


    /* variables for runtime */

    // Speed multipliers
    private final double slowMultiplier = 0.4;
    private final double defaultMultiplier = 0.7;
    private final double fastMultiplier = 1.0;

    // Default speed
    private final double defaultCanonPower = 0.37;
    private final double defaultCollectorPower = 1.00;
    private final double defaultElevatorPower = 0.00;

    // rpm counter
    private boolean newRound = true;
    private double intervalHistory[] = new double[10];
    private int historyIndex = 0;
    private double totalTime = 0;

    // shooting
    private final double servoBack = 0.55;
    private final double servoPush = 0.00;
    private final double canonPower2meter = 0.37;
    private final double canonPower7meter = 0.8;
    private boolean shoot = false;


    // Angle config
    private final double maxAngle = 1.178;
    private final double minAngle = 0.399;
    private final double preset2meterAngle = 1.150;
    private final double preset7meterAngle = 0.750;


    /* Methods */

    private void mapDevices() {
        // Motors
        leftDrive  = hardwareMap.get(DcMotor.class, "leftDrive");
        rightDrive = hardwareMap.get(DcMotor.class, "rightDrive");
        collector = hardwareMap.get(DcMotor.class, "collector");
        elevator = hardwareMap.get(DcMotor.class, "elevator");
        canonLift = hardwareMap.get(DcMotor.class, "canonLift");
        canon = hardwareMap.get(DcMotor.class, "canon");
        trigger = hardwareMap.get(Servo.class, "trigger");

        // Sensors
        canonAngle = hardwareMap.analogInput.get("canonAngle");
        magnetSensor = hardwareMap.digitalChannel.get("magnet");
        rangeSensor = hardwareMap.get(DistanceSensor.class, "rangeSensor");
        canonSensor = hardwareMap.colorSensor.get("color");
    }


    private void config() {
        magnetSensor.setMode(DigitalChannel.Mode.INPUT);

        canon.setPower(defaultCanonPower);
        collector.setPower(defaultCollectorPower);
        elevator.setPower(defaultElevatorPower);

        leftDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setDirection(DcMotor.Direction.REVERSE);
    }


    private void drive() {
        double currentMultiplier;

        if (speedMode) {
            currentMultiplier = fastMultiplier;
        } else if (slowMode) {
            currentMultiplier = slowMultiplier;
        } else {
            currentMultiplier = defaultMultiplier;
        }

        double power = accelerator * currentMultiplier;
        double turn = -turning * currentMultiplier;

        double leftPower    = Range.clip(power + turn, -1.0, 1.0) ;
        double rightPower   = Range.clip(power - turn, -1.0, 1.0) ;

        leftDrive.setPower(leftPower);
        rightDrive.setPower(rightPower);
    }


    private void manualShooting() {
        if (shootTrigger) {
            trigger.setPosition(servoPush);
        } else {
            trigger.setPosition(servoBack);
        }
    }


    private void autonomousShooting() {
        if (shootTrigger) {
            shoot = true;
        }

        if (((DistanceSensor) canonSensor).getDistance(DistanceUnit.CM) < 7) {
            shoot = true;
        }

        if (trigger.getPosition() == 0) {
            if (ballSensorDelta == null) {
                ballSensorDelta = new ElapsedTime();
                ballSensorDelta.startTime();
            }
            if (ballSensorDelta.time() < 100){
                shoot = false;
            }
        }
        telemetry.addData("debug", "shoot: " + shoot);
        telemetry.addData("debug", "servo pos: " + trigger.getPosition());

        if (shoot) {
            trigger.setPosition(0);
        } else {
            trigger.setPosition(0.55);
        }
    }


    private void toggleElevator() {
        if (toggleElevator) {
            elevator.setPower(1);
        } else {
            elevator.setPower(0);
        }
    }


    private void aimCanon() {
        double angleOfCanon = canonAngle.getVoltage();

        if (aimUp && angleOfCanon < maxAngle) {
            canonLift.setPower(1);
        } else if (aimDown && angleOfCanon > minAngle) {
            canonLift.setPower(-1);
        }

        if (shooting2mPreset && angleOfCanon < preset2meterAngle) {
            canonLift.setPower(1);
        } else if (shooting2mPreset && angleOfCanon > preset7meterAngle) {
            canonLift.setPower(-1);
        }


        if (shooting2mPreset) {
            canon.setPower(canonPower2meter);
        } else if (shooting7mPreset) {
            canon.setPower(canonPower7meter);
        }
    }



    private double calculateCanonSpeed () {
        if (magnetSensor.getState() && newRound) {
            if (historyIndex == 10) {
                historyIndex = 0;
                totalTime = 0;
                for (int i = 0; i < 10; i++) {
                    totalTime += intervalHistory[i];
                }
            }

            double time = intervalTime.time();
            intervalHistory[historyIndex] = time;
            historyIndex++;
            intervalTime.reset();
            newRound = false;
        }
        else if (!magnetSensor.getState()) {
            newRound = true;
        }

        double rpmOfBigWheel = (5 / totalTime) * 60;
        return rpmOfBigWheel;
    }


    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        mapDevices();
        config();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();
        intervalTime.startTime();
        while (opModeIsActive()) {
            drive();
            toggleElevator();
            aimCanon();
            autonomousShooting();
            double rpm = calculateCanonSpeed();

            telemetry.addData("info", "RPM: " + rpm);
            telemetry.addData("info", "Shooting angle: " + canonAngle.getVoltage());
            telemetry.addData("info", "Distance: " + rangeSensor.getDistance(DistanceUnit.CM));
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            // telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
            telemetry.update();
        }
    }
}

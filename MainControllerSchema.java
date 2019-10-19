package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


@TeleOp(name="Main", group="Linear Opmode")
public class MainControllerSchema extends LinearOpMode {
    //utils
    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime intervalTime = new ElapsedTime();
    private ElapsedTime TimeNow = new ElapsedTime();
    private ElapsedTime ContinusShootTimer = new ElapsedTime();

    // Motors
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    private DcMotor collector = null;
    private DcMotor elevator = null;
    private DcMotor canonLift = null;
    private DcMotor plowMotor = null;
    private DcMotor hookMotor = null;
    private DcMotor canon = null;
    private Servo trigger = null;

    // Other devices
    private AnalogInput canonAngle = null;
    private DigitalChannel magnetSensor = null;
    private DistanceSensor rangeSensor = null;
    private ColorSensor canonSensor = null;

    /* Controller schema */

    // gamepad 1
    private double turning;
    private double accelerator;
    private boolean slowMode;
    private boolean speedMode;
    private boolean HookUp;
    private boolean HookDown;
    private boolean hangHold;

    // gamepad 2
    private boolean shootTrigger;
    private boolean toggleElevator;
    private boolean aimUp;
    private boolean aimDown;
    private boolean shooting2mPreset;
    private boolean shooting7mPreset;
    private double plowPower;


    /* variables for runtime */

    // Speed multipliers
    private final double slowMultiplier = 0.4;
    private final double defaultMultiplier = 0.7;
    private final double fastMultiplier = 1.0;

    // Default speed
    private final double defaultCanonPower = 0.40; //0.37
    private final double defaultCollectorPower = 0.00; //1
    private final double defaultElevatorPower = 0.00; //1

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


    // Angle config
    private final double maxAngle = 1.178;
    private final double minAngle = 0.399;
    private final double preset2meterAngle = 1.150;
    private final double preset7meterAngle = 0.750;

    // Lifting
    private boolean isHookPositive = true;

    private boolean currentlyAutoDriving = false;

    /* Methods */

    private void mapDevices() {
        // Motors
        leftDrive  = hardwareMap.get(DcMotor.class, "leftDrive");
        rightDrive = hardwareMap.get(DcMotor.class, "rightDrive");
        collector = hardwareMap.get(DcMotor.class, "collector");
        elevator = hardwareMap.get(DcMotor.class, "elevator");
        canonLift = hardwareMap.get(DcMotor.class, "canonLift");
        plowMotor = hardwareMap.get(DcMotor.class, "plow");
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
        TimeNow.startTime();
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



    private void driveToPos() {
        currentlyAutoDriving = true;
        if (rangeSensor.getDistance(DistanceUnit.CM) > 105) {
            leftDrive.setPower(slowMultiplier);
            rightDrive.setPower(slowMultiplier);
        } else if (rangeSensor.getDistance(DistanceUnit.CM) < 100) {
            leftDrive.setPower(-slowMultiplier);
            rightDrive.setPower(-slowMultiplier);
        } else {
            currentlyAutoDriving = false;
        }

    }


    private void manualShooting() {
        if (!shootTrigger) {
            ContinusShootTimer.reset();
        }
        if (ContinusShootTimer.time() < 2) {
            if (shootTrigger) {
                trigger.setPosition(servoPush);
            } else {
                trigger.setPosition(servoBack);
            }
        }
    }


    private void autonomousShooting() {
        boolean ballInCanon = ((DistanceSensor) canonSensor).getDistance(DistanceUnit.CM) < 7;

        if (ballInCanon && (TimeNow.time() > 1) && ContinusShootTimer.time() > 2){
            TimeNow.reset();
        }

        if ((TimeNow.time() > 0.2) && (TimeNow.time() < 0.6)) {
            trigger.setPosition(servoPush);
        } else {
            trigger.setPosition(servoBack);
        }

        telemetry.addData("time now? ", TimeNow.time());
        telemetry.addData("ball in canon? ", ballInCanon);
    }


    private void toggleElevator() {
        elevator.setPower(toggleElevator ? 0.00 : 1.00);
    }

    private void aimCanon() {
        double angleOfCanon = canonAngle.getVoltage();

        if (aimUp) {
            canonLift.setPower(1);
        } else if (aimDown) {
            canonLift.setPower(-1);
        }else {
            canonLift.setPower(0);
        }

        if ((shooting2mPreset && shooting7mPreset) || (runtime.time() < 4 && runtime.time() > 1)) {
            if (angleOfCanon < (preset2meterAngle + preset7meterAngle) / 2 * 0.9) {
                canonLift.setPower(1);
            }
            else if (angleOfCanon > (preset2meterAngle + preset7meterAngle) / 2 * 1.1) {
                canonLift.setPower(-1);
            }
        }else if (shooting2mPreset) {
            canon.setPower(canonPower2meter);
            if (angleOfCanon < preset2meterAngle) {
                canonLift.setPower(1);
            }
        } else if (shooting7mPreset) {
            canon.setPower(canonPower7meter);
            if (angleOfCanon > preset7meterAngle) {
                canonLift.setPower(-1);
            }
        }
    }

    private void plow() {
        plowMotor.setPower(plowPower);
    }

    private void liftRobot() {
        if (HookUp) {
            hookMotor.setPower(1);
            isHookPositive = true;
        } else if (HookDown) {
            hookMotor.setPower(-1);
            isHookPositive = false;
        } else if (hangHold) {
            hookMotor.setPower(isHookPositive ? 0.3 : -0.3);
        } else {
            hookMotor.setPower(0);
        }
    }


    private double calculateCanonSpeed () {
        if (magnetSensor.getState() && newRound) {
            totalTime = 0;
            for (int i = 0; i < 10; i++) {
                totalTime += intervalHistory[i];
            }

            double time = intervalTime.time();
            intervalHistory[historyIndex] = time;
            historyIndex = (historyIndex +1) % 10;
            intervalTime.reset();
            newRound = false;
        }
        else if (!magnetSensor.getState()) {
            newRound = true;
        }

        double rpmOfBigWheel = ((125/30) / totalTime) * 60;
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
            turning = 0.00;
            accelerator = 0.00;
            slowMode = false;
            speedMode = false;

            shootTrigger = false;
            toggleElevator = false;
            aimUp = false;
            aimDown = false;
            shooting2mPreset = false;
            shooting7mPreset = false;
            plowPower = 0.00;
            HookUp = false;
            HookDown = false;
            hangHold = false;

            turning = gamepad1.right_stick_x;
            accelerator = gamepad1.left_stick_y;
            slowMode = (gamepad1.left_trigger == 1.00);
            speedMode = (gamepad1.right_trigger == 1.00);
            hangHold = gamepad1.a;

            shootTrigger = (gamepad2.right_trigger == 1.00);
            toggleElevator = (gamepad2.left_trigger == 1.00);
            aimUp = gamepad2.dpad_up;
            aimDown = gamepad2.dpad_down;
            shooting2mPreset = gamepad2.y;
            shooting7mPreset = gamepad2.a;
            plowPower = gamepad2.right_stick_y;
            HookUp = gamepad2.dpad_up;
            HookDown = gamepad2.dpad_down;

            if (gamepad1.a || currentlyAutoDriving && Math.abs(turning) <0.2 && Math.abs(accelerator) <0.2) {
                driveToPos();
            } else {
                drive();
            }
            toggleElevator();
            aimCanon();
            autonomousShooting();
            plow();
            liftRobot();
            double rpm = calculateCanonSpeed();


            telemetry.addData("info", "RPM: " + rpm);
            telemetry.addData("info", "Shooting angle: " + canonAngle.getVoltage());
            telemetry.addData("info", "Distance: " + rangeSensor.getDistance(DistanceUnit.CM));
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("info", "Currently AutoDriving: " + currentlyAutoDriving );

            telemetry.addData("debug", "rt" + gamepad1.right_stick_y);
            telemetry.addData("debug", "lt" + gamepad1.left_stick_x);
            // telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
            telemetry.update();
        }
    }
}

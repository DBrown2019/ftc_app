package org.firstinspires.ftc.team7234.RoverRuckus.opmodes;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.team7234.RoverRuckus.common.HardwareBotman;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp(name = "Botman TeleOp", group = "Botman")
public class BotmanTeleOp extends OpMode {

    private HardwareBotman robot = new HardwareBotman();
    private final String logTag = HardwareBotman.class.getName();

    //Define the speed of the motors as doubles
    private double leftSpeed;
    private double rightSpeed;

    private double extendSpeed;

    private double armExtendSpeed;
    private double armRotSpeed;

    private double lockHeading;

    private boolean collectorToggle;
    private boolean doGyroDrive;
    private boolean gyroToggle;

    private ElapsedTime elapsedTime;

    private int minTwist;
    private int maxTwist;

    @Override
    public void init() {
        elapsedTime = new ElapsedTime();
            /* This line of code tells that robot to ignore the camera.  We do not want to
            use the camera in teleop, so as to save CPU cycles and make the code run faster..*/
        try {
            robot.init(hardwareMap, false);
        }
        catch (Exception ex){
            Log.e(logTag, ex.toString());
        }

        Log.i(logTag, "Robot Initialized in " + robot.time + " ms");

        robot.resetEncoders();
        robot.extension.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        collectorToggle = false;

        robot.armLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.armTwist.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        elapsedTime.reset();

    }

    @Override
    public void start() {
        robot.leftWheel.setDirection(DcMotor.Direction.REVERSE);
        robot.rightWheel.setDirection(DcMotor.Direction.FORWARD);
        elapsedTime.reset();
        minTwist = robot.armTwist.getCurrentPosition();
        maxTwist = -150; //TODO: test and find correct position
    }

    @Override
    public void loop() {
       /*  This line of code determines which joysticks control which motors.  Earlier our
        driver was having trouble controlling the robot.  So, we inverted the controls.  */
        rightSpeed = gamepad1.right_stick_y;
        leftSpeed = gamepad1.left_stick_y;
        extendSpeed = gamepad1.left_trigger - gamepad1.right_trigger;

        armExtendSpeed = gamepad2.left_trigger - gamepad2.right_trigger;
        //armRotSpeed = (gamepad2.left_stick_y > 0) ? gamepad2.left_stick_y * gamepad2.left_stick_y : -1*gamepad2.left_stick_y*gamepad2.left_stick_y;

        armRotSpeed = 0.85*gamepad2.left_stick_y;
        //Set the power of the motors to previously defined speeds

        /*if (robot.armTwist.getCurrentPosition() > minTwist && armRotSpeed > 0){
            armRotSpeed = 0;
        }
        else if (robot.armTwist.getCurrentPosition() < maxTwist && armRotSpeed < 0){
            armRotSpeed = 0;
        }*/

        robot.extension.setPower(extendSpeed);
        robot.armLift.setPower(armExtendSpeed);
        robot.armTwist.setPower(armRotSpeed);

        if (doGyroDrive){
            robot.driveByGyro(0.9*(rightSpeed+leftSpeed)/2, lockHeading);
        }
        else {
            robot.rightWheel.setPower(rightSpeed);
            robot.leftWheel.setPower(leftSpeed);
        }

        if (gyroToggle){ //toggles the gyro drive mode
            if (gamepad1.a){
                doGyroDrive = !doGyroDrive;
                lockHeading = robot.heading(); //Updates the heading lock whenever the gyro drive is toggled.
                gyroToggle = false;
            }
        }
        else if (!gamepad1.a){
            gyroToggle = true;
        }

        if (collectorToggle && elapsedTime.milliseconds() > 5000){
            if (gamepad2.a){
                collectorToggle = false;
                robot.collectorSpinner.setPower(1.0);
            }
            else if (gamepad2.b){
                collectorToggle = false;
                robot.collectorSpinner.setPower(-1.0);
            }
            else if (gamepad2.x){
                collectorToggle = false;
                robot.collectorSpinner.setPower(0.0);
            }
        }
        else if (!(gamepad2.a || gamepad2.b || gamepad2.x)){
            collectorToggle = true;
        }


        telemetry.addData("Extension Position", robot.extension.getCurrentPosition());
        telemetry.addData("Robot Heading: ", robot.heading());
        telemetry.addData("Robot Pitch", robot.pitch());
        telemetry.addData("Robot Roll", robot.roll());

        telemetry.addData("Arm Twist", robot.armTwist.getCurrentPosition());



    }


    @Override
    public void stop(){
        //Stop the motors
        robot.rightWheel.setPower(0.0);
        robot.leftWheel.setPower(0.0);
        robot.extension.setPower(0.0);
        robot.collectorSpinner.setPower(0.0);
        robot.armTwist.setPower(0.0);
        robot.armLift.setPower(0.0);
    }

}

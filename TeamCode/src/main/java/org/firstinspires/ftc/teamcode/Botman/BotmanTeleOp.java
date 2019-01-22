package org.firstinspires.ftc.teamcode.Botman;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;


@TeleOp(name = "Botman TeleOp", group = "Botman")
public class BotmanTeleOp extends OpMode {

    private HardwareBotman robot = new HardwareBotman();
    private final String logTag = HardwareBotman.class.getName();

    //Define the speed of the motors as doubles
    double leftSpeed = 0;
    double rightSpeed = 0;
    double extendSpeed = 0;

    @Override
    public void init() {
            /* This line of code tells that robot to ignore the camera.  We do not want to
            use the camera as of now because we are currently running into problems.  The phone
            is not mounted in an ideal place for element detection.*/
        try {
            robot.init(hardwareMap, false);
        }
        catch (IllegalArgumentException ex){
            Log.e(logTag, ex.toString());
        }

        Log.i(logTag, "Robot Initialized in " + robot.time + " ms");

        robot.resetEncoders();
    }

    @Override
    public void start() {
        robot.extension.setDirection(DcMotor.Direction.REVERSE);
        robot.extension.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.DriveWithoutEncoders();
    }


    @Override
    public void loop() {
       /*  This line of code determines which joysticks control which motors.  Earlier our
        driver was having trouble controlling the robot.  So, we inverted the controls.  */
        rightSpeed = -gamepad1.right_stick_y;
        leftSpeed = -gamepad1.left_stick_y;
        extendSpeed = gamepad1.left_trigger - gamepad1.right_trigger;

        //Set the power of the motors to previously defined speeds
        robot.rightWheel.setPower(rightSpeed);
        robot.leftWheel.setPower(leftSpeed);
        robot.extension.setPower(extendSpeed);

//        if (gamepad2.a){
//            robot.collector.setPower(1);
//        } else if (gamepad2.b){
//            robot.collector.setPower(-1);
//        } else  if (gamepad2.x){
//            robot.collector.setPower(0);
//        }
//
//        if (gamepad2.right_stick_y<-0.1){
//            robot.armExtension.setPower(gamepad2.right_stick_y);
//        } else if (gamepad2.right_stick_y>0.1){
//            robot.armExtension.setPower(gamepad2.right_stick_y);
//        } else {
//            robot.armExtension.setPower(0);
//        }
        //Get the current position of the motor
        telemetry.addData("Extension Position", robot.extension.getCurrentPosition());
        telemetry.addData("Left Motor", robot.leftWheel.getCurrentPosition());
        telemetry.addData("Right Motor", robot.rightWheel.getCurrentPosition());



    }


    @Override
    public void stop(){
        //Stop the motors
        robot.rightWheel.setPower(0.0);
        robot.leftWheel.setPower(0.0);
        robot.extension.setPower(0.0);
    }

}

package org.firstinspires.ftc.teamcode.Botman;

import android.drm.DrmStore;
import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayList;
import java.util.List;
import java.util.OptionalDouble;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;

@Autonomous(name = "Crater Side", group = "Botman")
public class AutoCrater extends OpMode {

    HardwareBotman robot = new HardwareBotman();


    private final String TAG = "Autonomous";

    private static final int sampleRate = 4; //How many samples to take, per second.
    private static final int timeDelay = 1000/sampleRate; //Delay between samples in milliseconds

    private final double EXTENSION_TARGET = 10750;
    private final double LEFT_MINERAL_THETA = 15;
    private final double RIGHT_MINERAL_THETA = -20;

    private boolean modeChange = false;
    private boolean axisChanged = false;
    private boolean gyroRead = false;

    private final double DEPOT_THETA = -135; //Rotation target for going from


    ElapsedTime elapsedTime = new ElapsedTime();


    public enum CurrentState{
        PREP,               //Preparation for the match, sets up the timers and starts moving the robot
        LOWER,              //Lower from the hook and onto the field
        FORWARD,            //Briefly move forward from the hook so that we're not touching
        EVALUATE_MINERALS,  //Stop scanning for the minerals and start
        TURN_TO_MINERAL,    //Turn to the mineral in order to knock it off
        REMOVE_MINERAL,     //Go forward to knock off the mineral

        CRATER_BACKING_UP,
        CRATER_TURN_TO_WALL,
        CRATER_NAV_TO_WALL,
        CRATER_TURN_TO_DEPOT, //Turn towards the depot, from the crater
        CRATER_NAV_TO_DEPOT,  //Navigate to the depot from the crater
        CRATER_NAV_TO_CRATER, //Navigate to the crater

        STOP
    }

    private CurrentState state = CurrentState.PREP;


    @Override
    public void init() {

        robot.init(hardwareMap, true);
    }

    @Override
    public void init_loop(){
        if(!modeChange) {
            robot.gyroConfigMode();
            elapsedTime.reset();
            modeChange = true;
        } else if(!axisChanged && elapsedTime.milliseconds()>200){
            robot.changeAxis();
            elapsedTime.reset();
            axisChanged = true;
        } else if(!gyroRead && axisChanged) {
            robot.gyroReadMode();
            gyroRead = true;
            elapsedTime.reset();
        } else if(gyroRead && elapsedTime.milliseconds() > 200) {
            telemetry.addData("1 Program State: ", state);
            telemetry.addData("2 Robot Heading: ", robot.heading());
        }
        robot.ObjectDetetion(telemetry);

    }

    @Override
    public void start() {
        robot.extension.setDirection(DcMotor.Direction.REVERSE);
        robot.RunByEncoders();
        robot.tfod.deactivate();

    }


    @Override
    public void loop() {

        telemetry.addData("Program State: ", state);
        telemetry.addData("Robot Heading: ", robot.heading());

        switch (state){
            case PREP:
                elapsedTime.reset();
                robot.extension.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.leftWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.rightWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.extension.setPower(1.0);
                state = CurrentState.LOWER;
                elapsedTime.reset();
                break;
            case LOWER:
                if (robot.extension.getCurrentPosition() >= EXTENSION_TARGET || elapsedTime.seconds() > 5){
                    elapsedTime.reset();
                    robot.extension.setPower(0.0);
                    state = CurrentState.FORWARD;
                }

                break;
            case FORWARD:         //Moves the robot forward to completely detach from the latch.  May need modification.
                if (robot.leftWheel.getCurrentPosition()>1000){

                    robot.leftWheel.setPower(0.0);
                    robot.rightWheel.setPower(0.0);
                    state = CurrentState.TURN_TO_MINERAL;
                }
                else {
                    robot.leftWheel.setPower(.4);
                    robot.rightWheel.setPower(.4);
                }
                break;

            case TURN_TO_MINERAL:
                switch (robot.mineralState){
                    case GOLD_CENTER:
                        robot.leftWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        state = CurrentState.REMOVE_MINERAL;
                        break;
                    case GOLD_LEFT:
                        robot.leftWheel.setPower(-0.3);
                        robot.rightWheel.setPower(0.3);
                        Log.v(TAG, "Now turning to left mineral, robot heading is: " + robot.heading() + ", Target Heading is: " + LEFT_MINERAL_THETA);
                        if (robot.heading() > LEFT_MINERAL_THETA){
                            robot.leftWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                            state = CurrentState.REMOVE_MINERAL;
                        }
                        break;
                    case GOLD_RIGHT:
                        robot.leftWheel.setPower(0.3);
                        robot.rightWheel.setPower(-0.3);
                        Log.v(TAG, "Now turning to right mineral, robot heading is: " + robot.heading() + ", Target Heading is: " + RIGHT_MINERAL_THETA);
                        if (robot.heading() < RIGHT_MINERAL_THETA){
                            robot.leftWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                            state = CurrentState.REMOVE_MINERAL;
                        }
                        break;
                    default:
                        robot.leftWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        state = CurrentState.REMOVE_MINERAL;
                        break;
                }
                break;

            case REMOVE_MINERAL:
                robot.leftWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                if (robot.leftWheel.getCurrentPosition()>1200){
                    robot.rightWheel.setPower(0.0);
                    robot.leftWheel.setPower(0.0);
                    state = CurrentState.CRATER_BACKING_UP;
                }
                else {
                    robot.rightWheel.setPower(0.4);
                    robot.leftWheel.setPower(0.4);
                }
                break;
            case CRATER_BACKING_UP:
                if (robot.leftWheel.getCurrentPosition()<=350){
                    robot.rightWheel.setPower(0.0);
                    robot.leftWheel.setPower(0.0);
                    state = CurrentState.CRATER_TURN_TO_WALL;
                }
                else {
                    robot.rightWheel.setPower(-0.4);
                    robot.leftWheel.setPower(-0.4);
                }
                break;

            case CRATER_TURN_TO_WALL:
                robot.leftWheel.setPower(-0.3);
                robot.rightWheel.setPower(0.3);
                if (robot.heading() > 40){
                    robot.leftWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    state = CurrentState.CRATER_NAV_TO_WALL;
                }
                break;

            case CRATER_NAV_TO_WALL:
                robot.leftWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                if (robot.leftWheel.getCurrentPosition()>2800){
                    robot.rightWheel.setPower(0.0);
                    robot.leftWheel.setPower(0.0);
                    state = CurrentState.CRATER_TURN_TO_DEPOT;
                }
                else {
                    robot.driveByGyro(0.4,80);
                }

                break;

            case CRATER_TURN_TO_DEPOT:
                robot.leftWheel.setPower(-0.3);
                robot.rightWheel.setPower(0.3);
                if (robot.heading() > 100){
                    robot.leftWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    state = CurrentState.CRATER_NAV_TO_DEPOT;
                }
                break;

            case CRATER_NAV_TO_DEPOT:
                robot.leftWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                if (robot.leftWheel.getCurrentPosition()>2500){
                    robot.rightWheel.setPower(0.0);
                    robot.leftWheel.setPower(0.0);
                    state = CurrentState.CRATER_NAV_TO_CRATER;
                }
                else {
                    robot.driveByGyro(0.4,130);
                }
                break;

            case CRATER_NAV_TO_CRATER:
                robot.leftWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                robot.leftWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                if (robot.leftWheel.getCurrentPosition()<=-4000){
                    robot.rightWheel.setPower(0.0);
                    robot.leftWheel.setPower(0.0);
                    state = CurrentState.STOP;
                }
                else {
                    robot.driveByGyro(-0.4,180);
                }
                break;

            case STOP:
                robot.leftWheel.setPower(0.0);
                robot.rightWheel.setPower(0.0);
                robot.extension.setPower(0.0);
                break;
        }


    }

    @Override
    public void stop() {

    }
}

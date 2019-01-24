package org.firstinspires.ftc.team7234.RoverRuckus.common;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.team7234.RoverRuckus.common.Imaging.GoldMineral;
import org.firstinspires.ftc.team7234.RoverRuckus.common.Imaging.Mineral;
import org.firstinspires.ftc.team7234.RoverRuckus.common.Imaging.MineralPosition;
import org.firstinspires.ftc.team7234.RoverRuckus.common.Imaging.SilverMineral;
import org.firstinspires.ftc.team7234.RoverRuckus.common.enums.AllianceColor;
import org.firstinspires.ftc.team7234.RoverRuckus.common.enums.DetectorType;
import org.firstinspires.ftc.team7234.RoverRuckus.common.enums.FieldPosition;

import java.util.ArrayList;
import java.util.List;
import java.util.OptionalDouble;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

public class AutoBase extends OpMode {

    HardwareBotman robot = new HardwareBotman();

    private final AllianceColor allianceColor;
    private final FieldPosition fieldPosition;
    private final String TAG = "BotmanAutonomous";

    private static final int sampleRate = 4; //How many samples to take, per second.
    private static final int timeDelay = 1000/sampleRate; //Delay between samples in milliseconds

    private ArrayList<Mineral> allMinerals = new ArrayList<>();





    private MineralPosition finalPos = null;

    private final double EXTENSION_TARGET = 10750;
    private final double LEFT_MINERAL_THETA = 15;
    private final double RIGHT_MINERAL_THETA = -20;

    private boolean modeChange = false;
    private boolean axisChanged = false;
    private boolean gyroRead = false;

    private final double DEPOT_THETA = -135; //Rotation target for going from



    private ScheduledExecutorService timer;
    private Runnable mineralSensor;

    private int samplesTaken = 0;

    ElapsedTime elapsedTime = new ElapsedTime();
    boolean stopAtDescent;


    public AutoBase(AllianceColor allianceColor, FieldPosition fieldPosition, boolean stopAtDescent){
        this(allianceColor, fieldPosition);
        this.stopAtDescent = stopAtDescent;
    }

    public AutoBase(AllianceColor allianceColor, FieldPosition fieldPosition){
        this.allianceColor = allianceColor;
        this.fieldPosition = fieldPosition;
        this.stopAtDescent = false;
    }



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
        CRATER_NAV_TO_CRATER,

        DEPOT_TURN_TO_DEPOT,
        DEPOT_DRIVE_TO_DEPOT,
        DEPOT_TURN_TO_CRATER,
        DEPOT_DRIVE_TO_CRATER,

        STOP
    }

    private CurrentState state = CurrentState.PREP;


    @Override
    public void init() {

        robot.init(hardwareMap, DetectorType.CASCADE);
        if (fieldPosition == FieldPosition.CRATER){
            robot.detector.setMaskCrater(true);
        }

    }

    @Override
    public void start() {
        try { //Starts the mineral sensor process on another Thread
            mineralSensor = new Runnable() {
                @Override
                public void run() {
                    try {
                        robot.detector.update();

                        List<Mineral> minerals = robot.detector.getMinerals();
                        allMinerals.addAll(minerals);
                        samplesTaken++;

                    }
                    catch (Exception ex){
                        Log.w(TAG, ex.getMessage());
                    }

                }
            };
            this.timer = Executors.newSingleThreadScheduledExecutor();
            this.timer.scheduleAtFixedRate(mineralSensor, 250, timeDelay, TimeUnit.MILLISECONDS);
        }
        catch (Exception ex){
            Log.e(TAG, ex.getMessage());
        }

        robot.leftWheel.setDirection(DcMotor.Direction.FORWARD);
        robot.rightWheel.setDirection(DcMotor.Direction.REVERSE);
        ///robot.extension.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //robot.leftWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //robot.rightWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    @Override
    public void loop() {

        telemetry.addData("Program State: ", state);
        telemetry.addData("Extension Position", robot.extension.getCurrentPosition());
        telemetry.addData("Expected Mineral Position: ", finalPos);
        telemetry.addData("Samples Taken: ", samplesTaken);
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
                if (robot.extension.getCurrentPosition() >= EXTENSION_TARGET || elapsedTime.milliseconds() > 14000){ //Failsafe
                    elapsedTime.reset();

                    if (timer != null){
                        timer.shutdownNow();
                    }
                    Log.i(TAG, "Finished lowering");
                    robot.extension.setPower(0.0);

                    state = CurrentState.FORWARD;
                }
                Log.v(TAG, "Descending to " + EXTENSION_TARGET +", Current position is " + robot.extension.getCurrentPosition());

                break;
            case FORWARD:         //Moves the robot forward to completely detach from the latch.  May need modification.
                if (robot.leftWheel.getCurrentPosition() >= 1000){

                    robot.leftWheel.setPower(0.0);
                    robot.rightWheel.setPower(0.0);
                    Log.i(TAG, "Forward State completed, evaluating minerals");
                    if (stopAtDescent){
                        state = CurrentState.STOP;
                    }
                    else {
                        state = CurrentState.EVALUATE_MINERALS;
                    }
                }
                else {
                    Log.v(TAG, "Moving forward to target position");
                    robot.leftWheel.setPower(.75);
                    robot.rightWheel.setPower(.75);
                }
                break;
            case EVALUATE_MINERALS:
                try {
                    ArrayList<Mineral> goldReadings = new ArrayList<>();
                    ArrayList<Mineral> silverReadings = new ArrayList<>();

                    for (Mineral m :
                            allMinerals) {
                        if (m instanceof GoldMineral) {
                            goldReadings.add(m);
                        }
                        else if (m instanceof SilverMineral){
                            silverReadings.add(m);
                        }
                    }


                    //region Averaging
                    OptionalDouble avgGold = goldReadings.stream().mapToDouble(m -> (m.getX() + m.getWidth()/2.0) ).average();
                    OptionalDouble avgSilver = silverReadings.stream().mapToDouble(m -> (m.getX() + m.getWidth()/2.0) ).average();

                    ArrayList<Mineral> leftSilverReadings = new ArrayList<>();
                    ArrayList<Mineral> rightSilverReadings = new ArrayList<>();

                    if (avgSilver.isPresent()){
                        for (Mineral m :
                                silverReadings) {
                            if (m.getX() < avgSilver.getAsDouble()) {
                                leftSilverReadings.add(m);
                            }
                            else {
                                rightSilverReadings.add(m);
                            }
                        }
                    }
                    else {
                        finalPos = MineralPosition.CENTER;
                    }


                    OptionalDouble leftSilver = leftSilverReadings.stream().mapToDouble(m -> (m.getX() + m.getWidth()/2.0) ).average();
                    OptionalDouble rightSilver = rightSilverReadings.stream().mapToDouble(m -> (m.getX() + m.getWidth()/2.0) ).average();

                    //endregion
                    //region Set Positions
                    if (avgGold.isPresent() && leftSilver.isPresent() && rightSilver.isPresent()){
                        if (avgGold.getAsDouble() < leftSilver.getAsDouble()){
                            finalPos = MineralPosition.LEFT;
                        }
                        else if (avgGold.getAsDouble() > rightSilver.getAsDouble()){
                            finalPos = MineralPosition.RIGHT;
                        }
                        else{
                            finalPos = MineralPosition.CENTER;
                        }
                        Log.d(TAG, "Average Gold Position: " + avgGold.getAsDouble());
                        Log.d(TAG, "Right Silver Position: " + rightSilver.getAsDouble());
                        Log.d(TAG, "Left Silver Position: " + leftSilver.getAsDouble());
                    }
                    else if (avgGold.isPresent() && leftSilver.isPresent()){
                        if (avgGold.getAsDouble() < leftSilver.getAsDouble()){
                            finalPos = MineralPosition.LEFT;
                        }
                        else{
                            finalPos = MineralPosition.CENTER;
                        }
                        Log.d(TAG, "Average Gold Position: " + avgGold.getAsDouble());
                        Log.d(TAG, "Left Silver Position: " + leftSilver.getAsDouble());
                    }
                    else if (avgGold.isPresent() && rightSilver.isPresent()){
                        if (avgGold.getAsDouble() > rightSilver.getAsDouble()){
                            finalPos = MineralPosition.RIGHT;
                        }
                        else{
                            finalPos = MineralPosition.CENTER;
                        }
                        Log.d(TAG, "Average Gold Position: " + avgGold.getAsDouble());
                        Log.d(TAG, "Right Silver Position: " + rightSilver.getAsDouble());
                    }
                    else {
                        finalPos = MineralPosition.CENTER;
                    }
                    //endregion
                }
                catch (Exception ex){
                    Log.w(TAG, "Exception " + ex + " Encountered in Locating Mineral, defaulting to center.");
                    finalPos = MineralPosition.CENTER;
                }
                finally {
                    elapsedTime.reset();
                    Log.i(TAG, "Minerals evaluated, mineral is in the " + finalPos.toString() + " Position.");
                    state = CurrentState.TURN_TO_MINERAL;
                }

                break;

            case TURN_TO_MINERAL:
                switch (finalPos) {
                    case CENTER:
                        robot.leftWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        state = CurrentState.REMOVE_MINERAL;
                        break;
                    case LEFT:
                        robot.leftWheel.setPower(-0.3);
                        robot.rightWheel.setPower(0.3);
                        Log.v(TAG, "Now turning to left mineral, robot heading is: " + robot.heading() + ", Target Heading is: " + LEFT_MINERAL_THETA);
                        if (robot.heading() > LEFT_MINERAL_THETA){
                            robot.leftWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                            state = CurrentState.REMOVE_MINERAL;
                        }
                    case RIGHT:
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
                    switch (fieldPosition){
                        case DEPOT:
                            state = CurrentState.DEPOT_TURN_TO_DEPOT;
                            break;
                        case CRATER:
                            state = CurrentState.CRATER_BACKING_UP;
                            break;
                    }
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

            case DEPOT_TURN_TO_DEPOT:
                switch (finalPos){
                    case CENTER:
                        robot.leftWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        state = CurrentState.DEPOT_DRIVE_TO_DEPOT;
                        break;
                    case LEFT:
                        robot.leftWheel.setPower(0.1);
                        robot.rightWheel.setPower(-0.1);
                        Log.v(TAG, "Now turning to the depot, robot heading is: " + robot.heading() + ", Target Heading is: " + -DEPOT_THETA);
                        if (robot.heading() < -20){
                            robot.leftWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                            state = CurrentState.DEPOT_DRIVE_TO_DEPOT;
                        }
                        break;
                    case RIGHT:
                        robot.leftWheel.setPower(-0.1);
                        robot.rightWheel.setPower(0.1);
                        Log.v(TAG, "Now turning to the depot, robot heading is: " + robot.heading() + ", Target Heading is: " + DEPOT_THETA);
                        if (robot.heading() > DEPOT_THETA){
                            robot.leftWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                            state = CurrentState.DEPOT_DRIVE_TO_DEPOT;
                        }
                        break;
                    default:
                        robot.leftWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        state = CurrentState.DEPOT_DRIVE_TO_DEPOT;
                        break;
                }
                break;


            case DEPOT_DRIVE_TO_DEPOT:
                robot.leftWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                if (robot.leftWheel.getCurrentPosition()>=1800){
                    robot.rightWheel.setPower(0.0);
                    robot.leftWheel.setPower(0.0);
                    state = CurrentState.DEPOT_TURN_TO_CRATER;
                }
                else {
                    robot.leftWheel.setPower(.3);
                    robot.rightWheel.setPower(.3);
                }
                break;

            case DEPOT_TURN_TO_CRATER:
                if (robot.heading() > 85){
                    robot.rightWheel.setPower(0.0);
                    robot.leftWheel.setPower(0.0);
                    robot.leftWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    state = CurrentState.DEPOT_DRIVE_TO_CRATER;
                }
                else {
                    robot.rightWheel.setPower(0.3);
                    robot.leftWheel.setPower(-0.3);
                }
                break;

            case  DEPOT_DRIVE_TO_CRATER:
                robot.leftWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                if (robot.leftWheel.getCurrentPosition()>=4000){
                    robot.rightWheel.setPower(0.0);
                    robot.leftWheel.setPower(0.0);
                    state = CurrentState.STOP;
                }
                else {
                    robot.driveByGyro(0.4, 130);
                }
                break;

            case STOP:
                robot.leftWheel.setPower(0.0);
                robot.rightWheel.setPower(0.0);
                robot.extension.setPower(0.0);
                break;
        }


    }

    /*
    NOTES FOR FUTURE
    Goals:
        + Replace Time moving forward with encoder counts
        + Verify functionality of heading methods
        +
     */



    @Override
    public void stop() {

        if (this.timer != null){

            Log.i(TAG, "Stopping timer");
            this.timer.shutdownNow();
        }

        robot.detector.stop();

    }
}

package org.firstinspires.ftc.team7234.RoverRuckus.common;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.team7234.RoverRuckus.common.Imaging.ContourDetector;
import org.firstinspires.ftc.team7234.RoverRuckus.common.Imaging.MineralDetector;

public class HardwareBotman {

    public HardwareBotman(){}

    //region Members
    public DcMotor rightWheel;
    public DcMotor leftWheel;
    public DcMotor extension;


    public DcMotor collectorSpinner;
    public DcMotor armLift;
    public DcMotor armTwist;



    BNO055IMU imu;
    Orientation angles;

    MineralDetector detector;

    //endregion

    /* local OpMode members. */
    private HardwareMap hwMap   = null;
    private ElapsedTime period  = new ElapsedTime();

    public int time;

    private final String logTag = HardwareBotman.class.getName();

    public void init(HardwareMap ahwMap, boolean useCamera){
        if (useCamera){
            detector = new ContourDetector();
        }
        init(ahwMap);
    }

    public void init(HardwareMap ahwMap){

        period.reset();
        //Save reference to Hardware map
        hwMap = ahwMap;

        //Define and initialize Motors
        leftWheel = hwMap.get(DcMotor.class, "left_drive");
        rightWheel = hwMap.get(DcMotor.class, "right_drive");
        extension = hwMap.get(DcMotor.class, "latch");

        collectorSpinner = hwMap.get(DcMotor.class, "collector");
        armLift = hwMap.get(DcMotor.class, "arm_lift");
        armTwist = hwMap.get(DcMotor.class, "arm_twist");


        leftWheel.setDirection(DcMotor.Direction.REVERSE);
        rightWheel.setDirection(DcMotor.Direction.FORWARD);

        //Set all motors to zero power
        leftWheel.setPower(0.);
        rightWheel.setPower(0.);

        //Set ZeroPowerBehaviors
        armTwist.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);



        //resets encoders
        resetEncoders();
        //Define and initialize servos

        //Define sensors

        //Setup IMU
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hwMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        time = (int)period.milliseconds();
    }

    public double heading(){
        updateAngles();
        return angles.thirdAngle; //Rotation about Y axis, out from short side of hub
    }
    public double roll(){
        updateAngles();
        return angles.firstAngle; //Rotation about Z axis, out from top of hub
    }
    public double pitch(){
        updateAngles();
        return angles.secondAngle;
    }

    private void updateAngles(){
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZXY, AngleUnit.DEGREES);
    }



    public void resetEncoders(){
        //Stop the motors and reset the encoders
        leftWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extension.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armTwist.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        /*
        The left and right wheels are not hooked up to encoders so we cannot use them to drive
        these motors.  We do have one hooked up to the linear actuator.  So, we can use one to
        drive it
         */
        leftWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        extension.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armTwist.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    /*
    This block of code helps the robot follow a straight path while using the gyro.  If the robot
    rotates too far one way, then it will correct itself and continue following this path.
     */
    public void driveByGyro(double speed, double header){
        if(speed > 0.9 || speed < -0.9) {
            throw new IllegalArgumentException("Speed must be between 0.9 and -0.9" + speed);
        }
        if (heading() > header + 3) {
            leftWheel.setPower(speed + 0.1);
            rightWheel.setPower(speed - 0.1);
        }
        else if (heading() < header - 3) {
            leftWheel.setPower(speed - 0.1);
            rightWheel.setPower(speed + 0.1);
        }
        else{
            leftWheel.setPower(speed);
            rightWheel.setPower(speed);
        }

    }


}

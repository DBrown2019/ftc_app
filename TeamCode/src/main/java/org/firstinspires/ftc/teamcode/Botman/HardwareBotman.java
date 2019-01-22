package org.firstinspires.ftc.teamcode.Botman;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

public class HardwareBotman {

    public HardwareBotman(){}

    //region Members
    private static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    private static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    private static final String LABEL_SILVER_MINERAL = "Silver Mineral";
    private static final String VUFORIA_KEY = "AcZlc3n/////AAAAGWPeDCNLuk38gPuwF9cpyK2BYbGciGSeJy9AkSXPprQUEtg/VxgqB6j9WJuQvGo4pq+h4gwPSd134WD707FXnbuJjqdqkh5/92mATPs96WQ2RVoaU8QLbsJonufIl2T6qqqT83aOJHbz34mGJszad+Mw7VAWM11av5ltOoq8/rSKbmSFxAVi3d7oiT3saE0XBx4svhpGLwauy6Y0L7X0fC7FwHKCnw/RPL4V+Q8v2rtCTOwvjfnjxmRMind01HSWcxd9ppBwzvHVCPhePccnyWVv5jNiYXia9r4FlrJpAPgZ1GsCfdbt6AoT6Oh2Hnx267J+MHUnLi/C+0brvnQfcDregLBfnZApfd2c1WDiXJp/";

    private VuforiaLocalizer vuforia;
    public TFObjectDetector tfod;

    byte AXIS_MAP_CONFIG_BYTE = 0x18; //This is what to write to the AXIS_MAP_CONFIG register to swap x and z axes
    byte AXIS_MAP_SIGN_BYTE = 0x00; //This is what to write to the AXIS_MAP_SIGN register to negate the z axis

    public DcMotor rightWheel;
    public DcMotor leftWheel;
    public DcMotor extension;
    //public DcMotor armExtension;
    public DcMotorSimple collector;
    //public DcMotorSimple Elbow;

    BNO055IMU imu;
    Orientation angles;

    public minerals mineralState = minerals.GOLD_LEFT;

    public enum minerals {
        GOLD_LEFT,
        GOLD_CENTER,
        GOLD_RIGHT,
        GOLD_NOT_FOUND
    }

    //endregion

    /* local OpMode members. */
    private HardwareMap hwMap   = null;
    private ElapsedTime period  = new ElapsedTime();

    public int time;

    private final String logTag = HardwareBotman.class.getName();

    public void init(HardwareMap ahwMap, boolean useCamera){
        hwMap = ahwMap;
        if (useCamera){
            initVuforia();
            initTfod();

            if (tfod != null) {
                tfod.activate();
            }
        }

        period.reset();
        //Save reference to Hardware map


        //Define and initialize Motors
        leftWheel = hwMap.get(DcMotor.class, "left_drive");
        rightWheel = hwMap.get(DcMotor.class, "right_drive");
        extension = hwMap.get(DcMotor.class, "latch");
        //armExtension=hwMap.get(DcMotor.class,"arm_lift");
        //Elbow=hwMap.get(DcMotorSimple.class, "arm_twist");
        collector=hwMap.get(DcMotorSimple.class, "collector");



        leftWheel.setDirection(DcMotor.Direction.FORWARD);
        rightWheel.setDirection(DcMotor.Direction.REVERSE);

        //Set all motors to zero power
        leftWheel.setPower(0);
        rightWheel.setPower(0);
        extension.setPower(0);


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
        return angles.firstAngle; //Rotation about Z axis
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
    }

    public void RunByEncoders(){
        //Stop the motors and reset the encoders
        leftWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        extension.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void DriveWithoutEncoders(){
        //Stop the motors and reset the encoders
        leftWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    /*
    This block of code helps the robot follow a straight path while using the gyro.  If the robot
    rotates too far one way, then it will correct itself and continue following this path.
     */
    public void driveByGyro(double speed, double header){
        if(speed > 0.9 || speed < -0.9) {
            throw new IllegalArgumentException("Nah fam, keep it between -0.9 and 0.9" + speed);
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

    /**
     * Initialize the Vuforia localization engine.
     */
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = CameraDirection.BACK;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the Tensor Flow Object Detection engine.
    }

    /**
     * Initialize the Tensor Flow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hwMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hwMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL, LABEL_SILVER_MINERAL);
    }

    /**
     * Tensor Flow detect the gold mineral location
     */
    public void ObjectDetetion(Telemetry telemetry){
        if (tfod != null) {
            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
            if (updatedRecognitions != null) {
                telemetry.addData("# Object Detected", updatedRecognitions.size());
                int goldMineralX = -100;
                int goldMineralY=-100;
                for (Recognition recognition : updatedRecognitions) {
                    if (recognition.getLabel().equals(LABEL_GOLD_MINERAL)) {
                        if(recognition.getTop()>900) {
                            goldMineralX = (int) recognition.getLeft();
                            goldMineralY = (int) recognition.getTop();
                        }
                    }
                }
                telemetry.addData("Gold Mineral X Value",goldMineralX);
                telemetry.addData("Gold Mineral Y Value",goldMineralY);
                if (goldMineralX == -100) {
                    mineralState = minerals.GOLD_LEFT;
                    telemetry.addData("Gold Mineral Position", "Left");
                } else if (goldMineralX > 345) {
                    mineralState = minerals.GOLD_RIGHT;
                    telemetry.addData("Gold Mineral Position", "Right");
                } else if (goldMineralX < 345){
                    mineralState = minerals.GOLD_CENTER;
                    telemetry.addData("Gold Mineral Position", "Center");
                }

            }
        }
    }

    public void gyroConfigMode() {
        //Need to be in CONFIG mode to write to registers
        imu.write8(BNO055IMU.Register.OPR_MODE,BNO055IMU.SensorMode.CONFIG.bVal & 0x0F);
    }

    public void changeAxis() {
        //Write to the AXIS_MAP_CONFIG register
        imu.write8(BNO055IMU.Register.AXIS_MAP_CONFIG,AXIS_MAP_CONFIG_BYTE & 0b111111);

        //Write to the AXIS_MAP_SIGN register
        imu.write8(BNO055IMU.Register.AXIS_MAP_SIGN,AXIS_MAP_SIGN_BYTE & 0b111);
    }

    public void gyroReadMode() {
        //Need to change back into the IMU mode to use the gyro
        imu.write8(BNO055IMU.Register.OPR_MODE, BNO055IMU.SensorMode.IMU.bVal & 0b1111);
    }
}

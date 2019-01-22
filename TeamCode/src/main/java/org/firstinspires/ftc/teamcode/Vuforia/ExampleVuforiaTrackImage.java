/* Copyright (c) 2015 Qualcomm Technologies Inc

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Qualcomm Technologies Inc nor the names of its contributors
may be used to endorse or promote products derived from this software without
specific prior written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. */

package org.firstinspires.ftc.teamcode.Vuforia;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

@Autonomous(name = "Vuforia Track Andy Mark", group = "Vuforia")
@Disabled

public class ExampleVuforiaTrackImage extends OpMode {
    private VuforiaLocalizer vuforiaLocalizer;
    private VuforiaLocalizer.Parameters parameters;
    private VuforiaTrackables relicImages;
    private VuforiaTrackable relicRecoveryImageTarget;
    private VuforiaTrackable andyMarkTarget;
    private VuforiaTrackable relicRecoveryButtonTarget;
    private VuforiaTrackableDefaultListener relicRecoveryImageListener;
    private VuforiaTrackableDefaultListener andyMarkListener;
    private VuforiaTrackableDefaultListener relicRecoveryButtonListener;

    //public static final String TAG = "Vuforia Sample";

    private OpenGLMatrix lastLocation;
    private OpenGLMatrix phoneLocation;
    private OpenGLMatrix robotLocationTransform;

    double rightLeft = 0;          //Direction the robot should move
    double upDown = 0;              //The speed the robot should move
    float posX, posY, posZ, angleX, angleY, angleZ;

    @Override
    public void init() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId); // To remove the camera view from the screen, remove the cameraMonitorViewId
        parameters.vuforiaLicenseKey = "AcZlc3n/////AAAAGWPeDCNLuk38gPuwF9cpyK2BYbGciGSeJy9AkSXPprQUEtg/VxgqB6j9WJuQvGo4pq+h4gwPSd134WD707FXnbuJjqdqkh5/92mATPs96WQ2RVoaU8QLbsJonufIl2T6qqqT83aOJHbz34mGJszad+Mw7VAWM11av5ltOoq8/rSKbmSFxAVi3d7oiT3saE0XBx4svhpGLwauy6Y0L7X0fC7FwHKCnw/RPL4V+Q8v2rtCTOwvjfnjxmRMind01HSWcxd9ppBwzvHVCPhePccnyWVv5jNiYXia9r4FlrJpAPgZ1GsCfdbt6AoT6Oh2Hnx267J+MHUnLi/C+0brvnQfcDregLBfnZApfd2c1WDiXJp/";
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.FRONT;
        parameters.useExtendedTracking = false;
        vuforiaLocalizer = ClassFactory.createVuforiaLocalizer(parameters);

        // These are the vision targets that we want to use
        // The string needs to be the name of the appropriate .xml file in the assets folder
        relicImages = vuforiaLocalizer.loadTrackablesFromAsset("RelicKickoff");

        // Setup the targets to be tracked
        relicRecoveryImageTarget = relicImages.get(0);
        relicRecoveryImageTarget.setName("RelicImage");
        relicRecoveryImageTarget.setLocation(createMatrix(0, 0, 0, 0, 0 ,0));

        andyMarkTarget = relicImages.get(1);
        andyMarkTarget.setName("AndyMark");
        andyMarkTarget.setLocation(createMatrix(0, 0, 0, 0, 0 ,0));

        relicRecoveryButtonTarget = relicImages.get(2);
        relicRecoveryButtonTarget.setName("RelicButton");
        relicRecoveryButtonTarget.setLocation(createMatrix(0, 0, 0, 0, 0 ,0));


        // Set phone location on robot
        phoneLocation = createMatrix(0, 0, 0, 0, 180, -90);

        /**
         * Let the trackable listeners we care about know where the phone is. We know that each
         * listener is a {@link VuforiaTrackableDefaultListener} and can so safely cast because
         * we have not ourselves installed a listener of a different type.
         */
        relicRecoveryImageListener = (VuforiaTrackableDefaultListener) relicRecoveryImageTarget.getListener();
        relicRecoveryImageListener.setPhoneInformation(phoneLocation, parameters.cameraDirection);
        andyMarkListener = (VuforiaTrackableDefaultListener) andyMarkTarget.getListener();
        andyMarkListener.setPhoneInformation(phoneLocation, parameters.cameraDirection);
        relicRecoveryButtonListener = (VuforiaTrackableDefaultListener) relicRecoveryButtonTarget.getListener();
        relicRecoveryButtonListener.setPhoneInformation(phoneLocation, parameters.cameraDirection);

        lastLocation = createMatrix(0, 0, 0, 0, 0, 0);
    }


    @Override
    public void init_loop() {

    }


    @Override
    public void start() {
        relicImages.activate();//Start tracking the data sets we care about.
        robotLocationTransform = relicRecoveryImageListener.getUpdatedRobotLocation();
    }


    @Override
    public void loop() {

        if (relicRecoveryImageListener.isVisible()) {
            robotLocationTransform = relicRecoveryImageListener.getUpdatedRobotLocation();
        } else if (andyMarkListener.isVisible()) {
            robotLocationTransform = andyMarkListener.getUpdatedRobotLocation();
        } else if (relicRecoveryButtonListener.isVisible()) {
            robotLocationTransform = relicRecoveryButtonListener.getUpdatedRobotLocation();
        }

        if (robotLocationTransform != null) {
            lastLocation = robotLocationTransform;

            VectorF trans = robotLocationTransform.getTranslation();

            posX = trans.get(0);
            posY = trans.get(1);
            posZ = trans.get(2);
            angleX = Orientation.getOrientation(lastLocation, AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).firstAngle;
            angleY = Orientation.getOrientation(lastLocation, AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).secondAngle;
            angleZ = Orientation.getOrientation(lastLocation, AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle;

            //Calculate the desired rotate direction from Vuforia
            rightLeft = angleY - (Math.toDegrees(Math.atan2(posZ, -posX))-90);
            upDown =  angleX - (Math.toDegrees(Math.atan2(posZ, posY))-90);
        }

        // send the info back to driver station using telemetry function.
        telemetry.addData("Tilt Up Down", upDown);
        telemetry.addData("Pan Right Left", rightLeft);
        telemetry.addData(relicRecoveryImageTarget.getName(), relicRecoveryImageListener.isVisible() ? "Visible" : "Not Visible");
        telemetry.addData(andyMarkTarget.getName(), andyMarkListener.isVisible() ? "Visible" : "Not Visible");
        telemetry.addData(relicRecoveryButtonTarget.getName(), relicRecoveryButtonListener.isVisible() ? "Visible" : "Not Visible");
    }


    @Override
    public void stop() {

    }

    // Creates a matrix for determining the locations and orientations of objects
    // Units are millimeters for x, y, and z, and degrees for u, v, and w
    private OpenGLMatrix createMatrix(float x, float y, float z, float u, float v, float w)
    {
        return OpenGLMatrix.translation(x, y, z).
                multiplied(Orientation.getRotationMatrix(
                        AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES, u, v, w));
    }

    /**
     * A simple utility that extracts positioning information from a transformation matrix
     * and formats it in a form palatable to a human being.
     */
    String formatMatrix(OpenGLMatrix transformationMatrix) {
        return transformationMatrix.formatAsTransform();
    }
}
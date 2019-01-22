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

package org.firstinspires.ftc.team7234;


import android.graphics.Bitmap;
import android.os.Environment;
import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ThreadPool;
import com.vuforia.Frame;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.function.Consumer;
import org.firstinspires.ftc.robotcore.external.function.Continuation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.opencv.android.Utils;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;

import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

@Autonomous(name = "OpenCV Read Block", group = "Vuforia")
@Disabled

public class openCVBlockDetection extends OpMode {
    private VuforiaLocalizer vuforiaLocalizer;
    private VuforiaLocalizer.Parameters parameters;

    Bitmap bitmap = null;
    Boolean buttonPressed = false;

    //Scaler values for HSV.  These work much better than BRG
    public final static Scalar yellowLow = new Scalar(0, 150, 150);  //
    public final static Scalar yellowHigh = new Scalar(60, 255, 255);
    public final static Scalar whiteLow = new Scalar(0, 0, 150);
    public final static Scalar whiteHigh = new Scalar(180, 10, 255);
    //Scaler values for BGR.  Not very good for yellow
    /*public final static Scalar yellowLow = new Scalar(0, 150, 150);  //
    public final static Scalar yellowHigh = new Scalar(100, 255, 255);
    public final static Scalar whiteLow = new Scalar(200, 100, 100);
    public final static Scalar whiteHigh = new Scalar(255, 255, 255);*/

    minerals mineralState = minerals.GOLD_NOT_FOUND;

    public enum minerals {
        GOLD_LEFT,
        GOLD_CENTER,
        GOLD_RIGHT,
        GOLD_NOT_FOUND
    }


    @Override
    public void init() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);// To add the camera view from the screen, add cameraMonitorViewId
        parameters.vuforiaLicenseKey = "AcZlc3n/////AAAAGWPeDCNLuk38gPuwF9cpyK2BYbGciGSeJy9AkSXPprQUEtg/VxgqB6j9WJuQvGo4pq+h4gwPSd134WD707FXnbuJjqdqkh5/92mATPs96WQ2RVoaU8QLbsJonufIl2T6qqqT83aOJHbz34mGJszad+Mw7VAWM11av5ltOoq8/rSKbmSFxAVi3d7oiT3saE0XBx4svhpGLwauy6Y0L7X0fC7FwHKCnw/RPL4V+Q8v2rtCTOwvjfnjxmRMind01HSWcxd9ppBwzvHVCPhePccnyWVv5jNiYXia9r4FlrJpAPgZ1GsCfdbt6AoT6Oh2Hnx267J+MHUnLi/C+0brvnQfcDregLBfnZApfd2c1WDiXJp/";
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        parameters.useExtendedTracking = false;
        vuforiaLocalizer = ClassFactory.getInstance().createVuforia(parameters);
        vuforiaLocalizer.enableConvertFrameToBitmap();
    }


    @Override
    public void init_loop() { }


    @Override
    public void start() {  }


    @Override
    public void loop() {
        vuforiaLocalizer.getFrameOnce(Continuation.create(ThreadPool.getDefault(), new Consumer<Frame>()
        {
            @Override public void accept(Frame frame)
            {
                bitmap = vuforiaLocalizer.convertFrameToBitmap(frame);
            }
        }));


        mineralState = getMineralState(bitmap);


        telemetry.addData("Mineral State", mineralState.toString());
    }


    @Override
    public void stop() {

    }


    //method to analyze the picture grabbed from Vuforia and return the Gold location
    public minerals getMineralState(Bitmap bm) {
        if(bm != null) {

            Mat mineralPic = new Mat(bm.getHeight(), bm.getWidth(), CvType.CV_8UC3);
            Utils.bitmapToMat(bm, mineralPic);

            Imgproc.cvtColor(mineralPic, mineralPic, Imgproc.COLOR_RGB2HSV);
            Imgproc.GaussianBlur(mineralPic, mineralPic, new Size(9, 9), 2, 2);

            Mat yellowMask = new Mat();
            Core.inRange(mineralPic, yellowLow, yellowHigh, yellowMask);
            Imgproc.GaussianBlur(yellowMask, yellowMask, new Size(9, 9), 2, 2);

            List<MatOfPoint> yellowContours = new ArrayList<MatOfPoint>();
            Point blockCenter = new Point();

            Imgproc.findContours(yellowMask, yellowContours, new Mat(), Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);

            double blockArea = 0;
            blockCenter.x = 0;
            blockCenter.y = 0;

            for (int i = 0; i < yellowContours.size(); i++) {
                MatOfPoint currentContour = yellowContours.get(i);
                double currentArea = Imgproc.contourArea(currentContour);

                if (currentArea > blockArea) {
                    blockArea = currentArea;
                }

                if (currentArea > 10000) {
                    Moments blockMoments = Imgproc.moments(currentContour, false);
                    blockCenter.x = blockMoments.get_m10() / blockMoments.get_m00();
                    blockCenter.y = blockMoments.get_m01() / blockMoments.get_m00();
                }
            }

            telemetry.addData("Area", blockArea);
            telemetry.addData("X Pos", blockCenter.x / yellowMask.width());
            telemetry.addData("Y Pos", blockCenter.y / yellowMask.height());

            if (gamepad1.x && !buttonPressed) {
                Bitmap bmOutYellow = Bitmap.createBitmap(yellowMask.width(), yellowMask.height(), Bitmap.Config.RGB_565);
                Utils.matToBitmap(yellowMask, bmOutYellow);
                buttonPressed = true;
                saveImage(bmOutYellow, "yellow");
            } else if (!gamepad1.x && buttonPressed) {
                buttonPressed = false;
            }

            if (blockCenter.x == 0) {
                return mineralState.GOLD_LEFT;
            } else if (blockCenter.x / yellowMask.width() < .5) {
                return mineralState.GOLD_CENTER;
            } else if (blockCenter.x / yellowMask.width() > .5) {
                return mineralState.GOLD_RIGHT;
            }
        }

        return mineralState.GOLD_NOT_FOUND;
    }


    public void saveImage(Bitmap bitmap, String stringout) {
        if (bitmap != null) {
            String nameOfFile = "imageCaptured" + stringout;

            String state = Environment.getExternalStorageState();
            if (Environment.MEDIA_MOUNTED.equals(state)) {
                Log.i("Mike", "Able to write to storage");
            } else {
                Log.i("Mike", "Cannot write to storage");
            }


            File dir = Environment.getExternalStoragePublicDirectory(Environment.DIRECTORY_PICTURES);

            Log.i("Mike", "saveImage: " + dir.getPath());

            if (!dir.exists()) {
                Log.i("Mike", "Dir does not exist");
            } else {
                Log.i("Mike", "Dir Exists");

                File file = new File(dir, nameOfFile + ".jpg");

                try {
                    FileOutputStream fOut = new FileOutputStream(file);
                    bitmap.compress(Bitmap.CompressFormat.JPEG, 100, fOut);

                    fOut.flush();
                    fOut.close();

                } catch (FileNotFoundException e) {
                } catch (IOException e) {
                    Log.i("Mike", e.toString());
                }
            }
        }
    }


}
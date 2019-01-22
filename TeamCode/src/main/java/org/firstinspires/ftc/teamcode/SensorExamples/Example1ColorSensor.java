/* Copyright (c) 2014, 2015 Qualcomm Technologies Inc

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

package org.firstinspires.ftc.teamcode.SensorExamples;

import android.graphics.Color;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.I2cAddr;

/**
 * Demonstrates how to setup and use 1 MR color sensor
 */
@Autonomous(name = "Read Color Sensor with HSV", group = "SensorExamples")
@Disabled
public class Example1ColorSensor extends OpMode {

    ColorSensor colorSensor;
    //Array to hold the HSV values
    float hsvValues[] = {0F,0F,0F};

    @Override
    public void init() {
        colorSensor = hardwareMap.colorSensor.get("colorsensor");
        //This can be used to use a different address for the MR Color Sensor
        //It comes from the factory with 0x3C, but if you want to more than one
        //you need ot change the reference address on one of them.
        //colorSensor.setI2cAddress(I2cAddr.create8bit(0x3C));  //MR robotics color default address
        //colorSensor.setI2cAddress(I2cAddr.create8bit(0x39));    //Rev robotics color default address
        //This command is used to turn the color sensor LED on and off.  In this
        //statement the LED is turned off.
        colorSensor.enableLed(false);

        telemetry.addData("Status", "Initialized");
    }


    @Override
    public void init_loop() { }


    @Override
    public void start() { }


    @Override
    public void loop() {
        //Pressing "A" on gamepad 1 will turn the LED off.  When not pressed it will be on.
        if(gamepad1.a) {
            colorSensor.enableLed(false);
        } else {
            colorSensor.enableLed(true);
        }
        //Ths Java method will convert RGB values to HSV.  The values need to multiplied to have the correct input range
        //Color.RGBToHSV(colorSensor.red() * 8, colorSensor.green() * 8, colorSensor.blue() * 8, hsvValues); //For MR HSV conversion
        Color.RGBToHSV(colorSensor.red() * 255, colorSensor.green() * 255, colorSensor.blue() * 255, hsvValues); //For Rev HSV conversion

        telemetry.addData("0", "Press A on Gamepad1 to turn off LED");
        telemetry.addData("1", "Red: " + colorSensor.red());
        telemetry.addData("2", "Green: " + colorSensor.green());
        telemetry.addData("3", "Blue: " + colorSensor.blue());
        telemetry.addData("4", "Alpha: " + colorSensor.alpha());
        telemetry.addData("5", "Hue: " + hsvValues[0]);
        telemetry.addData("6", "Sat: " + hsvValues[1]);
        telemetry.addData("7", "Val: " + hsvValues[2]);
    }

    @Override
    public void stop() {}
}

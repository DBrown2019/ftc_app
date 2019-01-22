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

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoControllerEx;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "Example Servo with Joystick", group = "Example")
@Disabled

public class ExampleServo extends OpMode {

    Servo myServo;
    private ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
    private boolean isPressed = false;
    private double servoPosition = 0;

    @Override
    public void init() {
        myServo = hardwareMap.servo.get("servo name");

        ServoControllerEx theControl = (ServoControllerEx)myServo.getController();
        int thePort = myServo.getPortNumber();
        PwmControl.PwmRange theRange = new PwmControl.PwmRange(553,2500);
        theControl.setServoPwmRange(thePort,theRange);

        myServo.setPosition(1);

        timer.reset();
    }

    @Override
    public void init_loop() {


        telemetry.addData("Status:", "Robot is Initialized");
        telemetry.addData("Press Start ", "To control the servo with the joystick.");
    }


    @Override
    public void loop() {
        if(gamepad1.y && !isPressed){
            servoPosition += 0.1;
            isPressed = true;
        } else if(gamepad1.b && !isPressed){
            servoPosition += 0.01;
            isPressed = true;
        } else if(gamepad1.a && !isPressed){
            servoPosition -= 0.1;
            isPressed = true;
        } else if(gamepad1.x && !isPressed){
            servoPosition -= .01;
            isPressed = true;
        } else if(!gamepad1.a && !gamepad1.b && !gamepad1.y && !gamepad1.x){
            isPressed = false;
        }

        myServo.setPosition(servoPosition);
        // send the info back to driver station using telemetry function.
        telemetry.addData("Press Y ", "To increase the servo .1");
        telemetry.addData("Press Y ", "To increase the servo .01");
        telemetry.addData("Press A ", "To decrease the servo .1");
        telemetry.addData("Press X ", "To decrease the servo .01");
        telemetry.addData("Servo setpoint", servoPosition);

    }


    @Override
    public void stop() {

    }
}
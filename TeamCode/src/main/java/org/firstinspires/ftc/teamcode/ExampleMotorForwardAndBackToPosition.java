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
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "Example Motor Forward and Back", group = "Example")
@Disabled

public class ExampleMotorForwardAndBackToPosition extends OpMode {

    DcMotor Motor;


    static final double     countsPerMotorRev       = 1120;
    static final double     rotations               = 3;

    private int             targetPosition          = 0;
    private boolean         cycled                  = false;

    @Override
    public void init() {
        Motor = hardwareMap.dcMotor.get("motor");

        Motor.setDirection(DcMotor.Direction.FORWARD);
        Motor.setTargetPosition(targetPosition);
        Motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    @Override
    public void init_loop() {
        telemetry.addData("Status:", "Robot is Initialized");
        telemetry.addData("1", "Motor Pos: " + Motor.getCurrentPosition());
    }


    @Override
    public void start() {
        Motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }


    @Override
    public void loop() {
        if (!Motor.isBusy() && !cycled) {
            if (targetPosition == 0) {
                targetPosition = (int) (rotations * countsPerMotorRev);
                Motor.setTargetPosition(targetPosition);
            }
            else {
                targetPosition = 0;
                Motor.setTargetPosition(targetPosition);
                cycled = true;
            }
        } else if(!Motor.isBusy()) {
            targetPosition = (int) (rotations * countsPerMotorRev);
            Motor.setTargetPosition(targetPosition);
        }


        Motor.setPower(.05);

        // send the info back to driver station using telemetry function.

        telemetry.addData("1", "Motor Power: " + Motor.getPower());
        telemetry.addData("2", "Motor Position: " + Motor.getCurrentPosition());
        telemetry.addData("3", "Target Position: " + Motor.getTargetPosition());
        telemetry.addData("4", "Motor Mode: " + Motor.getMode());
        telemetry.addData("5", "Zero Power Mode: " + Motor.getZeroPowerBehavior());
        telemetry.addData("6", "Motor Info: " + Motor.getConnectionInfo());
    }


    @Override
    public void stop() {
        Motor.setPower(0);
    }
}
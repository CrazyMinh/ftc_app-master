/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;

/**
 * This OpMode uses the common Pushbot hardware class to define the devices on the robot.
 * All device access is managed through the HardwarePushbot class.
 * The code is structured as a LinearOpMode
 *
 * This particular OpMode executes a POV Game style Teleop for a PushBot
 * In this mode the left stick moves the robot FWD and back, the Right stick turns left and right.
 * It raises and lowers the claw using the Gampad Y and A buttons respectively.
 * It also opens and closes the claws slowly using the left and right Bumper buttons.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="Pushbot: Saint Johnny the 5th", group="Pushbot")
@Disabled
public class Pushbot_SafeAutomonousOperations extends LinearOpMode {

    //Hardware Device Objects
    private DigitalChannel Switch_Air;  // Hardware Device Object
    private DigitalChannel Switch_Ground;  // Hardware Device Object

    /* Declare OpMode members. */
    HardwarePushbot robot           = new HardwarePushbot();   // Use a Pushbot's hardware
    private ElapsedTime runtime     = new ElapsedTime();


    @Override
    public void runOpMode() {
        double left = 1;
        double right = 1;

        // get a reference to our digitalTouch object.
        Switch_Air = hardwareMap.get(DigitalChannel.class, "sensor_High");

        // set the digital channel to input.
        Switch_Air.setMode(DigitalChannel.Mode.INPUT);


        // get a reference to our digitalTouch object.
        Switch_Ground = hardwareMap.get(DigitalChannel.class, "sensor_Loh");

        // set the digital channel to input.
        Switch_Ground.setMode(DigitalChannel.Mode.INPUT);

        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Skynet Online. Mission Brief: WIN THE GAME AND WIN THE DAY.");    //
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end (End of Automonous Period)
        while (opModeIsActive()) {

            while (opModeIsActive() && (runtime.seconds() <= 10.0)) {
                robot.leftDrive.setPower(left);
                robot.rightDrive.setPower(right);
                while (opModeIsActive() && (runtime.seconds() <= 10.0)) {
                    telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
                    telemetry.update();
                }
            }

            while (opModeIsActive() && (runtime.seconds() >= 11.0) && (runtime.seconds() <= 20.0)) {
                robot.leftDrive.setPower(-left);
                robot.rightDrive.setPower(right);
                while (opModeIsActive() && (runtime.seconds() > 11.0) && (runtime.seconds() <= 20.0)) {
                    telemetry.addData("Path", "Leg 2: %2.5f S Elapsed", runtime.seconds());
                    telemetry.update();
                }
            }
            while (opModeIsActive() && (runtime.seconds() > 21.0) && (runtime.seconds() <= 30. -)) {
                robot.leftDrive.setPower(left);
                robot.rightDrive.setPower(right);
                while (opModeIsActive() && (runtime.seconds() > 21.0) && (runtime.seconds() <= 30. -)) {
                    telemetry.addData("Path", "Leg 3: %2.5f S Elapsed", runtime.seconds());
                    telemetry.update();
                }
            }
        }

        runtime.reset();
            // Pace this loop so jaw action is reasonable speed.
            sleep(50);
        }
    }
}

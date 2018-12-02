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
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp(name="Previous Year", group="Iterative Opmode")
@Disabled
public class PreviousYear extends OpMode
{
    private ElapsedTime runtime = new ElapsedTime();

    //
    //private Servo glyphLeftDown = null;
    private Servo relicGrabber = null;
    private Servo relicClaw = null;
    private Servo activator = null;
    private Servo shoulderleft = null;

    private DcMotor leftFront = null;
    private DcMotor rightFront = null;
    private DcMotor leftBack = null;
    private DcMotor rightBack = null;
    private DcMotor lift = null;
    private DcMotor relicMotor = null;
    private DcMotor intake = null;


    private float x1, x2, y1, y2;

    private boolean grabberUp = true;
    private boolean clawClosed = false;

    private double grabberunpressed = 0.0;
    private double clawunpressed = 0.0;

    private double dpadunpressed = 0.0;

    private double GRABBERUP = 1.0;
    private double GRABBERDOWN = 0.29;
    private double GRABBERINIT = 0.0;
    private double GRABBERCURRENT = GRABBERDOWN;
    private double CLAWCLOSE = 0.85;
    private double CLAWOPEN = 0.35;
    private double CLAWINIT = 0.35;
    private double activatorOut = 0.4;
    private double activatorIn = 1;
    static final double LEFT_SHOULDER_IN = 0.15;
    static final double LEFT_SHOULDER_OUT = 0.60;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {

        telemetry.addData("Status", "Initialized");

        shoulderleft = hardwareMap.servo.get("shoulderleft");
        relicClaw = hardwareMap.get(Servo.class, "relicClaw");
        relicGrabber = hardwareMap.get(Servo.class, "relicGrabber");
        activator = hardwareMap.get(Servo.class, "activator");

        leftFront  = hardwareMap.get(DcMotor.class, "leftFront");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        leftBack  = hardwareMap.get(DcMotor.class, "leftBack");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");
        lift = hardwareMap.get(DcMotor.class, "lift");
        relicMotor = hardwareMap.get(DcMotor.class, "relicMotor");
        intake = hardwareMap.get(DcMotor.class, "intake");

        leftFront.setDirection(DcMotor.Direction.FORWARD);
        leftBack.setDirection(DcMotor.Direction.FORWARD);
        lift.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.REVERSE);

        relicGrabber.setPosition(GRABBERINIT);
        relicClaw.setPosition(CLAWINIT);

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        float leftY = getWheelPower(gamepad1.left_stick_y*.75);
        float leftX = getWheelPower(gamepad1.left_stick_x);
        float rightX = getWheelPower(gamepad1.right_stick_x*.75);

        leftFront.setPower((leftY - leftX - rightX));
        leftBack.setPower((leftY + leftX - rightX)*.92);
        rightFront.setPower((leftY + leftX + rightX));
        rightBack.setPower((leftY - leftX + rightX)*.92);

        lift.setPower(gamepad2.left_stick_y);
        if(gamepad2.right_trigger>0.1)
            intake.setPower(gamepad2.right_trigger);
        else if(gamepad2.left_trigger>0.1)
            intake.setPower(-gamepad2.left_trigger);
        else{
            intake.setPower(0);
        }

        if(gamepad2.a&&runtime.seconds()>grabberunpressed){
            if(grabberUp){
                relicGrabber.setPosition(GRABBERDOWN);
                GRABBERCURRENT = GRABBERDOWN;
            }
            else{
                relicGrabber.setPosition(GRABBERUP);
                GRABBERCURRENT = GRABBERUP;
            }
            grabberunpressed = runtime.seconds()+0.4;
            grabberUp = !grabberUp;
        }
        if((gamepad2.dpad_up||gamepad2.dpad_down)&&runtime.seconds()>dpadunpressed){
            if(gamepad2.dpad_up&&GRABBERCURRENT<0.98)
                GRABBERCURRENT += 0.02;
            else if(gamepad2.dpad_down&&GRABBERCURRENT>0.02)
                GRABBERCURRENT -= 0.02;
            relicGrabber.setPosition(GRABBERCURRENT);
            dpadunpressed = runtime.seconds()+0.1;
        }
        if(gamepad2.x&&runtime.seconds()>clawunpressed){
            if(clawClosed){
                relicClaw.setPosition(CLAWOPEN);
            }

            else{
                relicClaw.setPosition(CLAWCLOSE);
            }
            clawunpressed = runtime.seconds()+0.4;
            clawClosed = !clawClosed;
        }
        if(gamepad2.y)
            relicGrabber.setPosition(GRABBERINIT);
        if(gamepad2.right_bumper){
            activator.setPosition(activatorOut);
        }
        else{
            activator.setPosition(activatorIn);
        }
        shoulderleft.setPosition(LEFT_SHOULDER_IN);
        relicMotor.setPower(-gamepad2.right_stick_y);

        telemetry.addData("right trigger", gamepad1.right_trigger);
        telemetry.addData("left trigger", gamepad1.left_trigger);



    }



    @Override
    public void stop(){
    }


    //ALL OF THE NUMBERS WE'RE USING HERE ARE MAGIC NUMBERS, SEE THE DESMOS GRAPH FOR WHY WE USED THEM
    //OR JUST PLUG IT INTO DESMOS YOURSELF
    public static float getWheelPower(double in){

        if(in<0){// if in is negative
            in*=100;
            if(in>=-9){
                in = (float)((in*0.0315)/100);
            }
            else if(in<-9 && in>=-39.537){
                in = (float)((0.021*(Math.pow(in-2, 2))+0.063)/-1000);
            }
            else{
                in = (float)((0.75*in)/100);
            }
            return (float)(in);
        }
        else{//if in is positive
            in*=100;
            if(in<=9){
                in = (float)((in*0.0315)/100);
            }
            else if(in>9 && in<=39.537){
                in = (float)((0.021*(Math.pow(in-2, 2))+0.063)/1000);
            }
            else{
                in = (float)((0.75*in)/100);
            }
            return (float)(in);
        }

    }
}
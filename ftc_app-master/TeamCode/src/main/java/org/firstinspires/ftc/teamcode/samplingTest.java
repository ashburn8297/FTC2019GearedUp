/* Copyright (c) 2018 FIRST. All rights reserved.
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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import net.frogbots.ftcopmodetunercommon.opmode.TunableLinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Autonomous(name = "Sampling Test")
//@Disabled
public class samplingTest extends TunableLinearOpMode {
    robotBase robot = new robotBase();
    private ElapsedTime runtime = new ElapsedTime();

    boolean foundState = false;

    @Override
    public void runOpMode(){
        //Initalization
        robot.init(hardwareMap);
        robot.traverse.setPosition(robot.midTraverse);

        //Somehow home ADM, and use this as a known checkpoint for teleOp
        initialize();
        robot.marker.setPosition(robot.markerIn);

        //Prepare for start
        waitForStart();
        runtime.reset();

        int pos = robot.track(runtime);
        telemetry.addData("Location" , pos);
        telemetry.update();
        idle();
        switch(pos){
            case 0:
                robot.
                break;
            case 1:
                break;
            case 2:
                break
        }
    }

    public void initialize(){
        //If home isn't found
        if((robot.hall.getState() == false) && (foundState == false)){
            robot.ADM.setPower(-1);
            initialize();
        }
        //If home is found, runs this only once
        else if((robot.hall.getState() == true) && (foundState == false)){
            foundState = true;
            robot.ADM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.ADM.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.ADM.setTargetPosition(100);
            initialize();
        }
        //While in transit
        else if(robot.ADM.getTargetPosition() != robot.ADM.getCurrentPosition()){
            robot.ADM.setPower(.1);
            initialize();
        }
        //If 100 is found, stop motion.
        else{
            robot.ADM.setPower(0);
        }
    }
}

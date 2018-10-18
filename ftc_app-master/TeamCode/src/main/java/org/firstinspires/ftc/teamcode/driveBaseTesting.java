package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import net.frogbots.ftcopmodetunercommon.opmode.TunableOpMode;

import org.firstinspires.ftc.teamcode.robotBase;

import static android.os.SystemClock.sleep;

@TeleOp(name = "Drive Base Test")
//@Disabled
public class driveBaseTesting extends TunableOpMode {

    robotBase robot                     = new robotBase();
    private ElapsedTime runtime         = new ElapsedTime();

    boolean aDown                       = false;
    boolean bDown                       = false;
    boolean finished                    = true;

    @Override
    public void init() {
        robot.init(hardwareMap);

        //Run down until limit switch is hit
        telemetry.addData("Status", "HOMING");
        telemetry.update();
        while(robot.admLim.getState() != true){
            robot.ADM.setPower(-.5); //negative is down
        }
        //Zero encoder
        robot.ADM.setPower(0.0);
        robot.ADM.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.ADM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.ADM.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        telemetry.addData("Homed. Current Position", robot.ADM.getCurrentPosition());
        telemetry.update();
    }
    @Override
    public void loop() {
            runtime.reset();

            robot.leftDrive.setPower(gamepad1.left_stick_y);
            robot.rightDrive.setPower(gamepad1.right_stick_y);
            telemetry.addData("Lim Switch Status", robot.admLim.getState());
            telemetry.update();

            //@TODO check to see if the limit switch system works
            if(gamepad1.a){
                aDown = true;
            }
            if(aDown){
                robot.ADM.setTargetPosition((int) robotBase.COUNTS_PER_MOTOR_REV * robot.LEAD_SCREW_TURNS);
                robot.ADM.setPower(1.0);
                finished = false;
            }


            if(gamepad1.b){
                bDown = true;
            }
            if(bDown){
                robot.ADM.setTargetPosition(0);
                robot.ADM.setPower(1.0);
                finished = false;
            }

            if(finished = false){
                if(robot.ADM.getTargetPosition() == robot.ADM.getCurrentPosition())
                    finished = true;
                robot.ADM.setPower(0);
                aDown = false;
                bDown = false;
            }

    }

}

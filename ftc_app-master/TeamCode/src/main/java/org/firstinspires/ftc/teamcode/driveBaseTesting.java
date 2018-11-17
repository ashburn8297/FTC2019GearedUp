package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import net.frogbots.ftcopmodetunercommon.opmode.TunableOpMode;

import org.firstinspires.ftc.teamcode.robotBase;

import static android.os.SystemClock.sleep;

@TeleOp(name = "TeleOp Test")
//@Disabled
public class driveBaseTesting extends TunableOpMode {

    robotBase robot                     = new robotBase();
    private ElapsedTime runtime         = new ElapsedTime();

    int direction = -1;

    double leftPower = 0.0;
    double rightPower = 0.0;

    boolean yDown = false;
    boolean aDown = false;
    boolean bDown = false;
    boolean rbDown = false;
    boolean xDown = false;
    boolean dpadUp = false;


    @Override
    public void init() {
        robot.init(hardwareMap);
        robot.traverse.setPosition(robot.midTraverse);
        sleep(3500);

        robot.ADM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.ADM.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.ADM.setTargetPosition(250);

    }
    @Override
    public void loop() {
        if(direction == 1) {
            leftPower = robot.getWheelPower(gamepad1.left_stick_y);
            rightPower = robot.getWheelPower(gamepad1.right_stick_y);
        }
        else if(direction == -1){
            rightPower = robot.getWheelPower(gamepad1.left_stick_y);
            leftPower = robot.getWheelPower(gamepad1.right_stick_y);
        }
        robot.leftDrive.setPower((leftPower * direction));
        robot.rightDrive.setPower((rightPower * direction));


        yDown = gamepad1.y;
        aDown = gamepad1.a;
        xDown = gamepad1.x;
        bDown = gamepad1.b;
        rbDown = gamepad1.right_bumper;
        dpadUp = gamepad1.dpad_up;

        if(yDown){
            robot.ADM.setTargetPosition((int)(robot.LEAD_SCREW_TURNS * robot.COUNTS_PER_MOTOR_REV));
            robot.ADM.setPower(.75);
            if(robot.ADM.getCurrentPosition() > robot.ADM.getTargetPosition() - 2000){
                robot.ADM.setPower(.1);
            }
            if(robot.ADM.getCurrentPosition() > robot.ADM.getTargetPosition() - 350) {
                yDown = false;
                robot.ADM.setPower(0);
            }
        }

        if(aDown){
            robot.ADM.setTargetPosition(0);
            robot.ADM.setPower(.75);
            if(robot.ADM.getCurrentPosition() < robot.ADM.getTargetPosition() + 2000){
                robot.ADM.setPower(.1);
            }
            if(robot.ADM.getCurrentPosition() < 550) {
                aDown = false;
                robot.ADM.setPower(0);
            }
        }

        if(xDown){
            robot.traverse.setPosition(robot.maxTraverse);
            xDown = false;
        }

        if(bDown){
            robot.traverse.setPosition(robot.minTraverse);
            bDown = false;
        }

        if(rbDown){
            robot.traverse.setPosition(robot.midTraverse);
            rbDown = false;
        }

        if(dpadUp) {
            direction *= -1;
            sleep(500);
            dpadUp = false;
        }
    }
}

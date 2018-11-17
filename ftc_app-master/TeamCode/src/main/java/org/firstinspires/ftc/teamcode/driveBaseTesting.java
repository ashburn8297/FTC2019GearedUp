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

    int direction = 1;
    int power = 0;

    double leftPower = 0.0;
    double rightPower = 0.0;
    double servoPower = 0.0;

    boolean yDown = false;
    boolean aDown = false;
    boolean bDown = false;
    boolean rbDown = false;
    boolean xDown = false;
    boolean lbDown = false;
    boolean dpadUp = false;
    boolean dpadDown = false;


    @Override
    public void init() {
        robot.init(hardwareMap);
        robot.traverse.setPosition(robot.minTraverse);
        sleep(3000);
        //Home ADM
        while(robot.admLim.getState() == false) {
            robot.ADM.setPower(-.5);
        }
        robot.ADM.setPower(0);
        robot.ADM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.ADM.setMode(DcMotor.RunMode.RUN_TO_POSITION);


    }
    @Override
    public void loop() {

            leftPower = robot.getWheelPower(gamepad1.left_stick_y) - robot.getWheelPowerLinear(gamepad1.right_stick_x);
            rightPower = robot.getWheelPower(gamepad1.left_stick_y) + robot.getWheelPowerLinear(gamepad1.right_stick_x);

            robot.leftDrive.setPower(leftPower * direction + power);
            robot.rightDrive.setPower(rightPower * direction + power);


            yDown = gamepad1.y;
            aDown = gamepad1.a;
            xDown = gamepad1.x;
            bDown = gamepad1.b;
            rbDown = gamepad1.right_bumper;
            lbDown = gamepad1.left_bumper;
            dpadUp = gamepad1.dpad_up;
            dpadDown = gamepad1.dpad_down;

            if(yDown){
                robot.ADM.setTargetPosition((int)(robot.LEAD_SCREW_TURNS * robot.COUNTS_PER_MOTOR_REV));
                robot.ADM.setPower(.75);
                if(robot.ADM.getCurrentPosition() > robot.ADM.getTargetPosition() - 350) {
                    yDown = false;
                    robot.ADM.setPower(0);
                }
            }


            if(aDown){
                robot.ADM.setTargetPosition(0);
                robot.ADM.setPower(.75);
                if(robot.ADM.getCurrentPosition() < 350) {
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

            if(lbDown) {
                direction *= -1;
            }

            if(dpadDown) {
                if(power != 0) {
                    power -= 25;
                }
            }

            if(dpadUp) {
                if(power != 25) {
                    power += 25;
                }
            }


    }

}

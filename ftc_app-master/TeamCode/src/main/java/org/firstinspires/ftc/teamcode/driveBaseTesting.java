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

    double leftPower = 0.0;
    double rightPower = 0.0;

    @Override
    public void init() {
        robot.init(hardwareMap);

    }
    @Override
    public void loop() {
            runtime.reset();

            leftPower = robot.getWheelPower(gamepad1.left_stick_y) - robot.getWheelPowerLinear(gamepad1.right_stick_x);
            rightPower = robot.getWheelPower(gamepad1.left_stick_y) + robot.getWheelPowerLinear(gamepad1.right_stick_x);

            robot.leftDrive.setPower(leftPower);
            robot.rightDrive.setPower(rightPower);

    }

}

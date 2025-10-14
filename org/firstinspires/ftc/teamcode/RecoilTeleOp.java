package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "RecoilTeleOp", group = "TeleOp")
public class RecoilTeleOp extends LinearOpMode {
    float threshold = 0.3f;
    long cooldown = 200;
    long lastLeft = 0;
    long lastRight = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();
        while (opModeIsActive()) {
            float lt = gamepad1.left_trigger;
            float rt = gamepad1.right_trigger;
            long now = System.currentTimeMillis();
            if (lt > threshold && now - lastLeft > cooldown) {
                lastLeft = now;
                gamepad1.rumble(75); // basic rumble for 150 ms
            }
            if (rt > threshold && now - lastRight > cooldown) {
                lastRight = now;
                gamepad1.rumble(75);
            }
            telemetry.addData("LT", "%.2f", lt);
            telemetry.addData("RT", "%.2f", rt);
            telemetry.update();
            sleep(10);
        }
    }
}

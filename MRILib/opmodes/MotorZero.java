package MRILib.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp

public class MotorZero extends LinearOpMode{
    
    private DcMotorEx pivot;
    private DcMotorEx slide;
    
    @Override
    public void runOpMode() {
        
        pivot = hardwareMap.get(DcMotorEx.class, "pivot");
        slide = hardwareMap.get(DcMotorEx.class, "slide");
        
        pivot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        
        pivot.setDirection(DcMotor.Direction.FORWARD);
        slide.setDirection(DcMotor.Direction.FORWARD);
        
        waitForStart();
        while(opModeIsActive()) {
            telemetry.addData("Pivot", pivot.getCurrentPosition());
            telemetry.addData("Slide", slide.getCurrentPosition());
            telemetry.update();
        }
    }
}
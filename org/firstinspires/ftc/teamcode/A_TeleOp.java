// package org.firstinspires.ftc.teamcode;

// import MRILib.*;
// import com.qualcomm.robotcore.hardware.Gamepad;
// import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
// import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
// import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
// import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
// import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
// import com.qualcomm.robotcore.hardware.DcMotor;
// import MRILib.managers.*;
// import MRILib.statemachine.*;
// import MRILib.util.*;
// import static MRILib.BotValues.*;

// @TeleOp(name = "A_Teleop (in development)")
// public class A_TeleOp extends LinearOpMode{
//     public ArmBotFF bot;
//     public ArmFSM asm;
    
//     boolean rezero = false;
    
//     public Bpad gpad1;
//     public Bpad gpad2;
    
//     Gamepad.RumbleEffect rumble;
    
//     @Override
//     public void runOpMode(){
//         initialize();
//         bot.setSlidesPower(-.5);
//         bot.setPivotPower(-.2);
        
//         rumble = new Gamepad.RumbleEffect.Builder().addStep(1.0,1.0,100).build();
        
//         telemetry.addLine("initialized.");
//         telemetry.update();
        

//         waitForStart();
//         initialize();
//         bot.startMultiThread();

//         while(opModeIsActive()){
//             if(gamepad1!=null)gpad1.update(gamepad1);
//             if(gamepad2!=null)gpad2.update(gamepad2);
            
//             bot.update();
//             if(rezero){
//                 bot.setPivotControllerMode(PivotArmController.PivotMode.RUN_WITH_POWER);
//                 bot.setSlidesPower(-gamepad2.right_stick_y);
//                 bot.setPivotPower(-gamepad2.left_stick_y/2);
//                 if(gpad2.get("back")){
//                     gamepad2.runRumbleEffect(rumble);
//                     bot.stopMultiThread();
//                     initialize();
//                     bot.startMultiThread();
//                     rezero = false;
//                 }
                
//             }else{
//                 double multi = gamepad1.left_trigger;
//                 double jx = -gamepad1.left_stick_y;
//                 double jy = -gamepad1.left_stick_x;
//                 double jw = -gamepad1.right_stick_x;
    
//                 if(gamepad1.dpad_up) {bot.resetHeading(); bot.resetIMUHeading(); gamepad1.runRumbleEffect(rumble);}
//                 if(gamepad1.dpad_down){bot.useIMU(); gamepad1.runRumbleEffect(rumble);}
//                 if(gamepad1.dpad_left){bot.useOdo(); }
    
//                 double clampedMulti = Math.max(.28, (1-multi));
//                 bot.driveFieldXYW(jx*clampedMulti*.9, jy*clampedMulti, jw*clampedMulti);
                
//                 asm.update();
//             }
            
//             //if(gpad2.get("db_right_trigger")) asm.flipClaw();
            
//             if(gpad2.get("db_start")){
//                 gamepad2.runRumbleEffect(rumble);
//                 rezero = !rezero;
//             }
            
//             Pose2D pos = bot.odo.getPosition();
            
//             if(true){
//                 //telemetry.addData("degrees per second", bot.getLeftPivotMotor().getVelocity()/TICKS_PER_DEGREE_ARM);
//                 telemetry.addData("Using IMU", bot.getUsingIMU());
//                 telemetry.addData("X ", pos.getX(DistanceUnit.INCH));
//                 telemetry.addData("Y ", pos.getY(DistanceUnit.INCH));
//                 telemetry.addData("Heading", pos.getHeading(AngleUnit.DEGREES));
//                 telemetry.addData("IMU Heading", bot.getIMUHeading());
//                 telemetry.addLine();
//                 telemetry.addData("menu", gpad2.get("db_menu"));
//                 telemetry.addData("db_start", gpad2.get("db_start"));
//                 telemetry.addData("db_back", gpad2.get("db_back"));
                
//                 telemetry.update();
//             }
            

//         }

//         bot.stopMultiThread();
//     }

//     private void initialize(){
//         gpad1 = new Bpad();
//         gpad2 = new Bpad();
        
//         bot = new ArmBotFF(this);
//         bot.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//         bot.enableBrakeMode(true);

//         asm = new ArmFSM(bot, gpad1, gpad2, telemetry);
//         //bot.setSlidesPower(1);
        
//     }

// }

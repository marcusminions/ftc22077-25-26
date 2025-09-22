// package MRILib.managers;

// import com.qualcomm.robotcore.hardware.DcMotor;
// import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
// import org.firstinspires.ftc.robotcore.external.Telemetry;
// import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
// import com.qualcomm.robotcore.hardware.Servo;
// import com.qualcomm.robotcore.hardware.DcMotorEx;
// import com.qualcomm.robotcore.hardware.HardwareMap;
// import static MRILib.BotValues.*;
// import MRILib.motion.*;

// public class ArmBotFF extends Bot{

//     // Declaring OpMode members
//     private DcMotorEx slide;
//     private DcMotorEx pivot;

//     private Servo claw;
//     private Servo pitch;
//     private Servo roll;

//     private PivotArmController armController;
//     private Thread armThread;
    
//     private PID slidesPID = new PID(.0066, 0.00006, 0.001);
    
//     int slidePos = 0;
//     int pivotPos = 0;

//     int slidePosPrev = 0;
//     int pivotPosPrev = 0;
    
//     private double currentArmTheta;

//     public ArmBotFF(LinearOpMode op)
//     { //constructor method
//         super(op);
//         initMotors();
//         initServos();
//         armController = new PivotArmController(this);
//         armThread = new Thread(armController);
//         armController.setThetaOffset(armOffset);
//         resetEncoders();
//     }

//     private void initMotors()
//     { //initialising motors
//         slide = op.hardwareMap.get(DcMotorEx.class, "slide");
//         pivot = op.hardwareMap.get(DcMotorEx.class, "pivot");
        
//         slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//         pivot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        
//         slide.setDirection(SLIDEDIR);
//         pivot.setDirection(PIVOTDIR);
        
//         slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//         pivot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        
//         resetEncoders();
//     }

//     private void initServos()
//     { //initialising servos
//         claw = op.hardwareMap.get(Servo.class, "claw");
//         pitch = op.hardwareMap.get(Servo.class, "pitch");
//         roll = op.hardwareMap.get(Servo.class, "roll");
//     }
//     public void setPivotMode(DcMotor.RunMode mode){
//         pivot.setMode(mode);
//     }
//     public void setSlideMode(DcMotor.RunMode mode)
//     {
//         slide.setMode(mode);
//     }
    
//     public void resetEncoders()
//     { //Resetting encoders to 0, keeping current mode
//         super.resetEncoders(); //calling Bot.resetEncoders()
//         DcMotorEx.RunMode slideMode = slide.getMode();
//         DcMotorEx.RunMode pivotMode = pivot.getMode();

//         slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//         pivot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

//         slide.setMode(slideMode);
//         pivot.setMode(pivotMode);
//     }
    
//     public void update(){
//         super.update();
//         currentArmTheta = armController.updateCurrentState(pivotPos, getPivotVelocity(), slide.getCurrentPosition());
//         updateSlidesPID();
//         updateEncoders();
//     }

//     public void updateEncoders()
//     { //updates current and last encoder positions
//         super.updateEncoders(); //Bot.updateEncoders
//         slidePosPrev = slidePos;
//         pivotPosPrev = pivotPos;

//         slidePos = slide.getCurrentPosition();
//         pivotPos = pivot.getCurrentPosition();
//     }
    
//     //getter methods
//     public DcMotorEx getSlideMotor()    { return slide;     }
//     public DcMotorEx getPivotMotor()    { return pivot;     }
//     public int getSlidePos()            { return slidePos;  }
//     public int getPivotPos()            { return pivotPos;  }
//     public synchronized int getSlidePosSync(){return slidePos;}
//     public double getSlideVelocity()    { return slide.getVelocity(); }
//     public double getPivotVelocity()    { return pivot.getVelocity(); }

//     public Servo getPitch()         { return pitch;    }
//     public Servo getRoll()          { return roll;     }
//     public Servo getClaw()          { return claw;     }
    
//     public void setRoll(double pos){
//         roll.setPosition(pos);
//     }
//     public void setPitch(double pos){
//         pitch.setPosition(pos);
//     }
//     public void setClaw(double pos){
//         claw.setPosition(pos);
//     }
    
//     public void setForearm(double pitchPos, double rollPos){
//         setPitch(pitchPos);
//         setRoll(rollPos);
//     }

//     public void setPivotTheta(int theta){
//         armController.setTargetTheta(theta);
//     }
//     public double getPivotTheta(){
//         return currentArmTheta; //radians
//     }
//     public double getPivotThetaDegrees(){
//         return Math.toDegrees(currentArmTheta);
//     }
//     public void setPivotPower(double pow){
//         pivot.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//         pivot.setPower(pow);
//     }
//     public void setPivotControllerMode(PivotArmController.PivotMode mode){
//         armController.setPivotMode(mode);
//     }
    
//     public void setSlidesPower(double pow){
//         slide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//         slide.setPower(pow);
//     }

//     public void setSlides(int target){
//         slide.setTargetPosition(target);
//         slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//         slide.setVelocity(1200);
//     }
    
//     public void updateSlidesPID(){
//         double powerVal = slidesPID.update(slide.getCurrentPosition());
//         powerVal = (12/getVoltage())*(powerVal<0?.25:.4)*powerVal;
        
//         if(powerVal<0 && slide.getCurrentPosition()<200 && Math.abs(slide.getVelocity())<300){
//             powerVal*=4;
//             powerVal = Math.max(-.5,powerVal);
//             if(slide.getVelocity()==0 && slide.getCurrentPosition()<20){
//                 powerVal = 0;
//                 slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//                 slide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//             }
//         }
//         if(powerVal>0 && slide.getVelocity()==0 && slide.getCurrentPosition()>800){
//             powerVal=.2;
//         }
        
//         if(getPivotThetaDegrees()<30 && powerVal>0) powerVal*=.5;
//         // if(Math.abs(slide.getVelocity())>2500){
//         //     powerVal*=.5;
//         // }
//         setSlidesPower(powerVal);
//     }
    
//     public void setSlidesTarget(double target){
//         slidesPID.setTarget(target);
//     }

//     public void setArm(int pivotTarget, int slideTarget){
//         setPivotTheta(pivotTarget);
//         setSlides(slideTarget);
//     }
    
//     public void startMultiThread(){
//         armThread.start();
//     }

//     public void stopMultiThread(){
//         armController.stop();
//         try {
//             armThread.join();  // Wait for the control thread to finish
//         } catch (InterruptedException e) {
//             Thread.currentThread().interrupt();
//         }
//     }
// }


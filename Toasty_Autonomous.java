package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import java.util.List;
import java.util.Timer;

@Autonomous(name="Toasty Auto", group="Autonomous")
public class Toasty_Autonomous extends LinearOpMode {
    public DcMotor MotorFL, MotorBL;
    public DcMotor MotorFR, MotorBR;
    public DcMotor rot, intake, pulley;
    public Servo carriage;
    public boolean linearSlidesDown;
    public boolean linearSlidesMiddle;
    
    private static final String TFOD_MODEL_ASSET = "FreightFrenzy_BCDM.tflite";
    private static final String[] LABELS = {
      "Ball",
      "Cube",
      "Duck",
      "Marker"
    };  
        
    private static final String VUFORIA_KEY =
            "ASdNKIP/////AAABmdFRJ5lzZU9trvU+e5m6JXgIW2PDH+WmK3xTNC7Htzdfb4CzcB8AHCXyEH7T0rnwdrijg1f3B8EKSCwhIC59xvknhxUk1XFl4p662I1v4lqIloMVxHKagWKkt+p8IYWTAgre1AkYn882sqZee2DpCRp2Hu47655aQKTxMQ2HMEPO7tepR1AOrwiuliAYi5TRJEa4DVVi9+M5IXaX+mmHk6ftmCX0SuVCODwXeIjlHbeUVktKAXZwgV+ld9qQzMcLyZByz5qowLVqjJsFNpul2Z4xsg6hGL7sjO0uwgineew/mYzVYGvKNJlXUBCHA3gH4I2HwFzznFbjZCPSXmphITW97fNKKR+2fieFs3mJeHLr";

    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;
    
    //PID
    double prop = 1;
    double deriv = 1;
    double integ = 1;
        
    @Override
    public void runOpMode() {

        MotorFL = hardwareMap.get(DcMotor.class, "fL");
        MotorBL = hardwareMap.get(DcMotor.class, "bL");
        MotorFR = hardwareMap.get(DcMotor.class, "fR");
        MotorBR = hardwareMap.get(DcMotor.class, "bR");
        rot = hardwareMap.get(DcMotor.class, "ro");
        intake = hardwareMap.get(DcMotor.class, "in");
        pulley = hardwareMap.get(DcMotor.class, "pul");
        carriage = hardwareMap.get(Servo.class, "car");
        
        pulley.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        
        carriage.setPosition(0.95);
        
        initVuforia();
        initTfod();
        
        MotorFL.setDirection(DcMotor.Direction.REVERSE);
        MotorFR.setDirection(DcMotor.Direction.REVERSE);
        MotorBL.setDirection(DcMotor.Direction.REVERSE);
        MotorBR.setDirection(DcMotor.Direction.REVERSE);
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        MotorFL.setPower(0);
        MotorBL.setPower(0);
        MotorFR.setPower(0);
        MotorBR.setPower(0);
        waitForStart();

        while (opModeIsActive()) {
            
            if (tfod != null) {
                tfod.activate();
                tfod.setZoom(2.5, 16.0/9.0);
            }
            
            //main code
        }
    }
    
    
    public void runPID(double error) {
        MotorFL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Timer t = new Timer();
        double i = 0;
        double p = 0;
        double d = 0;
        double prevTime = 0;
        double prev = 0;
        while (error > 0.000001) {
            error -= MotorFL.getCurrentPosition();
            
            p = error;
            i += error;
            
            double time = System.nanoTime();
            d = (error - prev)/(time - prevTime);
            
            prevTime = time;
            prev = error;
            
            double power = p * prop + i * integ + d * deriv;
            power = -1 + 2 * (power * 0.01);
            
            MotorFL.setPower(power);
            MotorFR.setPower(power);
            MotorBL.setPower(power);
            MotorBR.setPower(power);
            
        }
        
        MotorFL.setPower(0);
        MotorFR.setPower(0);
        MotorBL.setPower(0);
        MotorBR.setPower(0);
        
    }
    
    public List<Recognition> getObjects() {
        List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
        return updatedRecognitions;
    }

    //Front is 223rpm, Back is 312rpm
    void Drive (float speed){
        if(gamepad1.a) {
            MotorFL.setPower(1*speed);
            MotorBL.setPower(1*speed);
            MotorFR.setPower(1*-speed);
            MotorBR.setPower(1*-speed);
        }
        if(gamepad1.b) {
            MotorFL.setPower(0.2*speed);
            MotorBL.setPower(0.2*speed);
            MotorFR.setPower(0.2*-speed);
            MotorBR.setPower(0.2*-speed);
        }
        else{
            MotorFL.setPower(0.6 * speed);
            MotorBL.setPower(0.6 * speed);
            MotorFR.setPower(0.6 * -speed);
            MotorBR.setPower(0.6 * -speed);
        }
    }
    
    private int getTicks(double inches) {
        double TICKS_PER_MOTOR_REV = 537.6;
        double DRIVE_GEAR_REDUCTION = 1.0;
        double WHEEL_DIAMETER_INCHES = 1.5;
        double COUNTS_PER_INCH = (TICKS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
           (WHEEL_DIAMETER_INCHES * 3.1415);
        
        return (int)(inches * COUNTS_PER_INCH);
    }
    
    private void runEncoders(double powerFL, double powerFR, double powerBL, double powerBR,
                            double fLeft, double fRight, double bLeft, double bRight) {
        int newFL, newFR, newBL, newBR;
        
        if (opModeIsActive()) {
            MotorFL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            MotorFR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            MotorBL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            MotorBR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            
            newFL = getTicks(fLeft);
            newFR = getTicks(fRight);
            newBL = getTicks(bLeft);
            newBR = getTicks(bRight);
            
            MotorFL.setTargetPosition(newFL);
            MotorFR.setTargetPosition(newFR);
            MotorBL.setTargetPosition(newBL);
            MotorBR.setTargetPosition(newBR);
            
            MotorFL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            MotorFR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            MotorBL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            MotorBR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            
            MotorFL.setPower(powerFL);
            MotorFR.setPower(powerFR);
            MotorBL.setPower(powerBL);
            MotorBR.setPower(powerBR);
            
            while(MotorFL.isBusy() && MotorFR.isBusy() 
                && MotorBL.isBusy() && MotorBR.isBusy()) {}
            
            MotorFL.setPower(0);
            MotorFR.setPower(0);
            MotorBL.setPower(0);
            MotorBR.setPower(0);
            
            MotorFL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            MotorFR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            MotorBL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            MotorBR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            
            sleep(250);
        }
        
    }
    
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");
        //parameters.cameraDirection = CameraDirection.BACK;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }

    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
            "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.8f;
        tfodParameters.isModelTensorFlow2 = true;
        tfodParameters.inputSize = 320;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABELS);
    }
    
    private void runSlides(double pos, double power) {
        double TICKS_PER_MOTOR_REV = 537.6;
       //countable events per revolution for output shaft, THIS NUMBER CHANGES DEPENDING ON THE TYPE OF MOTOR. VISIT SPECIFICATIONS ON THE WEBSITE THE TEAM GOT THE MOTOR FROM.
        double DRIVE_GEAR_REDUCTION = 1.0;
       //drive gear reduction
        double WHEEL_DIAMETER_INCHES = 1.5;
       //diameter of wheels
        double COUNTS_PER_INCH = (TICKS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
           (WHEEL_DIAMETER_INCHES * 3.1415);
           
        int newPos;
    
        if (opModeIsActive()) {
            
            pulley.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            
            newPos = (int)(pos * COUNTS_PER_INCH);
          
            pulley.setTargetPosition(newPos);

            pulley.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

            pulley.setPower(power);

            while (opModeIsActive() && pulley.isBusy())
            {
               //Wait for it to finish rotating
            }
          
            pulley.setPower(0);

            pulley.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

            sleep(250);
            /*
            1. Pause to prevent current movements to affect any future movements.
            */
       }
    }

    void Turn (float speed){
        if(gamepad1.a)
        {
            MotorFL.setPower(1*speed);
            MotorBL.setPower(1*speed);
            MotorFR.setPower(1*speed);
            MotorBR.setPower(1*speed);

        }
        if(gamepad1.b)
        {
            MotorFL.setPower(0.3*speed);
            MotorBL.setPower(0.3*speed);
            MotorFR.setPower(0.3*speed);
            MotorBR.setPower(0.3*speed);

        }
        else {
            MotorFL.setPower(0.6 * speed);
            MotorBL.setPower(0.6 * speed);
            MotorFR.setPower(0.6 * speed);
            MotorBR.setPower(0.6 * speed);
        }
    }

    void Strafe (float speed){
        if(gamepad1.a)
        {
            MotorFL.setPower(0.9*speed);
            MotorBL.setPower(0.9*-speed);
            MotorFR.setPower(0.9*speed);
            MotorBR.setPower(0.9*-speed);
        }
        if(gamepad1.b)
        {
            MotorFL.setPower(0.3*speed);
            MotorBL.setPower(0.3*-speed);
            MotorFR.setPower(0.3*speed);
            MotorBR.setPower(0.3*-speed);
        }
        else {
            MotorFL.setPower(0.6 * speed);
            MotorBL.setPower(0.6 * -speed);
            MotorFR.setPower(0.6 * speed);
            MotorBR.setPower(0.6 * -speed);
        }
    }

    /**
     * Control a mecanum drive base with three double inputs
     *
     * @param Strafe  is the first double X value which represents how the base should strafe
     * @param Forward is the only double Y value which represents how the base should drive forward
    //* @param Turn    is the second double X value which represents how the base should turn
     */
    public void mecanum(double Strafe, double Forward) {
        //Find the magnitude of the controller's input
        double r = Math.hypot(Strafe, Forward);

        //returns point from +X axis to point (forward, strafe)
        double robotAngle = Math.atan2(Forward, Strafe) - Math.PI / 4;

        //Quantity to turn by (turn)
        // double rightX = Turn;

        //double vX represents the velocities sent to each motor
        final double v1 = (r * Math.cos(robotAngle)); // + rightX;
        final double v2 = (r * Math.sin(robotAngle)); // - rightX;
        final double v3 = (r * Math.sin(robotAngle)); // + rightX;
        final double v4 = (r * Math.cos(robotAngle)); // - rightX;

        MotorFL.setPower(v1);
        MotorFR.setPower(v2);
        MotorBL.setPower(v3);
        MotorBR.setPower(v4);
    }
}

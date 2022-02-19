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

@Autonomous(name="Toasty Auto", group="Autonomous")
public class Toasty_Autonomous extends LinearOpMode {
    public DcMotor MotorFL, MotorBL;
    public DcMotor MotorFR, MotorBR;
    public DcMotor rot, intake, pulley;
    public Servo carriage;
    public boolean linearSlidesDown;
    public boolean linearSlidesMiddle;
        
        
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
            
        }
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
    
    private void runEncoders(double powerFL, double powerFR, double powerBL, double powerBR, double fLeft, double fRight, double bLeft, double bRight) {
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

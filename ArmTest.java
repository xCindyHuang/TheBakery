package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="Basic: Omni Linear OpMode", group="Linear Opmode")

public class Test extends LinearOpMode {

    @Override
    public void runOpMode() {

        Servo claw = hardwareMap.get(Servo.class, "claw");
        Servo claw2 = hardwareMap.get(Servo.class, "claw2");
        
        DcMotor motor1 = hardwareMap.get(DcMotor.class, "arm");
        DcMotor motor2 = hardwareMap.get(DcMotor.class, "arm2");
        
        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        long start;
        while (opModeIsActive()) {
            if (gamepad2.left_stick_y != 0) {
                motor1.setPower(gamepad2.left_stick_y * 0.75);
                motor2.setPower(gamepad2.left_stick_y * 0.75);
                start = System.currentTimeMillis();
            } else {
                if (System.currentTimeMillis() - start > 25) {
                    motor1.setPower(-0.35);
                    motor2.setPower(-0.35);
                } else {
                    motor1.setPower(0);
                    motor2.setPower(0);
                }
            }
        }
    }}

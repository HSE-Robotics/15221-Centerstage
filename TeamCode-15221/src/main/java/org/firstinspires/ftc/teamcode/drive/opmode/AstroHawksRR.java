//package org.firstinspires.ftc.teamcode;
package org.firstinspires.ftc.teamcode.drive.opmode;


import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;




@TeleOp(group = "AstroHawksRR")
public class AstroHawksRR extends LinearOpMode {

    private DcMotor Brazo;

    private Servo Mano; // SliderServo
    private Servo droneLauncher; // DroneServo
//    private Servo intake; //Intake Servo
    private CRServo rightClaw; // Claw
    private CRServo leftClaw; // Claw
    /**
     * This function is executed when this Op Mode is selected from the Driver Station.
     */
    @Override
    public void runOpMode() {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        double armSpeed;
        double drivingSpeed;
        int initialPosition;

        armSpeed = 1;

        Brazo = hardwareMap.get(DcMotor.class, "Brazo");

        Mano = hardwareMap.get(Servo.class, "Mano");
        rightClaw = hardwareMap.get(CRServo.class,"rightClaw");
        leftClaw = hardwareMap.get(CRServo.class,"leftClaw");
        droneLauncher = hardwareMap.get(Servo.class, "droneLauncher");

        Mano.scaleRange(0,1.00);
        // Put initialization blocks here.

        waitForStart();
        if (opModeIsActive()) {

            drivingSpeed = 0.7;

            Brazo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            Brazo.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            Brazo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            Brazo.setDirection(DcMotor.Direction.REVERSE);
            initialPosition = Brazo.getCurrentPosition();

            Brazo.setTargetPosition((initialPosition)+224);
            Brazo.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            Brazo.setPower(armSpeed);

            while (opModeIsActive() && !isStopRequested()) {

                drive.setWeightedDrivePower(
                        new Pose2d(
                                -gamepad1.left_stick_y * drivingSpeed,
                                gamepad1.left_stick_x * drivingSpeed,
                                gamepad1.right_stick_x * drivingSpeed
                        )
                );

                drive.update();

                Pose2d poseEstimate = drive.getPoseEstimate();
                telemetry.addData("x", poseEstimate.getX());
                telemetry.addData("y", poseEstimate.getY());
                telemetry.addData("heading", poseEstimate.getHeading());

                if ((gamepad1.right_trigger)>0) {
                    rightClaw.setPower(1);
                    leftClaw.setPower(-1);
                }else if ((gamepad1.left_trigger)>0) {
                    rightClaw.setPower(-1);
                    leftClaw.setPower(0);
                }else{
                    rightClaw.setPower(0);
                    leftClaw.setPower(0);
                }

                if (gamepad1.share) {
                    droneLauncher.setPosition(0);
                }else if (gamepad1.options) {
                    droneLauncher.setPosition(0.5);
                }

                if (gamepad1.a) {
                    Brazo.setTargetPosition(initialPosition + 500);
                    Brazo.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    Brazo.setPower(armSpeed);
                } else if (gamepad1.x) {
                    Brazo.setTargetPosition(initialPosition + 1000);
                    Brazo.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    Brazo.setPower(armSpeed);
                } else if (gamepad1.y) {
                    Brazo.setTargetPosition(initialPosition + 1500);
                    Brazo.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    Brazo.setPower(armSpeed);
                } else if (gamepad1.b) {
                    Brazo.setTargetPosition(initialPosition);
                    Brazo.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    Brazo.setPower(armSpeed);
                }
                if (gamepad1.start) {
                    initialPosition = Brazo.getCurrentPosition();
                }
                if (gamepad1.dpad_down) {
                    Brazo.setTargetPosition(Brazo.getCurrentPosition() - 100);
                    Brazo.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    Brazo.setPower(armSpeed);
                }
                if (gamepad1.dpad_up) {
                    Brazo.setTargetPosition(Brazo.getCurrentPosition() + 100);
                    Brazo.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    Brazo.setPower(armSpeed);
                }
               // telemetry.addData("gripper", gripper.getPosition());
               // telemetry.addData("gripper ", gripper.getPosition());
                telemetry.addData("slider position", Brazo.getCurrentPosition());
                telemetry.addData("starting Pos.", initialPosition);
                telemetry.addData("Target Pos.", Brazo.getTargetPosition());
                telemetry.update();
            }
        }
    }
}
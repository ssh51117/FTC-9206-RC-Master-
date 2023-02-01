package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Autonomous(name = "Roadrunner Sample Auto")
public class RoadRunnerSampleAuto extends LinearOpMode {

    private DcMotorEx arm;
    private Servo wrist;
    private Servo spooky;
    private Servo claw;
    private DistanceSensor distance;
    private VoltageSensor ControlHub_VoltageSensor;
    private ColorSensor clawSensor;

    @Override
    public void runOpMode() {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        arm = hardwareMap.get(DcMotorEx.class, "arm");
        wrist = hardwareMap.get(Servo.class, "wrist");
        spooky = hardwareMap.get(Servo.class, "spooky");
        claw = hardwareMap.get(Servo.class, "claw");
        distance = hardwareMap.get(DistanceSensor.class, "distance");
        clawSensor = hardwareMap.get(ColorSensor.class, "claw color");
        ControlHub_VoltageSensor = hardwareMap.get(VoltageSensor.class, "Control Hub");

        spooky.setPosition(0.95);
        claw.setPosition(.82);
        sleep(1000);
        arm.setDirection(DcMotorSimple.Direction.REVERSE);
        claw.setPosition(.6);
        sleep(1000);
        wrist.setPosition(.7);
        telemetry.update();

        Trajectory toFirstSpline = drive.trajectoryBuilder(new Pose2d())
                .forward(40)
                .build();
        Trajectory toFirstCone = drive.trajectoryBuilder(toFirstSpline.end())
                        .lineToSplineHeading(new Pose2d(12, 0, Math.toRadians(-135)))
                        .addDisplacementMarker(40, () -> {
                            wrist.setPosition(0.59);
                            arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                            arm.setTargetPosition(1000);
                            arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                            while(arm.getCurrentPosition() < 1000) {
                                if(arm.getCurrentPosition() > 800) {
                                    arm.setPower(.5 * ((1000 - arm.getCurrentPosition())/200.));
                                }
                                if (arm.getCurrentPosition() > 300) {
                                    spooky.setPosition(0.21);
                                }
                                else {
                                    arm.setPower(.5);
                                }
                                telemetry.addData("Arm Power", arm.getPower());
                                telemetry.addData("Arm Position", arm.getCurrentPosition());
                                telemetry.update();
                                if((gamepad2.a) && (gamepad2.left_bumper)) {
                                    break;
                                }
                            }
                        })
                        .build();
        waitForStart();
        drive.followTrajectory(toFirstSpline);
        drive.followTrajectory(toFirstCone);
    }


}

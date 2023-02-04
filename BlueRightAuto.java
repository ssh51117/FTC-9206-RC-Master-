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

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.cameracode.AprilTagDetectionPipeline;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

@Autonomous(name = "Blue + Right")
public class BlueRightAuto extends LinearOpMode {

    private DcMotorEx arm;
    private Servo wrist;
    private Servo spooky;
    private Servo claw;
    private DistanceSensor distance;
    private VoltageSensor ControlHub_VoltageSensor;
    private ColorSensor clawSensor;

    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    static final double FEET_PER_METER = 3.28084;

    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    // UNITS ARE METERS
    double tagsize = 0.166;

    int tag1 = 0;// Tag ID 18 from the 36h11 family, https://github.com/AprilRobotics/apriltag-imgs/blob/master/tag36h11/tag36_11_00000.png
    int tag2 = 1;//https://github.com/AprilRobotics/apriltag-imgs/blob/master/tag36h11/tag36_11_00001.png
    int tag3 = 2;//https://github.com/AprilRobotics/apriltag-imgs/blob/master/tag36h11/tag36_11_00002.png

    AprilTagDetection tagOfInterest = null;
    @Override
    public void runOpMode() {

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(1280, 720, OpenCvCameraRotation.SIDEWAYS_LEFT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });

        telemetry.setMsTransmissionInterval(50);

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        arm = hardwareMap.get(DcMotorEx.class, "arm");
        wrist = hardwareMap.get(Servo.class, "wrist");
        spooky = hardwareMap.get(Servo.class, "spooky");
        claw = hardwareMap.get(Servo.class, "claw");
        distance = hardwareMap.get(DistanceSensor.class, "distance");
        clawSensor = hardwareMap.get(ColorSensor.class, "claw color");
        ControlHub_VoltageSensor = hardwareMap.get(VoltageSensor.class, "Control Hub");

        int coneLevel = 5;
        arm.setDirection(DcMotorSimple.Direction.REVERSE);
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Pose2d startPose = new Pose2d(36, -61.5, Math.toRadians(270));
        drive.setPoseEstimate(startPose);
        telemetry.addLine("building trajectory 1/7");
        telemetry.update();
        TrajectorySequence toFirstScore = drive.trajectorySequenceBuilder(startPose)
                .lineToSplineHeading(new Pose2d(38, -8.5, Math.toRadians(-40)))
                .UNSTABLE_addTemporalMarkerOffset(-3, () -> {
                        arm.setTargetPosition(900);
                        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        arm.setPower(.3);
                    })
                .UNSTABLE_addTemporalMarkerOffset(-2, () -> {
                    spooky.setPosition(0.21);
                    wrist.setPosition(.3);
                })
                .build();
        telemetry.addLine("building trajectory 2/7");
        telemetry.update();
        Trajectory toFirstCone = drive.trajectoryBuilder(toFirstScore.end())
                .addTemporalMarker(0, () -> {
                    arm.setTargetPosition(0);
                    arm.setPower(.3);
                })
                .addTemporalMarker(.3, () -> {
                    spooky.setPosition(0.95);
                    wrist.setPosition(.7);
                })
                .lineToSplineHeading(new Pose2d(57.5, -7.5, 0))
                .build();
        telemetry.addLine("building trajectory 3/7");
        telemetry.update();
        TrajectorySequence toLoopScore = drive.trajectorySequenceBuilder(toFirstCone.end())
                .lineToSplineHeading(new Pose2d(40, -9, Math.toRadians(-40)))
                .UNSTABLE_addTemporalMarkerOffset(-3, () -> {
                    arm.setTargetPosition(900);
                    arm.setPower(.3);
                })
                .UNSTABLE_addTemporalMarkerOffset(-2, () -> {
                    spooky.setPosition(0.21);
                    wrist.setPosition(.3);
                })
                .build();
        telemetry.addLine("building trajectory 4/7");
        telemetry.update();
        Trajectory toLoopCone = drive.trajectoryBuilder(toFirstScore.end())
                .addTemporalMarker(0, () -> {
                    arm.setTargetPosition(0);
                    arm.setPower(.2);
                })
                .addTemporalMarker(.3, () -> {
                    spooky.setPosition(0.95);
                    wrist.setPosition(.7);
                })
                .lineToSplineHeading(new Pose2d(57.5, -7.5, 0))
                .build();
        telemetry.addLine("building trajectory 5/7");
        telemetry.update();
        Trajectory AprilTag1 = drive.trajectoryBuilder(toLoopScore.end())
                .lineToSplineHeading(new Pose2d(14, -12, 0))
                .addTemporalMarker(.4, () -> {
                    wrist.setPosition(.7);
                    spooky.setPosition(0.95);
                    claw.setPosition(.82);
                })
                .build();
        telemetry.addLine("building trajectory 6/7");
        telemetry.update();
        Trajectory AprilTag2 = drive.trajectoryBuilder(toLoopScore.end())
                .lineToSplineHeading(new Pose2d(39, -10, 0))
                .addTemporalMarker(.4, () -> {
                    wrist.setPosition(.7);
                    spooky.setPosition(0.95);
                    claw.setPosition(.82);
                })
                .build();
        telemetry.addLine("building trajectory 7/7");
        telemetry.update();
        Trajectory AprilTag3 = drive.trajectoryBuilder(toLoopScore.end())
                .lineToSplineHeading(new Pose2d(62, -10, 0))
                .addTemporalMarker(.4, () -> {
                    wrist.setPosition(.7);
                    spooky.setPosition(0.95);
                    claw.setPosition(.82);
                })
                .build();

        wrist.setPosition(.7);
        spooky.setPosition(0.95);
        sleep(1000);
        claw.setPosition(.82);
        sleep(2000);
        claw.setPosition(.6);

        while (!isStarted() && !isStopRequested()) {
            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

            if (currentDetections.size() != 0) {
                boolean tagFound = false;

                for (AprilTagDetection tag : currentDetections) {
                    if (tag.id == tag1 || tag.id == tag2 || tag.id == tag3) {
                        tagOfInterest = tag;
                        tagFound = true;
                        break;
                    }
                }
                if (tagFound) {
                    telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
                    tagToTelemetry(tagOfInterest);
                } else {
                    telemetry.addLine("Don't see tag of interest :(");

                    if (tagOfInterest == null) {
                        telemetry.addLine("(The tag has never been seen)");
                    } else {
                        telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                        tagToTelemetry(tagOfInterest);
                    }
                }

            } else {
                telemetry.addLine("Don't see tag of interest :(");

                if (tagOfInterest == null) {
                    telemetry.addLine("(The tag has never been seen)");
                } else {
                    telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                    tagToTelemetry(tagOfInterest);
                }

            }

            telemetry.update();
            sleep(20);
        }

        /**
         * start of program
         */
        waitForStart();

        drive.followTrajectorySequence(toFirstScore);
        telemetry.addData("arm position", arm.getCurrentPosition());
        telemetry.update();
        arm.setTargetPosition(1030);
        sleep(500);
        telemetry.addData("arm scoring position", arm.getCurrentPosition());
        telemetry.update();
        wrist.setPosition(.3);
        sleep(400);
        wrist.setPosition(.45);
        sleep(300);
        claw.setPosition(.82);

        drive.followTrajectory(toFirstCone);
        for(int i = 0; i < 2; i++) {
            if(i != 0) {
                drive.followTrajectory(toLoopCone);
            }
            grabStack(coneLevel);
            coneLevel--;
            drive.followTrajectorySequence(toLoopScore);
            arm.setTargetPosition(1100);
            arm.setPower(.2);
            telemetry.addData("arm scoring position", arm.getCurrentPosition());
            telemetry.update();
            sleep(500);
            wrist.setPosition(.3);
            sleep(400);
            wrist.setPosition(.45);
            sleep(300);
            claw.setPosition(.82);
        }

        arm.setTargetPosition(0);
        sleep(300);
        if(tagOfInterest.id == 1) {
            drive.followTrajectory(AprilTag2);
        }
        else if(tagOfInterest.id == 2) {
            drive.followTrajectory(AprilTag3);
        }
        else {
            drive.followTrajectory(AprilTag1);
        }
    }
    void tagToTelemetry(AprilTagDetection detection)
    {
        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
        telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z*FEET_PER_METER));
        telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
        telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
        telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));
    }

    public void grabStack(int level) {
        switch(level) {
            case (1):
                wrist.setPosition(.3);
                break;
            case (2):
                wrist.setPosition(.357);
                break;
            case (3):
                wrist.setPosition(.387);
                break;
            case (4):
                wrist.setPosition(.405);
                break;
            case (5):
                wrist.setPosition(.42);

        }
        sleep(250);
        claw.setPosition(.6);
        sleep(300);
        wrist.setPosition(.7);
        sleep(200);
    }
}

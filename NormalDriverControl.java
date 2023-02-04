package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Config
@TeleOp(name = "Normal Driver Control Java")
public class NormalDriverControl extends LinearOpMode {

    SampleMecanumDrive drive;


    private DcMotorEx arm;
    private Servo wrist;
    private Servo spooky;
    private Servo claw;
    private DistanceSensor distance;
    private VoltageSensor ControlHub_VoltageSensor;
    private ColorSensor clawSensor;

    int coneGrab;
    double currentWristPos;

    private final double ticks_per_degree = 2400;

    @Override
    public void runOpMode() {
        drive = new SampleMecanumDrive(hardwareMap);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        arm = hardwareMap.get(DcMotorEx.class, "arm");
        wrist = hardwareMap.get(Servo.class, "wrist");
        spooky = hardwareMap.get(Servo.class, "spooky");
        claw = hardwareMap.get(Servo.class, "claw");
        distance = hardwareMap.get(DistanceSensor.class, "distance");
        clawSensor = hardwareMap.get(ColorSensor.class, "claw color");
        ControlHub_VoltageSensor = hardwareMap.get(VoltageSensor.class, "Control Hub");
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // Put initialization blocks here.
        double slowLeftPower;
        double slowRightPower;
        coneGrab = 1;
        waitForStart();
        spooky.setPosition(0.95);
        wrist.setPosition(.7);
        claw.setPosition(0.82);
        arm.setDirection(DcMotorSimple.Direction.REVERSE);
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        if (opModeIsActive()) {
            while (opModeIsActive()) {

                telemetry.addData("Red levels", clawSensor.red());
                telemetry.addData("Blue levels", clawSensor.blue());
                telemetry.addData("Distance sensor", distance.getDistance(DistanceUnit.INCH));
                telemetry.addData("Arm Position", arm.getCurrentPosition());
                telemetry.addData("Arm Power", arm.getPower());
                telemetry.addData("Wrist Pos", wrist.getPosition());
                telemetry.addData("Target Level", coneGrab);
                telemetry.addData("Flip Position", spooky.getPosition());
                telemetry.addData("Claw_Pos", claw.getPosition());

                /**
                 * Drive code
                 *
                 * gamepad1 left stick controls slow drive
                 * gamepad1 right stick controls fast drive
                 */
                slowLeftPower = 0;
                slowRightPower = 0;
                if(gamepad1.right_bumper) {
                    slowRightPower = .35;
                }
                if(gamepad1.left_bumper) {
                    slowLeftPower = .35;
                }
                drive.setWeightedDrivePower(
                        new Pose2d(
                                -((gamepad1.left_stick_y * .35) + (gamepad1.right_stick_y)),
                                -((gamepad1.left_stick_x * .35) + (gamepad1.right_stick_x)),
                                -((gamepad1.right_trigger - gamepad1.left_trigger)  + ((slowRightPower - slowLeftPower)))
                        )
                );
                drive.update();

                /**
                 * Claw Code
                 *
                 * gamepad2 'a' without the left bumper closes the claw
                 * gamepad2 b opens the claw
                 */
                if ((gamepad2.a) && !(gamepad2.left_bumper)) {
                    claw.setPosition(.6);
                }
                if (gamepad2.b) {
                    claw.setPosition(.82);
                }

                /**
                 *  Wrist Code
                 *
                 *  gamepad 2 triggers control the code
                 *  when the bumper is pressed, the adjustments become smaller
                 */
                currentWristPos = wrist.getPosition();
                if (gamepad2.left_bumper) {
                    if (gamepad2.right_trigger != 0 || gamepad2.left_trigger != 0) {
                        wrist.setPosition(currentWristPos + (gamepad2.right_trigger - gamepad2.left_trigger) * 0.01);
                    }
                }
                else
                {
                    wrist.setPosition(currentWristPos + (gamepad2.right_trigger - gamepad2.left_trigger) * 0.03);
                }

                /**
                 * wrist level adjustment code
                 *
                 * uses buttons on gamepad1
                 */

                if (gamepad1.x) {
                    coneGrab = 1;
                    wrist.setPosition(0.3);
                }
                if (gamepad1.dpad_down) {
                    coneGrab = 2;
                    wrist.setPosition(0.357);
                }
                if (gamepad1.dpad_left) {
                    coneGrab = 3;
                    wrist.setPosition(0.387);
                }
                if (gamepad1.dpad_right) {
                    coneGrab = 4;
                    wrist.setPosition(0.42);
                }
                if (gamepad1.dpad_up) {
                    coneGrab = 5;
                    wrist.setPosition(0.46);
                }

                /**
                 * Wrist Flip Code
                 *
                 * gamepad 2 right stick x direction flips the wrist
                 */
                if (gamepad2.right_stick_x > .5) {
                    spooky.setPosition(0.95);
                }
                if (gamepad2.right_stick_x < -.5) {
                    spooky.setPosition(0.21);
                }

                /**
                 * Resets Arm at 0 position
                 *
                 * uses gamepad2 a and left bumper
                 */
                if ((gamepad2.a) && (gamepad2.left_bumper))
                {
                    arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                }

                /**
                 * Controls Arm using gamepad2 left joystick and floats arm otherwise
                 */
                arm.setPower(gamepad2.left_stick_y * -.5);
                double wristFF = .3 * (1 - Math.abs(.2-wrist.getPosition()));
                double gravityFF = .14 * (Math.cos((Math.PI * (arm.getCurrentPosition() / ticks_per_degree)) * wristFF));
                if((Math.cos(Math.PI * (arm.getCurrentPosition() / ticks_per_degree))) < 0) {
                    wristFF = -wristFF;
                }
                if(gamepad2.left_stick_y == 0 && arm.getCurrentPosition() != 0) {
                    telemetry.addData("cos measurement", (Math.cos(Math.PI * (arm.getCurrentPosition() / ticks_per_degree))));
                    arm.setPower(gravityFF);
                }
                telemetry.addData("wristFF", wristFF);
                telemetry.addData("gravityFF", gravityFF);

                /**
                 * Controls arm by setting it to preset locations
                 */
                if(gamepad2.dpad_down && gamepad2.right_bumper)
                {
                    wrist.setPosition(0.59);
                    while (arm.getCurrentPosition() < 1600) {

                        if (arm.getCurrentPosition() > 300) {
                            spooky.setPosition(0.21);
                        }
                        if(gamepad2.dpad_down || gamepad1.y) {
                            arm.setPower(0);
                            break;
                        }
                        slowLeftPower = 0;
                        slowRightPower = 0;
                        if(gamepad1.right_bumper) {
                            slowRightPower = .35;
                        }
                        if(gamepad1.left_bumper) {
                            slowLeftPower = .35;
                        }
                        drive.setWeightedDrivePower(
                                new Pose2d(
                                        -((gamepad1.left_stick_y * .35) + (gamepad1.right_stick_y)),
                                        -((gamepad1.left_stick_x * .35) + (gamepad1.right_stick_x)),
                                        -((gamepad1.right_trigger - gamepad1.left_trigger)  + ((slowRightPower - slowLeftPower)))
                                )
                        );
                        drive.update();

                        telemetry.addData("Arm Power", arm.getPower());
                        telemetry.addData("Arm Position", arm.getCurrentPosition());
                        telemetry.update();
                    }
                }
                if(gamepad2.dpad_down && !gamepad2.right_bumper) {
                    while(arm.getCurrentPosition() > 0){
                        if(arm.getCurrentPosition() < 1000) {
                            spooky.setPosition(0.95);
                            wrist.setPosition(.7);
                        }
                        if(arm.getCurrentPosition() < 200) {
                            arm.setPower((-.3 * (arm.getCurrentPosition()/200.)) - .2);
                        }
                        else {
                            arm.setPower(-.3);
                        }
                        if(gamepad1.y) {
                            arm.setPower(0);
                            break;
                        }
                        slowLeftPower = 0;
                        slowRightPower = 0;
                        if(gamepad1.right_bumper) {
                            slowRightPower = .35;
                        }
                        if(gamepad1.left_bumper) {
                            slowLeftPower = .35;
                        }
                        drive.setWeightedDrivePower(
                                new Pose2d(
                                        -((gamepad1.left_stick_y * .35) + (gamepad1.right_stick_y)),
                                        -((gamepad1.left_stick_x * .35) + (gamepad1.right_stick_x)),
                                        -((gamepad1.right_trigger - gamepad1.left_trigger)  + ((slowRightPower - slowLeftPower)))
                                )
                        );
                        drive.update();

                        telemetry.addData("Arm Power", arm.getPower());
                        telemetry.addData("Arm Position", arm.getCurrentPosition());
                        telemetry.update();
                    }
                }
                if (gamepad2.dpad_left && !gamepad2.right_bumper) {
                    while(arm.getCurrentPosition() < 350) {
                        arm.setPower(.7);
                        slowLeftPower = 0;
                        slowRightPower = 0;
                        if(gamepad1.right_bumper) {
                            slowRightPower = .35;
                        }
                        if(gamepad1.left_bumper) {
                            slowLeftPower = .35;
                        }
                        drive.setWeightedDrivePower(
                                new Pose2d(
                                        -((gamepad1.left_stick_y * .35) + (gamepad1.right_stick_y)),
                                        -((gamepad1.left_stick_x * .35) + (gamepad1.right_stick_x)),
                                        -((gamepad1.right_trigger - gamepad1.left_trigger)  + ((slowRightPower - slowLeftPower)))
                                )
                        );
                        drive.update();

                        telemetry.addData("Arm Power", arm.getPower());
                        telemetry.addData("Arm Position", arm.getCurrentPosition());
                        telemetry.update();
                        if(gamepad2.dpad_down || gamepad1.y) {
                            arm.setPower(0);
                            break;
                        }
                    }
                }
                if (gamepad2.dpad_right && !gamepad2.right_bumper) {
                    while (arm.getCurrentPosition() < 650) {
                        arm.setPower(.7);

                        if(arm.getCurrentPosition() < 300) {
                            wrist.setPosition(.23);
                        }
                        slowLeftPower = 0;
                        slowRightPower = 0;
                        if(gamepad1.right_bumper) {
                            slowRightPower = .35;
                        }
                        if(gamepad1.left_bumper) {
                            slowLeftPower = .35;
                        }
                        drive.setWeightedDrivePower(
                                new Pose2d(
                                        -((gamepad1.left_stick_y * .35) + (gamepad1.right_stick_y)),
                                        -((gamepad1.left_stick_x * .35) + (gamepad1.right_stick_x)),
                                        -((gamepad1.right_trigger - gamepad1.left_trigger)  + ((slowRightPower - slowLeftPower)))
                                )
                        );
                        drive.update();

                        telemetry.addData("Arm Power", arm.getPower());
                        telemetry.addData("Arm Position", arm.getCurrentPosition());
                        telemetry.update();
                        if(gamepad2.dpad_down || gamepad1.y) {
                            arm.setPower(0);
                            break;
                        }
                    }
                }
                if (gamepad2.dpad_right && gamepad2.right_bumper) {
                    while (arm.getCurrentPosition() < 800) {
                        arm.setPower(1);
                        slowLeftPower = 0;
                        slowRightPower = 0;
                        if(gamepad1.right_bumper) {
                            slowRightPower = .35;
                        }
                        if(gamepad1.left_bumper) {
                            slowLeftPower = .35;
                        }
                        drive.setWeightedDrivePower(
                                new Pose2d(
                                        -((gamepad1.left_stick_y * .35) + (gamepad1.right_stick_y)),
                                        -((gamepad1.left_stick_x * .35) + (gamepad1.right_stick_x)),
                                        -((gamepad1.right_trigger - gamepad1.left_trigger)  + ((slowRightPower - slowLeftPower)))
                                )
                        );
                        drive.update();

                        telemetry.update();
                        if(gamepad2.dpad_down || gamepad1.y) {
                            arm.setPower(0);
                            break;
                        }
                    }
                    wrist.setPosition(0.3);
                    while (arm.getCurrentPosition() < 1500) {
                        spooky.setPosition(0.21);
                        arm.setPower(0.2);
                        slowLeftPower = 0;
                        slowRightPower = 0;
                        if(gamepad1.right_bumper) {
                            slowRightPower = .35;
                        }
                        if(gamepad1.left_bumper) {
                            slowLeftPower = .35;
                        }
                        drive.setWeightedDrivePower(
                                new Pose2d(
                                        -((gamepad1.left_stick_y * .35) + (gamepad1.right_stick_y)),
                                        -((gamepad1.left_stick_x * .35) + (gamepad1.right_stick_x)),
                                        -((gamepad1.right_trigger - gamepad1.left_trigger)  + ((slowRightPower - slowLeftPower)))
                                )
                        );
                        drive.update();

                        telemetry.addData("Arm Power", arm.getPower());
                        telemetry.addData("Arm Position", arm.getCurrentPosition());
                        telemetry.update();
                        if(gamepad2.dpad_down || gamepad1.y) {
                            arm.setPower(0);
                            break;
                        }
                    }
                }
                if (gamepad2.dpad_up && !gamepad2.right_bumper) {
                    while (arm.getCurrentPosition() < 750) {
                        arm.setPower(.8);
                        slowLeftPower = 0;
                        slowRightPower = 0;
                        if(gamepad1.right_bumper) {
                            slowRightPower = .35;
                        }
                        if(gamepad1.left_bumper) {
                            slowLeftPower = .35;
                        }
                        drive.setWeightedDrivePower(
                                new Pose2d(
                                        -((gamepad1.left_stick_y * .35) + (gamepad1.right_stick_y)),
                                        -((gamepad1.left_stick_x * .35) + (gamepad1.right_stick_x)),
                                        -((gamepad1.right_trigger - gamepad1.left_trigger)  + ((slowRightPower - slowLeftPower)))
                                )
                        );
                        drive.update();

                        telemetry.addData("Arm Power", arm.getPower());
                        telemetry.addData("Arm Position", arm.getCurrentPosition());
                        telemetry.addData("Claw Position", claw.getPosition());
                        telemetry.update();
                        if(gamepad2.dpad_down || gamepad1.y) {
                            arm.setPower(0);
                            break;
                        }
                        if(arm.getCurrentPosition() > 200) {
                            wrist.setPosition(0);
                        }
                    }
                    while (arm.getCurrentPosition() < 900) {
                        arm.setPower(.4);
                        slowLeftPower = 0;
                        slowRightPower = 0;
                        if(gamepad1.right_bumper) {
                            slowRightPower = .35;
                        }
                        if(gamepad1.left_bumper) {
                            slowLeftPower = .35;
                        }
                        drive.setWeightedDrivePower(
                                new Pose2d(
                                        -((gamepad1.left_stick_y * .35) + (gamepad1.right_stick_y)),
                                        -((gamepad1.left_stick_x * .35) + (gamepad1.right_stick_x)),
                                        -((gamepad1.right_trigger - gamepad1.left_trigger)  + ((slowRightPower - slowLeftPower)))
                                )
                        );
                        drive.update();

                        telemetry.addData("Claw Position", claw.getPosition());
                        telemetry.addData("Arm Power", arm.getPower());
                        telemetry.addData("Arm Position", arm.getCurrentPosition());
                        telemetry.update();
                        if(gamepad2.dpad_down || gamepad1.y) {
                            arm.setPower(0);
                            break;
                        }
                    }
                }
                if (gamepad2.dpad_up && gamepad2.right_bumper) {
                    while (arm.getCurrentPosition() < 800) {
                        spooky.setPosition(0.21);
                        arm.setPower(1);
                        slowLeftPower = 0;
                        slowRightPower = 0;
                        if(gamepad1.right_bumper) {
                            slowRightPower = .35;
                        }
                        if(gamepad1.left_bumper) {
                            slowLeftPower = .35;
                        }
                        drive.setWeightedDrivePower(
                                new Pose2d(
                                        -((gamepad1.left_stick_y * .35) + (gamepad1.right_stick_y)),
                                        -((gamepad1.left_stick_x * .35) + (gamepad1.right_stick_x)),
                                        -((gamepad1.right_trigger - gamepad1.left_trigger)  + ((slowRightPower - slowLeftPower)))
                                )
                        );
                        drive.update();

                        telemetry.update();
                        if(gamepad2.dpad_down || gamepad1.y) {
                            arm.setPower(0);
                            break;
                        }
                    }
                    while (arm.getCurrentPosition() < 1200) {
                        wrist.setPosition(0.59);
                        arm.setPower(0.8);
                        slowLeftPower = 0;
                        slowRightPower = 0;
                        if(gamepad1.right_bumper) {
                            slowRightPower = .35;
                        }
                        if(gamepad1.left_bumper) {
                            slowLeftPower = .35;
                        }
                        drive.setWeightedDrivePower(
                                new Pose2d(
                                        -((gamepad1.left_stick_y * .35) + (gamepad1.right_stick_y)),
                                        -((gamepad1.left_stick_x * .35) + (gamepad1.right_stick_x)),
                                        -((gamepad1.right_trigger - gamepad1.left_trigger)  + ((slowRightPower - slowLeftPower)))
                                )
                        );
                        drive.update();

                        telemetry.update();
                        if(gamepad2.dpad_down || gamepad1.y) {
                            arm.setPower(0);
                            break;
                        }
                    }
                    while (arm.getCurrentPosition() < 1300) {
                        arm.setPower(0.3);
                        slowLeftPower = 0;
                        slowRightPower = 0;
                        if(gamepad1.right_bumper) {
                            slowRightPower = .35;
                        }
                        if(gamepad1.left_bumper) {
                            slowLeftPower = .35;
                        }
                        drive.setWeightedDrivePower(
                                new Pose2d(
                                        -((gamepad1.left_stick_y * .35) + (gamepad1.right_stick_y)),
                                        -((gamepad1.left_stick_x * .35) + (gamepad1.right_stick_x)),
                                        -((gamepad1.right_trigger - gamepad1.left_trigger)  + ((slowRightPower - slowLeftPower)))
                                )
                        );
                        drive.update();

                        telemetry.update();
                        if(gamepad2.dpad_down || gamepad1.y) {
                            arm.setPower(0);
                            break;
                        }
                    }
                }

                /**
                 * Funky Little cycles for tall and medium poles
                 */
                if (gamepad2.y) {
                    boolean skip = false;
                    wrist.setPosition(0.36);
                    sleep(300);
                    claw.setPosition(.6);
                    sleep(350);
                    wrist.setPosition(.7);
                    arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    arm.setTargetPosition(1100);
                    arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    while(arm.getCurrentPosition() < 1100) {
                        slowLeftPower = 0;
                        slowRightPower = 0;
                        if(gamepad1.right_bumper) {
                            slowRightPower = .35;
                        }
                        if(gamepad1.left_bumper) {
                            slowLeftPower = .35;
                        }
                        drive.setWeightedDrivePower(
                                new Pose2d(
                                        -((gamepad1.left_stick_y * .35) + (gamepad1.right_stick_y)),
                                        -((gamepad1.left_stick_x * .35) + (gamepad1.right_stick_x)),
                                        -((gamepad1.right_trigger - gamepad1.left_trigger)  + ((slowRightPower - slowLeftPower)))
                                )
                        );
                        drive.update();
                        if(gamepad2.dpad_down || gamepad1.y) {
                            arm.setPower(0);
                            skip = true;
                            break;
                        }
                        if(arm.getCurrentPosition() > 800) {
                            arm.setPower(.5 * ((1100 - arm.getCurrentPosition())/300.));
                        }
                        else {
                            arm.setPower(.5);
                        }
                        if (arm.getCurrentPosition() > 300) {
                            spooky.setPosition(0.21);
                            wrist.setPosition(0.25);
                        }

                        telemetry.addData("Arm Power", arm.getPower());
                        telemetry.addData("Arm Position", arm.getCurrentPosition());
                        telemetry.update();
                    }
                    if(!skip) {
                        slowLeftPower = 0;
                        slowRightPower = 0;
                        if(gamepad1.right_bumper) {
                            slowRightPower = .35;
                        }
                        if(gamepad1.left_bumper) {
                            slowLeftPower = .35;
                        }
                        drive.setWeightedDrivePower(
                                new Pose2d(
                                        -((gamepad1.left_stick_y * .35) + (gamepad1.right_stick_y)),
                                        -((gamepad1.left_stick_x * .35) + (gamepad1.right_stick_x)),
                                        -((gamepad1.right_trigger - gamepad1.left_trigger)  + ((slowRightPower - slowLeftPower)))
                                )
                        );
                        drive.update();
                        sleep(300);
                        telemetry.addData("Arm Position when claw opens", arm.getCurrentPosition());
                        telemetry.update();
                        claw.setPosition(.82);
                        sleep(200);
                        wrist.setPosition(.7);
                        while (arm.getCurrentPosition() > 0) {
                            telemetry.addData("Arm Position when arm starts retracting", arm.getCurrentPosition());
                            telemetry.update();
                            arm.setTargetPosition(0);
                            if (arm.getCurrentPosition() < 1000) {
                                spooky.setPosition(0.95);
                                wrist.setPosition(.7);
                            }
                            if (arm.getCurrentPosition() < 200) {
                                arm.setPower(-.3 * (arm.getCurrentPosition() / 200.));
                            } else {
                                arm.setPower(-.3);
                            }
                            telemetry.update();
                            if (gamepad2.dpad_down || gamepad1.y) {
                                arm.setPower(0);
                                skip = true;
                                break;
                            }
                        }
                    }
                    arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                }
                if (gamepad2.x) {
                    slowLeftPower = 0;
                    slowRightPower = 0;
                    if(gamepad1.right_bumper) {
                        slowRightPower = .35;
                    }
                    if(gamepad1.left_bumper) {
                        slowLeftPower = .35;
                    }
                    drive.setWeightedDrivePower(
                            new Pose2d(
                                    -((gamepad1.left_stick_y * .35) + (gamepad1.right_stick_y)),
                                    -((gamepad1.left_stick_x * .35) + (gamepad1.right_stick_x)),
                                    -((gamepad1.right_trigger - gamepad1.left_trigger)  + ((slowRightPower - slowLeftPower)))
                            )
                    );
                    drive.update();
                    boolean skip = false;
                    wrist.setPosition(0.36);
                    sleep(500);
                    claw.setPosition(.6);
                    sleep(500);
                    wrist.setPosition(.7);
                    arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    arm.setTargetPosition(1600);
                    arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    while(arm.getCurrentPosition() < 1400) {
                        slowLeftPower = 0;
                        slowRightPower = 0;
                        if(gamepad1.right_bumper) {
                            slowRightPower = .35;
                        }
                        if(gamepad1.left_bumper) {
                            slowLeftPower = .35;
                        }
                        drive.setWeightedDrivePower(
                                new Pose2d(
                                        -((gamepad1.left_stick_y * .35) + (gamepad1.right_stick_y)),
                                        -((gamepad1.left_stick_x * .35) + (gamepad1.right_stick_x)),
                                        -((gamepad1.right_trigger - gamepad1.left_trigger)  + ((slowRightPower - slowLeftPower)))
                                )
                        );
                        drive.update();
                        if(gamepad2.dpad_down || gamepad1.y) {
                            arm.setPower(0);
                            break;
                        }
                        if(arm.getCurrentPosition() > 800) {
                            arm.setPower(.4 * ((1400 - arm.getCurrentPosition())/600.));
                        }
                        else {
                            arm.setPower(.5);
                        }
                        if (arm.getCurrentPosition() > 300) {
                            spooky.setPosition(0.21);
                            wrist.setPosition(0.17);
                        }

                        telemetry.addData("Arm Power", arm.getPower());
                        telemetry.addData("Arm Position", arm.getCurrentPosition());
                        telemetry.update();
                    }
                    if(!skip) {
                        sleep(100);
                        claw.setPosition(.82);
                        sleep(500);
                        wrist.setPosition(.7);

                        while (arm.getCurrentPosition() > 20) {
                            if (gamepad2.dpad_down || gamepad1.y) {
                                arm.setPower(0);
                                skip = true;
                                break;
                            }
                            telemetry.addData("Arm Position when arm starts retracting", arm.getCurrentPosition());
                            telemetry.update();
                            arm.setTargetPosition(0);
                            if (arm.getCurrentPosition() < 1000) {
                                spooky.setPosition(0.95);
                                wrist.setPosition(.7);
                            }
                            if (arm.getCurrentPosition() < 600) {
                                arm.setPower(-.3 * (arm.getCurrentPosition() / 600.));
                            } else {
                                arm.setPower(-.5);
                            }
                            telemetry.update();
                        }
                    }
                    arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                }
                telemetry.update();
            }
        }
    }
}

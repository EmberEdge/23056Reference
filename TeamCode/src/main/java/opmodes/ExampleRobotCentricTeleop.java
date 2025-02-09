package opmodes;

import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.util.Constants;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import helpers.data.Enums;
import helpers.data.Enums.DetectedColor;
import helpers.hardware.MotorControl;
import helpers.hardware.actions.ActionOpMode;
import helpers.hardware.actions.MotorActions;
import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

/**
 * This is an example teleop that showcases movement and robot-centric driving.
 *
 * @author Baron Henderson - 20077 The Indubitables
 * @version 2.0, 12/30/2024
 */

@TeleOp(name = "Actions Teleop", group = "Examples")
public class ExampleRobotCentricTeleop extends ActionOpMode {
    private Follower follower;
    private final Pose startPose = new Pose(0, 0, 0);
    private MotorControl motorControl;
    private MotorActions motorActions;

    private DetectedColor allianceColor = DetectedColor.RED;

    private boolean gamepad2XPressed = false;
    private boolean poopPressed = false;
    private boolean dpadDownPressed = false;
    private boolean rightBumperPressed = false;
    private boolean leftBumperPressed = false;
    private boolean rightTriggerPressed = false;
    private boolean leftTriggerPressed = false;


    /** This method is call once when init is played, it initializes the follower **/
    @Override
    public void init() {
        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);

        motorControl = new MotorControl(hardwareMap);
        motorActions = new MotorActions(motorControl);
    }

    /** This method is called continuously after Init while waiting to be started. **/
    @Override
    public void init_loop() {
    }

    /** This method is called once at the start of the OpMode. **/
    @Override
    public void start() {
        follower.startTeleopDrive();
        run(new SequentialAction(motorActions.intakeTransfer(),
                motorActions.outtakeTransfer()));
    }

    /** This is the main loop of the opmode and runs continuously after play **/
    @Override
    public void loop() {

        if (gamepad1.x && !gamepad2XPressed) {
            if (allianceColor == DetectedColor.RED) {
                allianceColor = DetectedColor.BLUE;
            } else {
                allianceColor = DetectedColor.RED;
            }
            gamepad2XPressed = true;
        } else if (!gamepad1.x) {
            gamepad2XPressed = false;
        }

        if (gamepad1.right_bumper && !rightBumperPressed) {
            if (motorActions.intakePosition != Enums.Intake.Extended) {
                run(motorActions.intakeExtend(420));
            }
            rightBumperPressed = true;
        } else if (!gamepad1.right_bumper) {
            rightBumperPressed = false;
        }

        if (gamepad1.left_bumper && !leftBumperPressed) {
            if (motorActions.intakePosition == Enums.Intake.Extended) {
                run(motorActions.spin.eat());
                run(motorActions.intakeGrabUntil(allianceColor));
            }
            leftBumperPressed = true;
        } else if (!gamepad1.left_bumper) {
            leftBumperPressed = false;
        }

        if (gamepad1.right_trigger > 0 && !rightTriggerPressed) {
            if (motorActions.intakePosition == Enums.Intake.Transfer) {
                run(motorActions.outtakeSample());
            }
            rightTriggerPressed = true;
        } else if (gamepad1.right_trigger == 0) {
            rightTriggerPressed = false;
        }

        if (gamepad1.left_trigger > 0 && !leftTriggerPressed) {
            if (motorActions.outtakePosition == Enums.OutTake.Deposit) {
                run(motorActions.outtakeTransfer());
            }
            leftTriggerPressed = true;
        } else if (gamepad1.left_trigger == 0) {
            leftTriggerPressed = false;
        }

        if (gamepad1.dpad_down && !dpadDownPressed) {
            poopPressed = !poopPressed;

            if (poopPressed) {
                run(new ParallelAction(
                        motorActions.intakeExtend(420),
                        motorActions.spin.poop()
                ));
            } else {
                run(new ParallelAction(
                        motorActions.spin.stop(),
                        motorActions.intakeTransfer()
                ));
            }

            dpadDownPressed = true;
        } else if (!gamepad1.dpad_down) {
            dpadDownPressed = false;
        }

        if (gamepad1.a) {
            run(motorActions.intakeSpecimen());
        } else if (gamepad1.b) {
            run(motorActions.outtakeSpecimen());
        } else if (gamepad1.y) {
            run(motorActions.depositSpecimen());
        }

        super.loop();
        motorControl.update();

        double rotationFactor = 1.0;

        if (motorActions.intakePosition != Enums.Intake.Extended){
            rotationFactor = 0.5;
        }

        if (gamepad1.dpad_left) {
            run(motorActions.hang.up());
        }
        else if (gamepad1.dpad_right) {
            run(motorActions.hang.down());
        }
        else {
            run(motorActions.hang.stop());
        }

        follower.setTeleOpMovementVectors(-gamepad1.right_stick_y, -gamepad1.right_stick_x, -gamepad1.left_stick_x * rotationFactor, true);
        follower.update();

        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading in Degrees", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.addData("Alliance Color:", allianceColor);
        telemetry.addData("Intake Color:", motorControl.getDetectedColor());
        telemetry.addData("Intake Position State:", motorActions.intakePosition);
        telemetry.addData("Extendo Position:", motorControl.extendo.motor.getCurrentPosition());
        telemetry.addData("Lift Position:", motorControl.lift.motor.getCurrentPosition());
        telemetry.update();
    }

    /** We do not use this because everything automatically should disable **/
    @Override
    public void stop() {
    }
}

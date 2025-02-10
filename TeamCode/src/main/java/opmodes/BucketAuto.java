package opmodes;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import helpers.data.Enums;
import helpers.hardware.MotorControl;
import helpers.hardware.actions.MotorActions;
import helpers.hardware.actions.PathChainAutoOpMode;
import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

/**
 * BucketAuto is an autonomous OpMode that uses a series of path chains and tasks.
 * It extends PathChainAutoOpMode so that you only need to override buildPathChains() and buildTaskList(),
 * and also provide implementations for the dummy methods that integrate with your follower.
 */
@Autonomous(name = "Bucket Sample")
public class BucketAuto extends PathChainAutoOpMode {

    // ---------------------------
    // Hardware and Helper Fields
    // ---------------------------
    private Follower follower;
    private MotorActions motorActions;
    private MotorControl motorControl;
    private MotorControl.Limelight limelight;

    // ---------------------------
    // Poses used in the autonomous routine.
    // ---------------------------
    private final Pose startPose   = new Pose(9, 111, Math.toRadians(270));
    private final Pose scorePose   = new Pose(17, 128, Math.toRadians(315));
    private final Pose pickup1Pose = new Pose(20, 122, Math.toRadians(0));
    private final Pose pickup2Pose = new Pose(20, 130, Math.toRadians(0));
    private final Pose pickup3Pose = new Pose(24, 128, Math.toRadians(20));

    private final Pose parkPose        = new Pose(62, 97, Math.toRadians(315));
    private final Pose parkControlPose = new Pose(64.5, 116, Math.toRadians(270));

    // ---------------------------
    // PathChains
    // ---------------------------
    private PathChain scorePreload;
    private PathChain intake1, intake2, intake3;
    private PathChain score1, score2, score3;
    private PathChain parkChain;

    // ---------------------------
    // Override buildPathChains(): Define the geometry for your paths.
    // ---------------------------
    @Override
    protected void buildPathChains() {
        // Build the intake and scoring paths using your follower's path builder.
        intake1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(scorePose), new Point(pickup1Pose)))
                .setConstantHeadingInterpolation(pickup1Pose.getHeading())
                .addParametricCallback(0.6, () -> run(motorActions.intakeGrabUntil(Enums.DetectedColor.YELLOW)))
                .addParametricCallback(1, () -> run(motorActions.extendo.setTargetPosition(200)))
                .build();

        intake2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(scorePose), new Point(pickup2Pose)))
                .setConstantHeadingInterpolation(pickup2Pose.getHeading())
                .addParametricCallback(0.6, () -> run(motorActions.intakeGrabUntil(Enums.DetectedColor.YELLOW)))
                .addParametricCallback(1, () -> run(motorActions.extendo.setTargetPosition(350)))
                .build();

        intake3 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(scorePose), new Point(pickup3Pose)))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup3Pose.getHeading(), 50)
                .addParametricCallback(0.6, () -> run(motorActions.intakeGrabUntil(Enums.DetectedColor.YELLOW)))
                .addParametricCallback(1, () -> run(motorActions.extendo.setTargetPosition(400)))
                .build();

        score1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pickup1Pose), new Point(scorePose)))
                .setLinearHeadingInterpolation(pickup1Pose.getHeading(), scorePose.getHeading(), 50)
                .addParametricCallback(0, ()->run(motorActions.spin.slow()))
                .addParametricCallback(0.3, () -> run(new SequentialAction(
                        motorActions.lift.waitUntilFinished(0),
                        motorActions.outtakeSampleAuto()
                )))
                .build();

        score2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pickup2Pose), new Point(scorePose)))
                .setLinearHeadingInterpolation(pickup2Pose.getHeading(), scorePose.getHeading(), 50)
                .addParametricCallback(0, ()->run(motorActions.spin.slow()))
                .addParametricCallback(0.3, () -> run(new SequentialAction(
                        motorActions.lift.waitUntilFinished(0),
                        motorActions.outtakeSampleAuto()
                )))
                .build();

        score3 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pickup3Pose), new Point(scorePose)))
                .setLinearHeadingInterpolation(pickup3Pose.getHeading(), scorePose.getHeading(), 50)
                .addParametricCallback(0, ()->run(motorActions.spin.slow()))
                .addParametricCallback(0.3, () -> run(new SequentialAction(
                        motorActions.lift.waitUntilFinished(0),
                        motorActions.outtakeSampleAuto()
                )))
                .build();

        scorePreload = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(startPose), new Point(scorePose)))
                .addParametricCallback(0, () -> run(motorActions.outtakeSampleAuto()))
                .setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading())
                .build();

        parkChain = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(scorePose), new Point(parkControlPose), new Point(parkPose)))
                .setLinearHeadingInterpolation(scorePose.getHeading(), parkPose.getHeading())
                .build();
    }

    // ---------------------------
    // Override buildTaskList(): Define the sequence of tasks.
    // ---------------------------
    @Override
    protected void buildTaskList() {
        tasks.clear();

        // Preload task.
        PathChainTask preloadTask = new PathChainTask(scorePreload, 0.8)
                .addWaitAction(() -> motorControl.lift.closeEnough(770), new SequentialAction(
                        motorActions.outTakeLinkage.sample(),
                        new SleepAction(0.5),
                        motorActions.outtakeTransfer(),
                        motorActions.intakePivot.Grab(),
                        motorActions.intakeArm.Grab()
                ))
                .setMaxWaitTime(2)
                .setWaitCondition(() -> motorControl.lift.closeEnough(770));
        tasks.add(preloadTask);

        // Pickup and scoring tasks.
        PathChainTask pickup1Task = new PathChainTask(intake1, 0.5)
                .setMaxWaitTime(1.25)
                .addWaitAction(0.5, motorActions.extendo.setTargetPosition(500))
                .addWaitAction(1, motorActions.intakeTransfer())
                .setWaitCondition(() -> motorControl.getDetectedColor() != Enums.DetectedColor.UNKNOWN);
        tasks.add(pickup1Task);

        PathChainTask score1Task = new PathChainTask(score1, 0.8)
                .addWaitAction(() -> motorControl.lift.closeEnough(770), new SequentialAction(
                        motorActions.outTakeLinkage.sample(),
                        new SleepAction(0.5),
                        motorActions.outtakeTransfer(),
                        motorActions.intakePivot.Grab(),
                        motorActions.intakeArm.Grab()
                ))
                .setMaxWaitTime(2)
                .setWaitCondition(() -> motorControl.lift.closeEnough(770));
        tasks.add(score1Task);

        PathChainTask pickup2Task = new PathChainTask(intake2, 0.5)
                .setMaxWaitTime(1.25)
                .addWaitAction(0, () -> motorControl.getDetectedColor() == Enums.DetectedColor.UNKNOWN, motorActions.extendo.setTargetPosition(500))
                .addWaitAction(1, motorActions.intakeTransfer())
                .setWaitCondition(() -> motorControl.getDetectedColor() != Enums.DetectedColor.UNKNOWN);
        tasks.add(pickup2Task);

        PathChainTask score2Task = new PathChainTask(score2, 0.8)
                .addWaitAction(() -> motorControl.lift.closeEnough(770), new SequentialAction(
                        motorActions.outTakeLinkage.sample(),
                        new SleepAction(0.5),
                        motorActions.outtakeTransfer(),
                        motorActions.intakePivot.Grab(),
                        motorActions.intakeArm.Grab()
                ))
                .setMaxWaitTime(2)
                .setWaitCondition(() -> motorControl.lift.closeEnough(770));
        tasks.add(score2Task);

        PathChainTask pickup3Task = new PathChainTask(intake3, 0.5)
                .setMaxWaitTime(1.25)
                .addWaitAction(1, motorActions.intakeTransfer())
                .setWaitCondition(() -> motorControl.getDetectedColor() != Enums.DetectedColor.UNKNOWN);
        tasks.add(pickup3Task);

        PathChainTask score3Task = new PathChainTask(score3, 1)
                .addWaitAction(() -> motorControl.lift.closeEnough(770), new SequentialAction(
                        motorActions.outTakeLinkage.sample(),
                        new SleepAction(0.5),
                        motorActions.outtakeTransfer(),
                        motorActions.intakePivot.Grab(),
                        motorActions.intakeArm.Grab()
                ))
                .setMaxWaitTime(2)
                .setWaitCondition(() -> motorControl.lift.closeEnough(770));
        tasks.add(score3Task);
    }

    // ---------------------------
    // Override dummy path-follower methods to integrate with your follower.
    // ---------------------------
    @Override
    protected boolean isPathActive() {
        return follower.isBusy();
    }

    @Override
    protected double getCurrentTValue() {
        return follower.getCurrentTValue();
    }

    @Override
    protected void startPath(PathChainTask task) {
        follower.followPath((PathChain) task.pathChain, true);
    }

    // ---------------------------
    // Standard OpMode Lifecycle Methods
    // ---------------------------
    @Override
    public void init() {
        super.init();

        // Initialize hardware and helper classes.
        motorControl = new MotorControl(hardwareMap);
        motorActions = new MotorActions(motorControl);
        Constants.setConstants(FConstants.class, LConstants.class);
        limelight = new MotorControl.Limelight(hardwareMap, telemetry);

        // Initialize the follower and set its starting pose.
        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);

        // Run any starting actions.
        run(motorActions.outTakeClaw.Close());

        // Build your path chains and task list.
        buildPathChains();
        buildTaskList();
    }

    @Override
    public void start() {
        super.start();
        currentTaskIndex = 0;
        taskPhase = 0;
        pathTimer.resetTimer();
        run(motorActions.intakePivot.Transfer());
        run(motorActions.outTakeLinkage.Transfer());
        run(motorActions.intakeArm.Intake());
        run(motorActions.outtakeSampleAuto());
    }

    @Override
    public void loop() {
        super.loop();
        follower.update();
        runTasks();
        motorControl.update();

        telemetry.addData("Task Index", currentTaskIndex + "/" + tasks.size());
        telemetry.addData("Phase", (taskPhase == 0) ? "DRIVE" : "WAIT");
        telemetry.addData("T Value", follower.getCurrentTValue());
        telemetry.addData("Wait Timer", pathTimer.getElapsedTimeSeconds());
        telemetry.addData("Running Actions", runningActions.size());
        telemetry.update();
    }
}

package opmodes;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.pedropathing.pathgen.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.pedropathing.util.Timer;

import helpers.hardware.MotorControl;
import helpers.hardware.actions.ActionOpMode;
import helpers.hardware.actions.MotorActions;
import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

import java.util.ArrayList;
import java.util.List;

@Autonomous(name = "Bucket Sample")
public class BucketAuto extends ActionOpMode {

    // -------------------------------------------------------------------------
    // For time-based triggers during WAIT
    // -------------------------------------------------------------------------
    private static class WaitAction {
        // Use boxed Double so it can be null if not used.
        Double triggerTime; // in seconds; may be null
        WaitCondition condition; // may be null
        Action action;
        boolean triggered;

        // Constructor for time-only trigger:
        WaitAction(double triggerTime, Action action) {
            this.triggerTime = triggerTime;
            this.action = action;
            this.condition = null;
            this.triggered = false;
        }

        // Constructor for condition-only trigger:
        WaitAction(WaitCondition condition, Action action) {
            this.triggerTime = null;
            this.condition = condition;
            this.action = action;
            this.triggered = false;
        }

        // Constructor for both a time and condition:
        WaitAction(double triggerTime, WaitCondition condition, Action action) {
            this.triggerTime = triggerTime;
            this.condition = condition;
            this.action = action;
            this.triggered = false;
        }

        // Determines if this wait action should trigger, based on elapsed time.
        boolean shouldTrigger(double elapsedSeconds) {
            boolean timeMet = (triggerTime != null && elapsedSeconds >= triggerTime);
            boolean conditionMet = (condition != null && condition.isMet());
            return timeMet || conditionMet;
        }
    }

    @FunctionalInterface
    public interface WaitCondition {
        boolean isMet();
    }

    private static class PathChainTask {
        PathChain pathChain;
        double waitTime; // maximum time to wait if condition is not met
        List<WaitAction> waitActions = new ArrayList<>();
        WaitCondition waitCondition; // optional overall condition

        PathChainTask(PathChain pathChain, double waitTime) {
            this.pathChain = pathChain;
            this.waitTime = waitTime;
        }

        // Add a "wait action," triggered at a certain second in the WAIT phase.
        PathChainTask addWaitAction(double triggerTime, Action action) {
            waitActions.add(new WaitAction(triggerTime, action));
            return this;
        }

        // Set a wait condition for early exit (optional).
        PathChainTask setWaitCondition(WaitCondition condition) {
            this.waitCondition = condition;
            return this;
        }

        void resetWaitActions() {
            for (WaitAction wa : waitActions) {
                wa.triggered = false;
            }
        }
    }

    // -------------------------------------------------------------------------
    // Robot & Timing
    // -------------------------------------------------------------------------
    private Follower follower;
    private Timer pathTimer, opModeTimer;
    private MotorActions motorActions;
    private MotorControl motorControl;

    // We'll consider the PathChain "done" at 99% param progress.
    private static final double PATH_COMPLETION_T = 0.985;

    private MotorControl.Limelight limelight;

    // -------------------------------------------------------------------------
    // Poses
    // -------------------------------------------------------------------------
    private final Pose startPose   = new Pose(9, 111, Math.toRadians(270));
    private final Pose scorePose   = new Pose(16, 128, Math.toRadians(315));
    private final Pose pickup1Pose = new Pose(20, 123, Math.toRadians(0));
    private final Pose pickup2Pose = new Pose(20, 130, Math.toRadians(0));
    private final Pose pickup3Pose = new Pose(24, 129, Math.toRadians(30));

    private final Pose parkPose        = new Pose(62, 97, Math.toRadians(315));
    private final Pose parkControlPose = new Pose(64.5, 116, Math.toRadians(270));

    // This field temporarily stores our dynamic (vision-based) path.
    private PathChain SubmersiblePose;

    // -------------------------------------------------------------------------
    // PathChains
    // -------------------------------------------------------------------------
    private PathChain scorePreload;
    private PathChain intake1, intake2, intake3;
    private PathChain score1, score2, score3;
    private PathChain parkChain;

    // -------------------------------------------------------------------------
    // A List of PathChainTask
    // -------------------------------------------------------------------------
    private final List<PathChainTask> tasks = new ArrayList<>();
    private int currentTaskIndex = 0;
    private int taskPhase = 0;

    // -------------------------------------------------------------------------
    // Build PathChains (with param callbacks for driving)
    // -------------------------------------------------------------------------
    private void buildPathChains() {

        intake1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(scorePose), new Point(pickup1Pose)))
                .setConstantHeadingInterpolation(pickup1Pose.getHeading())
                .addParametricCallback(0.6, () -> run(new SequentialAction(
                        motorActions.spin.eat(),
                        motorActions.intakeArm.Grab()
                )))
                .addParametricCallback(0.92, () -> run(motorActions.extendo.setTargetPosition(400)))
                .build();

        intake2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(scorePose), new Point(pickup2Pose)))
                .setConstantHeadingInterpolation(pickup2Pose.getHeading())
                .addParametricCallback(0.6, () -> run(new SequentialAction(
                        motorActions.spin.eat(),
                        motorActions.intakeArm.Grab()
                )))
                .addParametricCallback(0.92, () -> run(motorActions.extendo.setTargetPosition(350)))
                .build();

        intake3 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(scorePose), new Point(pickup3Pose)))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup3Pose.getHeading(), 50)
                .addParametricCallback(0.6, () -> run(new SequentialAction(
                        motorActions.spin.eat(),
                        motorActions.intakeArm.Grab()
                )))
                .addParametricCallback(0.92, () -> run(motorActions.extendo.setTargetPosition(400)))
                .build();

        score1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pickup1Pose), new Point(scorePose)))
                .setLinearHeadingInterpolation(pickup1Pose.getHeading(), scorePose.getHeading())
                .addParametricCallback(0.2, () -> run(new SequentialAction(
                        motorActions.outtakeSampleAuto()
                )))
                .build();

        score2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pickup2Pose), new Point(scorePose)))
                .setLinearHeadingInterpolation(pickup2Pose.getHeading(), scorePose.getHeading())
                .addParametricCallback(0.2, () -> run(new SequentialAction(
                        motorActions.outtakeSampleAuto()
                )))
                .build();

        score3 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pickup3Pose), new Point(scorePose)))
                .setLinearHeadingInterpolation(pickup3Pose.getHeading(), scorePose.getHeading())
                .addParametricCallback(0.2, () -> run(new SequentialAction(
                        motorActions.outtakeSampleAuto()
                )))
                .build();

        scorePreload = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(startPose), new Point(scorePose)))
                .addParametricCallback(0, () -> run(motorActions.outtakeSampleAuto()))
                .setLinearHeadingInterpolation(startPose.getHeading(), Math.toRadians(315))
                .build();

        // Preplanned park path.
        parkChain = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(scorePose), new Point(parkControlPose), new Point(parkPose)))
                .setLinearHeadingInterpolation(scorePose.getHeading(), parkPose.getHeading())
                .build();
    }

    // -------------------------------------------------------------------------
    // Create a dynamic alignment path based on the current pose and limelight data.
    // This method returns a new PathChain.
    // -------------------------------------------------------------------------
    private PathChain followDynamicAlignmentPath() {
        Pose currentPose = follower.getPose();
        Vector2d limelightAvg = limelight.getAverage();
        double rawOffset = limelightAvg.x;  // Make sure this is valid!

        // Set a threshold for valid offset; adjust as needed.
        double threshold = 0.1;

        // Fallback default offset (in inches) if limelight data is not valid.
        double defaultOffset = 5.0;

        if (Math.abs(rawOffset) < threshold) {
            // No valid target detected: use the fallback offset.
            telemetry.addData("Dynamic Align", "No valid limelight data, using fallback");
            rawOffset = defaultOffset;  // or choose to use 0.0 if you prefer no adjustment.
        }

        double heading = currentPose.getHeading();
        double offsetX = rawOffset * Math.cos(heading);
        double offsetY = rawOffset * Math.sin(heading);

        Pose targetPose = new Pose(
                currentPose.getX() + offsetX,
                currentPose.getY() + offsetY,
                currentPose.getHeading()
        );

        telemetry.addData("Dynamic Align: Raw Offset", rawOffset);
        telemetry.addData("Dynamic Align: Target Pose", targetPose);
        telemetry.update();

        return follower.pathBuilder()
                .addPath(new BezierLine(new Point(currentPose), new Point(targetPose)))
                .setConstantHeadingInterpolation(targetPose.getHeading())
                .build();
    }

    // -------------------------------------------------------------------------
    // Build the tasks (preplanned and dynamic tasks)
    // -------------------------------------------------------------------------
    private void buildTaskList() {
        tasks.clear();

        // Preload task.
        PathChainTask preloadTask = new PathChainTask(scorePreload, 1.5)
                .addWaitAction(0.5, new SequentialAction(
                        motorActions.outTakeLinkage.sample(),
                        new SleepAction(0.5),
                        motorActions.outtakeTransfer()
                ));
        tasks.add(preloadTask);

        // Pickup and scoring tasks.
        PathChainTask pickup1Task = new PathChainTask(intake1, 1.5)
                .addWaitAction(1, motorActions.intakeTransfer());
        tasks.add(pickup1Task);

        PathChainTask score1Task = new PathChainTask(score1, 1.3)
                .addWaitAction(0.5, new SequentialAction(
                        motorActions.outTakeLinkage.sample(),
                        new SleepAction(0.5),
                        motorActions.outtakeTransfer()
                ));
        tasks.add(score1Task);

        PathChainTask pickup2Task = new PathChainTask(intake2, 1.5)
                .addWaitAction(1, motorActions.intakeTransfer());
        tasks.add(pickup2Task);

        PathChainTask score2Task = new PathChainTask(score2, 1.3)
                .addWaitAction(0.5, new SequentialAction(
                        motorActions.outTakeLinkage.sample(),
                        new SleepAction(0.5),
                        motorActions.outtakeTransfer()
                ));
        tasks.add(score2Task);

        PathChainTask pickup3Task = new PathChainTask(intake3, 1.5)
                .addWaitAction(1, motorActions.intakeTransfer());
        tasks.add(pickup3Task);

        PathChainTask score3Task = new PathChainTask(score3, 1.3)
                .addWaitAction(0.5, new SequentialAction(
                        motorActions.outTakeLinkage.sample(),
                        new SleepAction(0.5),
                        motorActions.outtakeTransfer()
                ));
        tasks.add(score3Task);


    }

    // -------------------------------------------------------------------------
    // Main task-runner logic: DRIVING and WAITING phases.
    // -------------------------------------------------------------------------
    private void runTasks() {
        if (currentTaskIndex >= tasks.size()) {
            return; // All tasks completed.
        }

        PathChainTask currentTask = tasks.get(currentTaskIndex);

        switch (taskPhase) {
            case 0: // DRIVING phase.
                if (!follower.isBusy()) {
                    follower.followPath(currentTask.pathChain, true);
                    pathTimer.resetTimer();
                    currentTask.resetWaitActions();
                }
                double tValue = follower.getCurrentTValue(); // Progress value in [0..1]
                if (tValue >= PATH_COMPLETION_T) {
                    pathTimer.resetTimer();
                    taskPhase = 1;
                }
                break;

            case 1: // WAITING phase.
                double waitElapsed = pathTimer.getElapsedTimeSeconds();
                for (WaitAction wa : currentTask.waitActions) {
                    if (!wa.triggered && wa.shouldTrigger(waitElapsed)) {
                        run(wa.action);
                        wa.triggered = true;
                    }
                }
                if (waitElapsed >= currentTask.waitTime) {
                    currentTaskIndex++;
                    taskPhase = 0;
                }
                break;
        }
    }

    // -------------------------------------------------------------------------
    // Standard OpMode methods
    // -------------------------------------------------------------------------
    @Override
    public void init() {
        pathTimer = new Timer();
        opModeTimer = new Timer();
        opModeTimer.resetTimer();

        motorControl = new MotorControl(hardwareMap);
        motorActions = new MotorActions(motorControl);

        Constants.setConstants(FConstants.class, LConstants.class);

        limelight = new MotorControl.Limelight(hardwareMap, telemetry);

        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);

        run(motorActions.outTakeClaw.Close());

        // Build the path geometry and tasks.
        buildPathChains();
        buildTaskList();
    }

    @Override
    public void start() {
        opModeTimer.resetTimer();
        currentTaskIndex = 0;
        taskPhase = 0;
        pathTimer.resetTimer();
        run(motorActions.intakePivot.Transfer());
        run(motorActions.intakeArm.Intake());
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
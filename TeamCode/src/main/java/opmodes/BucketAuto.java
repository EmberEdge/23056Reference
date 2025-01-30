package opmodes;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
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
        double triggerTime; // seconds into the waiting phase
        Action action;
        boolean triggered;

        WaitAction(double triggerTime, Action action) {
            this.triggerTime = triggerTime;
            this.action = action;
            this.triggered = false;
        }
    }

    // -------------------------------------------------------------------------
    // PathChainTask: holds a PathChain (with param callbacks) and a WAIT phase
    // -------------------------------------------------------------------------
    private static class PathChainTask {
        PathChain pathChain;
        double waitTime; // how long to wait after the chain
        List<WaitAction> waitActions = new ArrayList<>();

        PathChainTask(PathChain pathChain, double waitTime) {
            this.pathChain = pathChain;
            this.waitTime = waitTime;
        }

        // Add a "wait action," triggered at a certain second in the WAIT phase
        PathChainTask addWaitAction(double triggerTime, Action action) {
            waitActions.add(new WaitAction(triggerTime, action));
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

    // We'll consider the PathChain "done" at 99% param progress
    private static final double PATH_COMPLETION_T = 0.98;

    // -------------------------------------------------------------------------
    // Poses
    // -------------------------------------------------------------------------
    private final Pose startPose   = new Pose(9,  111, Math.toRadians(270));
    private final Pose scorePose   = new Pose(18, 126, Math.toRadians(315));
    private final Pose pickup1Pose = new Pose(20, 123, Math.toRadians(0));
    private final Pose pickup2Pose = new Pose(20, 131.5, Math.toRadians(0));
    private final Pose pickup3Pose = new Pose(22, 129, Math.toRadians(36));

    private final Pose parkPose        = new Pose(60, 98, Math.toRadians(90));
    private final Pose parkControlPose = new Pose(60, 98, Math.toRadians(90));

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
                .addPath(new BezierLine(
                        new Point(scorePose),
                        new Point(pickup1Pose)))
                //.setPathEndHeadingConstraint(10)
                .setLinearHeadingInterpolation(Math.toRadians(scorePose.getHeading()), Math.toRadians(pickup1Pose.getHeading()))
                .addParametricCallback(0.6, () -> run(
                        new SequentialAction(
                                motorActions.spin.eat(),
                                motorActions.intakeArm.Grab()
                        )))
                .addParametricCallback(0.9, () -> run(motorActions.extendo.setTargetPosition(450)))

                .build();

        intake2 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Point(scorePose),
                        new Point(pickup2Pose)))
                .setLinearHeadingInterpolation(Math.toRadians(scorePose.getHeading()), Math.toRadians(pickup2Pose.getHeading()))
               // .setPathEndHeadingConstraint(10)
                .addParametricCallback(0.6, () -> run(
                        new SequentialAction(
                                motorActions.spin.eat(),
                                motorActions.intakeArm.Grab()
                        )))
                .addParametricCallback(0.9, () -> run(motorActions.extendo.setTargetPosition(450)))
                .build();

        intake3 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Point(scorePose),
                        new Point(pickup3Pose)))
                .setLinearHeadingInterpolation(Math.toRadians(scorePose.getHeading()), Math.toRadians(pickup3Pose.getHeading()))
                //.setPathEndHeadingConstraint(10)
                .addParametricCallback(0.6, () -> run(
                        new SequentialAction(
                                motorActions.spin.eat(),
                                motorActions.intakeArm.Grab()
                        )))
                .addParametricCallback(0.9, () -> run(motorActions.extendo.setTargetPosition(450)))

                .build();

        score1 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Point(pickup1Pose),
                        new Point(scorePose)))
                .setLinearHeadingInterpolation(Math.toRadians(pickup1Pose.getHeading()), Math.toRadians(scorePose.getHeading()),700)
                //.setPathEndHeadingConstraint(10)
                .addParametricCallback(0,() -> run(new SequentialAction(motorActions.intakeTransfer(),
                        new SleepAction(0.1),
                        motorActions.outtakeSample()
                        )))
                .build();

        score2 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Point(pickup2Pose),
                        new Point(scorePose)))
                .setLinearHeadingInterpolation(Math.toRadians(pickup2Pose.getHeading()), Math.toRadians(scorePose.getHeading()),700)
                //.setPathEndHeadingConstraint(10)
                .addParametricCallback(0,() -> run(new SequentialAction(motorActions.intakeTransfer(),
                        new SleepAction(0.1),
                        motorActions.outtakeSample()
                )))
                .build();

        score3 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Point(pickup3Pose),
                        new Point(scorePose)))
                .setLinearHeadingInterpolation(Math.toRadians(pickup3Pose.getHeading()), Math.toRadians(scorePose.getHeading()))
                //.setPathEndHeadingConstraint(10)
                .addParametricCallback(0,() -> run(new SequentialAction(motorActions.intakeTransfer(),
                        new SleepAction(0.1),
                        motorActions.outtakeSample()
                )))
                .build();


        scorePreload = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Point(startPose),
                        new Point(scorePose)
                ))
                .addParametricCallback(0, () -> run(new ParallelAction(motorActions.outtakeSample(),  motorActions.extendo.setTargetPosition(200))))
                .setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading())
                .build();



        // Park
        parkChain = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Point(scorePose),
                        new Point(parkControlPose),
                        new Point(parkPose)
                ))
                .setLinearHeadingInterpolation(scorePose.getHeading(), parkPose.getHeading())
                .build();
    }

    // -------------------------------------------------------------------------
    // Build the tasks (time-based triggers in WAIT, param-based in DRIVING)
    // -------------------------------------------------------------------------
    private void buildTaskList() {
        tasks.clear();

        PathChainTask preloadTask = new PathChainTask(scorePreload, 0.5).addWaitAction( 0.4,
                motorActions.outtakeTransfer()
        );

        tasks.add(preloadTask);

        PathChainTask pickup1Task = new PathChainTask(intake1, 1);

        tasks.add(pickup1Task);

        PathChainTask score1Task = new PathChainTask(score1, 0.4);

        tasks.add(score1Task);
/*
        PathChainTask pickup2Task = new PathChainTask(intake2, 1);

        tasks.add(pickup2Task);

        PathChainTask score2Task = new PathChainTask(score2, 2);

        tasks.add(score2Task);

        PathChainTask pickup3Task = new PathChainTask(intake3, 0.4);

        tasks.add(pickup3Task);

        PathChainTask score3Task = new PathChainTask(score3, 2);

        tasks.add(score3Task);*/



        // Optional final parking
        // tasks.add(new PathChainTask(parkChain, 0.0));
    }

    // -------------------------------------------------------------------------
    // Main task-runner logic:
    //   - DRIVING phase => param-based triggers from chain
    //   - WAITING phase => time-based triggers from waitActions
    // -------------------------------------------------------------------------
    private void runTasks() {
        if (currentTaskIndex >= tasks.size()) {
            return; // all done
        }

        PathChainTask currentTask = tasks.get(currentTaskIndex);

        switch (taskPhase) {
            case 0: // == DRIVING ==
                // If we aren't following yet, start
                if (!follower.isBusy()) {
                    follower.followPath(currentTask.pathChain, true);
                    pathTimer.resetTimer();

                    // We only "reset" the *wait* actions here.
                    // Param-based callbacks are attached in the chain already.
                    currentTask.resetWaitActions();
                }

                double tValue = follower.getCurrentTValue(); // param progress [0..1]
                // NOTE: Param-based callbacks happen automatically in the `Follower`
                // when tValue crosses the callback thresholds.

                // Consider chain done at 99%
                if (tValue >= PATH_COMPLETION_T) {
                    // Move to WAIT
                    pathTimer.resetTimer();
                    taskPhase = 1;
                }
                break;

            case 1: // == WAITING ==
                double waitElapsed = pathTimer.getElapsedTimeSeconds();

                // Trigger any "wait actions" whose time has arrived
                for (WaitAction wa : currentTask.waitActions) {
                    if (!wa.triggered && waitElapsed >= wa.triggerTime) {
                        run(wa.action); // schedule this action
                        wa.triggered = true;
                    }
                }

                // Once we've fully waited out the entire waitTime, move on
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

        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);

        // Build your chain geometry (with param callbacks)
        buildPathChains();
        // Build the tasks (with wait-based triggers)
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
        // 1) Let ActionOpMode handle running actions
        //    (this calls updateAsync(...) under the hood)
        super.loop();

        // 2) Update the follower
        follower.update();



        // 4) runTasks => handle param-based transitions + time-based wait triggers
        runTasks();

        // 5) (optional) Update hardware
        motorControl.update();

        // Telemetry
        telemetry.addData("Task Index", currentTaskIndex + "/" + tasks.size());
        telemetry.addData("Phase", (taskPhase == 0) ? "DRIVE" : "WAIT");
        telemetry.addData("T Value", follower.getCurrentTValue());
        telemetry.addData("Wait Timer", pathTimer.getElapsedTimeSeconds());
        telemetry.addData("Running Actions", runningActions.size());
        telemetry.update();
    }
}

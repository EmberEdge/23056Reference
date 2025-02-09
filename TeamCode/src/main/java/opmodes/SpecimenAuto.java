package opmodes;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
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

@Autonomous(name = "Specimen Auto")
public class SpecimenAuto extends ActionOpMode {

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
    private static final double PATH_COMPLETION_T = 0.985;

    // -------------------------------------------------------------------------
    // Poses
    // -------------------------------------------------------------------------
    private final Pose startPose   = new Pose(9,  58, Math.toRadians(0));
    private final Pose preloadPose  = new Pose(30, 69, Math.toRadians(0));
    private final Pose scorePose   = new Pose(34.5, 69, Math.toRadians(0));
    private final Pose scorePose1   = new Pose(32, 70, Math.toRadians(0));
    private final Pose scorePose2   = new Pose(32, 71, Math.toRadians(0));
    private final Pose scorePose3   = new Pose(32, 72, Math.toRadians(0));
    private final Pose scorePose4   = new Pose(32, 73, Math.toRadians(0));
    private final Pose pickup1Pose = new Pose(28, 45, Math.toRadians(311));
    private final Pose pickup1Control = new Pose(22, 76, Math.toRadians(311));
    private final Pose pickup2Pose = new Pose(25, 35, Math.toRadians(315));
    private final Pose pickup3Pose = new Pose(28, 30, Math.toRadians(305));
    private final Pose depositPose1 = new Pose(25, 44, Math.toRadians(250));
    private final Pose depositPose2 = new Pose(25, 40, Math.toRadians(250));

    private final Pose intake = new Pose(11,35, Math.toRadians(0));
    private final Pose intakeControl1 = new Pose(32,35, Math.toRadians(0));
    private final Pose intakeControl2 = new Pose(5,74, Math.toRadians(0));

    private final Pose intakeControl3 = new Pose(30,34, Math.toRadians(0));

    private final Pose parkPose        = new Pose(11, 22, Math.toRadians(90));
    private final Pose parkControlPose = new Pose(12, 74, Math.toRadians(90));

    // -------------------------------------------------------------------------
    // PathChains
    // -------------------------------------------------------------------------
    private PathChain scorePreload;
    private PathChain grabPickup1, grabPickup2, grabPickup3;
    private PathChain depositHP1, depositHP2, depositHP3;
    private PathChain intake1, intake2;
    private PathChain score, score1, score2, score3, score4;
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
                .addPath(new BezierCurve(
                        new Point(depositPose1),
                        new Point(intakeControl3),
                        new Point(intake)))
                .setLinearHeadingInterpolation(Math.toRadians(depositPose1.getHeading()), Math.toRadians(intake.getHeading()))
                .addParametricCallback(0.5, ()->motorControl.spin.setPower(0))
                .addParametricCallback(0, ()-> motorActions.intakeSpecimen())
                .build();

        score = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Point(intake),
                        new Point(intakeControl1),
                        new Point(intakeControl2),
                        new Point(scorePose)))
                .setLinearHeadingInterpolation(Math.toRadians(intake.getHeading()), Math.toRadians(scorePose.getHeading()))
                .addParametricCallback(0, ()-> run(new ParallelAction( motorActions.intakePivot.Transfer(),
                        motorActions.intakeArm.Intake())))
                .build();

        score1 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Point(intake),
                        new Point(intakeControl1),
                        new Point(intakeControl2),
                        new Point(scorePose1)))
                .setLinearHeadingInterpolation(Math.toRadians(intake.getHeading()), Math.toRadians(scorePose1.getHeading()))
                .addParametricCallback(0, ()-> run(new ParallelAction( motorActions.intakePivot.Transfer(),
                        motorActions.intakeArm.Intake())))
                .build();

        score2 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Point(intake),
                        new Point(intakeControl1),
                        new Point(intakeControl2),
                        new Point(scorePose2)))
                .setLinearHeadingInterpolation(Math.toRadians(intake.getHeading()), Math.toRadians(scorePose2.getHeading()))
                .addParametricCallback(0, ()-> run(new ParallelAction( motorActions.intakePivot.Transfer(),
                        motorActions.intakeArm.Intake())))
                .build();

        score3 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Point(intake),
                        new Point(intakeControl1),
                        new Point(intakeControl2),
                        new Point(scorePose3)))
                .setLinearHeadingInterpolation(Math.toRadians(intake.getHeading()), Math.toRadians(scorePose2.getHeading()))
                .addParametricCallback(0, ()-> run(new ParallelAction( motorActions.intakePivot.Transfer(),
                        motorActions.intakeArm.Intake())))
                .build();

        score4 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Point(intake),
                        new Point(intakeControl1),
                        new Point(intakeControl2),
                        new Point(scorePose4)))
                .setLinearHeadingInterpolation(Math.toRadians(intake.getHeading()), Math.toRadians(scorePose2.getHeading()))
                .addParametricCallback(0, ()-> run(new ParallelAction( motorActions.intakePivot.Transfer(),
                        motorActions.intakeArm.Intake())))
                .build();


        intake2 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Point(scorePose),
                        new Point(intakeControl2),
                        new Point(intakeControl1),
                        new Point(intake)))
                .setLinearHeadingInterpolation(Math.toRadians(scorePose.getHeading()), Math.toRadians(intake.getHeading()))
                .addParametricCallback(0.05, () -> run(motorActions.intakeSpecimen()))
                .build();

        scorePreload = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Point(startPose),
                        new Point(preloadPose)
                ))
                .setLinearHeadingInterpolation(startPose.getHeading(), preloadPose.getHeading())
                .setZeroPowerAccelerationMultiplier(2)
                .addParametricCallback(0, () -> run(
                        new SequentialAction(
                                motorActions.intakePivot.Transfer(),
                                motorActions.intakeArm.Intake(),
                                motorActions.outtakeSpecimenAuto()
                        )
                ))
                .build();

        // Similarly build other chains
        grabPickup1 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Point(scorePose),
                        new Point(pickup1Control),
                        new Point(pickup1Pose)

                ))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup1Pose.getHeading())
                .addParametricCallback(0.4, () -> run(
                        new SequentialAction(
                                motorActions.spin.eat(),
                                motorActions.intakeArm.Grab()
                        )))
                .build();

        grabPickup2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(scorePose), new Point(pickup2Pose)))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup2Pose.getHeading())
                .build();

        grabPickup3 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(scorePose), new Point(pickup3Pose)))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup3Pose.getHeading())
                .build();

        // Deposit #1, #2, #3
        depositHP1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pickup1Pose), new Point(depositPose1)))
                .setLinearHeadingInterpolation(pickup1Pose.getHeading(), depositPose1.getHeading())
                .addParametricCallback(0.8, ()->motorControl.spin.setPower(-1.0))
                .build();

        depositHP2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pickup2Pose), new Point(depositPose1)))
                .setLinearHeadingInterpolation(pickup2Pose.getHeading(), depositPose1.getHeading())
                .addParametricCallback(0.8, ()->motorControl.spin.setPower(-1.0))
                .build();

        depositHP3 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pickup3Pose), new Point(depositPose2)))
                .setLinearHeadingInterpolation(pickup3Pose.getHeading(), depositPose2.getHeading())
                .addParametricCallback(0.8, ()->motorControl.spin.setPower(-1.0))
                .build();

        // Park
        parkChain = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Point(scorePose4),
                        new Point(parkControlPose),
                        new Point(parkPose)
                ))
                .setLinearHeadingInterpolation(scorePose.getHeading(), parkPose.getHeading())
                .setZeroPowerAccelerationMultiplier(6)
                .build();
    }

    // -------------------------------------------------------------------------
    // Build the tasks (time-based triggers in WAIT, param-based in DRIVING)
    // -------------------------------------------------------------------------
    private void buildTaskList() {
        tasks.clear();

        PathChainTask preloadTask = new PathChainTask(scorePreload, 1)
                .addWaitAction(0.2, motorActions.depositSpecimen());

        tasks.add(preloadTask);

        PathChainTask pickUpTask1 = new PathChainTask(grabPickup1, 0.4).addWaitAction(0,
                motorActions.extendo.setTargetPosition(500)
        ).addWaitAction(0.39, motorActions.extendo.setTargetPosition(450));
        tasks.add(pickUpTask1);

        PathChainTask depositTask1 = new PathChainTask(depositHP1, 0.2).addWaitAction(0,
                motorActions.extendo.setTargetPosition(0)
        );
        tasks.add(depositTask1);

        PathChainTask pickUpTask2 = new PathChainTask(grabPickup2, 0.4).addWaitAction(0,
                new ParallelAction(
                        motorActions.intakeArm.Grab(),
                        motorActions.spin.eat(),
                        motorActions.extendo.setTargetPosition(500)

                )
        );
        tasks.add(pickUpTask2);
        PathChainTask depositTask2 = new PathChainTask(depositHP2, 0.2).addWaitAction(0,
                motorActions.extendo.setTargetPosition(0)
        );
        tasks.add(depositTask2);

        PathChainTask pickUpTask3 = new PathChainTask(grabPickup3, 0.4).addWaitAction(0,
                new ParallelAction(
                        motorActions.intakeArm.Grab(),
                        motorActions.spin.eat(),
                        motorActions.extendo.setTargetPosition(550)

                )
        );
        tasks.add(pickUpTask3);
        PathChainTask depositTask3 = new PathChainTask(depositHP3, 0.2).addWaitAction(0,
                motorActions.extendo.setTargetPosition(0)
        );
        tasks.add(depositTask3);


        PathChainTask intakeTask1 = new PathChainTask(intake1, 0.3).addWaitAction(0.05,
                motorActions.outtakeSpecimen()
        );

        tasks.add(intakeTask1); // intake from human player

        tasks.add(new PathChainTask(score1, 0.35).addWaitAction(0.01,
                motorActions.depositSpecimen()
        )); // score 1

        tasks.add(new PathChainTask(intake2, 0.35).addWaitAction(0.01,
                motorActions.outtakeSpecimen()
        ));// intake from human player

        tasks.add(new PathChainTask(score2, 0.35).addWaitAction(0.01,
                motorActions.depositSpecimen()
        )); // score 2


        tasks.add(new PathChainTask(intake2, 0.35).addWaitAction(0.01,
                motorActions.outtakeSpecimen()
        ));// intake from human player

        tasks.add(new PathChainTask(score3, 0.4).addWaitAction(0.01,
                motorActions.depositSpecimen()
        )); // score 3

        tasks.add(new PathChainTask(intake2, 0.4).addWaitAction(0.01,
                motorActions.outtakeSpecimen()
        ));// intake from human player

        tasks.add(new PathChainTask(score4, 0.2).addWaitAction(0.01,
                motorActions.depositSpecimen()
        )); // score 4


        tasks.add(new PathChainTask(parkChain, 0.0));
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

        run(motorActions.outTakeClaw.Close());

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

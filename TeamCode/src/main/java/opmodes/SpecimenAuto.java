package opmodes;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import helpers.data.Enums;
import helpers.hardware.MotorControl;
import helpers.hardware.actions.MotorActions;
import helpers.hardware.actions.PathChainAutoOpMode;
import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

/**
 * SpecimenAuto is an autonomous OpMode that uses a series of PathChainTasks.
 * It extends PathChainAutoOpMode so that you only need to override buildPathChains() and buildTaskList(),
 * plus the dummy path-follower methods.
 */
@Autonomous(name = "Specimen Auto")
public class SpecimenAuto extends PathChainAutoOpMode {

    // -------- Hardware & Helper Fields --------
    private Follower follower;
    private MotorActions motorActions;
    private MotorControl motorControl;
    private MotorControl.Limelight limelight;
    private Timer opModeTimer;  // additional timer if desired

    // -------- Poses --------
    private final Pose startPose = new Pose(9, 58, Math.toRadians(0));
    private final Pose preloadPose = new Pose(30, 69, Math.toRadians(0));
    private final Pose scorePose = new Pose(34.5, 69, Math.toRadians(0));
    private final Pose scorePose1 = new Pose(32, 70, Math.toRadians(0));
    private final Pose scorePose2 = new Pose(32, 71, Math.toRadians(0));
    private final Pose scorePose3 = new Pose(32, 72, Math.toRadians(0));
    private final Pose scorePose4 = new Pose(32, 73, Math.toRadians(0));
    private final Pose pickup1Pose = new Pose(28, 45, Math.toRadians(311));
    private final Pose pickup1Control = new Pose(22, 76, Math.toRadians(311));
    private final Pose pickup2Pose = new Pose(25, 35, Math.toRadians(315));
    private final Pose pickup3Pose = new Pose(28, 30, Math.toRadians(305));
    private final Pose depositPose1 = new Pose(25, 44, Math.toRadians(250));
    private final Pose depositPose2 = new Pose(25, 40, Math.toRadians(250));
    private final Pose intake = new Pose(11, 35, Math.toRadians(0));
    private final Pose intakeControl1 = new Pose(32, 35, Math.toRadians(0));
    private final Pose intakeControl2 = new Pose(5, 74, Math.toRadians(0));
    private final Pose intakeControl3 = new Pose(30, 34, Math.toRadians(0));
    private final Pose parkPose = new Pose(11, 22, Math.toRadians(90));
    private final Pose parkControlPose = new Pose(12, 74, Math.toRadians(90));

    // -------- PathChains --------
    private PathChain scorePreload;
    private PathChain grabPickup1, grabPickup2, grabPickup3;
    private PathChain depositHP1, depositHP2, depositHP3;
    private PathChain intake1, intake2;
    private PathChain score, score1, score2, score3, score4;
    private PathChain parkChain;

    // -------- Override buildPathChains() --------
    @Override
    protected void buildPathChains() {
        // Intake path from deposit to intake.
        intake1 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Point(depositPose1),
                        new Point(intakeControl3),
                        new Point(intake)))
                .setLinearHeadingInterpolation(Math.toRadians(depositPose1.getHeading()),
                        Math.toRadians(intake.getHeading()))
                .addParametricCallback(0.5, () -> motorControl.spin.setPower(0))
                .addParametricCallback(0, () -> motorActions.intakeSpecimen())
                .build();

        // Score paths.
        score = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Point(intake),
                        new Point(intakeControl1),
                        new Point(intakeControl2),
                        new Point(scorePose)))
                .setLinearHeadingInterpolation(Math.toRadians(intake.getHeading()),
                        Math.toRadians(scorePose.getHeading()))
                .addParametricCallback(0, () -> run(new ParallelAction(
                        motorActions.intakePivot.Transfer(),
                        motorActions.intakeArm.Intake())))
                .addParametricCallback(0, () -> run(motorActions.outtakeSpecimen()))
                .build();

        score1 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Point(intake),
                        new Point(intakeControl1),
                        new Point(intakeControl2),
                        new Point(scorePose1)))
                .setLinearHeadingInterpolation(Math.toRadians(intake.getHeading()),
                        Math.toRadians(scorePose1.getHeading()))
                .addParametricCallback(0, () -> run(new ParallelAction(
                        motorActions.intakePivot.Transfer(),
                        motorActions.intakeArm.Intake())))
                .addParametricCallback(0, () -> run(motorActions.outtakeSpecimen()))
                .build();

        score2 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Point(intake),
                        new Point(intakeControl1),
                        new Point(intakeControl2),
                        new Point(scorePose2)))
                .setLinearHeadingInterpolation(Math.toRadians(intake.getHeading()),
                        Math.toRadians(scorePose2.getHeading()))
                .addParametricCallback(0, () -> run(new ParallelAction(
                        motorActions.intakePivot.Transfer(),
                        motorActions.intakeArm.Intake())))
                .addParametricCallback(0, () -> run(motorActions.outtakeSpecimen()))
                .build();

        score3 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Point(intake),
                        new Point(intakeControl1),
                        new Point(intakeControl2),
                        new Point(scorePose3)))
                .setLinearHeadingInterpolation(Math.toRadians(intake.getHeading()),
                        Math.toRadians(scorePose3.getHeading()))
                .addParametricCallback(0, () -> run(new ParallelAction(
                        motorActions.intakePivot.Transfer(),
                        motorActions.intakeArm.Intake())))
                .addParametricCallback(0, () -> run(motorActions.outtakeSpecimen()))
                .build();

        score4 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Point(intake),
                        new Point(intakeControl1),
                        new Point(intakeControl2),
                        new Point(scorePose4)))
                .setLinearHeadingInterpolation(Math.toRadians(intake.getHeading()),
                        Math.toRadians(scorePose4.getHeading()))
                .addParametricCallback(0, () -> run(new ParallelAction(
                        motorActions.intakePivot.Transfer(),
                        motorActions.intakeArm.Intake())))
                .addParametricCallback(0, () -> run(motorActions.outtakeSpecimen()))
                .build();

        // Intake path from score to intake.
        intake2 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Point(scorePose),
                        new Point(intakeControl2),
                        new Point(intakeControl1),
                        new Point(intake)))
                .setLinearHeadingInterpolation(Math.toRadians(scorePose.getHeading()),
                        Math.toRadians(intake.getHeading()))
                .addParametricCallback(0.05, () -> run(motorActions.intakeSpecimen()))
                .build();

        // Preload path.
        scorePreload = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Point(startPose),
                        new Point(preloadPose)))
                .setLinearHeadingInterpolation(startPose.getHeading(), preloadPose.getHeading())
                .setZeroPowerAccelerationMultiplier(2)
                .addParametricCallback(0, () -> run(new SequentialAction(
                        motorActions.intakePivot.Transfer(),
                        motorActions.intakeArm.Intake(),
                        motorActions.outtakeSpecimen())))
                .build();

        // Grab (pickup) paths.
        grabPickup1 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Point(scorePose),
                        new Point(pickup1Control),
                        new Point(pickup1Pose)))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup1Pose.getHeading())
                .addParametricCallback(0.4, () -> run(new SequentialAction(
                        motorActions.spin.eat(),
                        motorActions.intakeArm.Grab())))
                .build();

        grabPickup2 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Point(scorePose),
                        new Point(pickup2Pose)))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup2Pose.getHeading())
                .build();

        grabPickup3 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Point(scorePose),
                        new Point(pickup3Pose)))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup3Pose.getHeading())
                .build();

        // Deposit paths.
        depositHP1 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Point(pickup1Pose),
                        new Point(depositPose1)))
                .setLinearHeadingInterpolation(pickup1Pose.getHeading(), depositPose1.getHeading())
                .addParametricCallback(0.8, () -> motorControl.spin.setPower(-1.0))
                .build();

        depositHP2 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Point(pickup2Pose),
                        new Point(depositPose1)))
                .setLinearHeadingInterpolation(pickup2Pose.getHeading(), depositPose1.getHeading())
                .addParametricCallback(0.8, () -> motorControl.spin.setPower(-1.0))
                .build();

        depositHP3 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Point(pickup3Pose),
                        new Point(depositPose2)))
                .setLinearHeadingInterpolation(pickup3Pose.getHeading(), depositPose2.getHeading())
                .addParametricCallback(0.8, () -> motorControl.spin.setPower(-1.0))
                .build();

        // Park path.
        parkChain = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Point(scorePose4),
                        new Point(parkControlPose),
                        new Point(parkPose)))
                .setLinearHeadingInterpolation(scorePose.getHeading(), parkPose.getHeading())
                .setZeroPowerAccelerationMultiplier(6)
                .build();
    }

    // -------- Override buildTaskList() --------
    @Override
    protected void buildTaskList() {
        tasks.clear();

        // Preload task.
        PathChainTask preloadTask = new PathChainTask(scorePreload, 1)
                .addWaitAction(0.2, motorActions.depositSpecimen());
        tasks.add(preloadTask);

        // Pickup task 1.
        PathChainTask pickUpTask1 = new PathChainTask(grabPickup1, 0.4)
                .addWaitAction(0, motorActions.extendo.setTargetPosition(500))
                .addWaitAction(0.39, motorActions.extendo.setTargetPosition(450));
        tasks.add(pickUpTask1);

        // Deposit task 1.
        PathChainTask depositTask1 = new PathChainTask(depositHP1, 0.2)
                .addWaitAction(0, motorActions.extendo.setTargetPosition(0));
        tasks.add(depositTask1);

        // Pickup task 2.
        PathChainTask pickUpTask2 = new PathChainTask(grabPickup2, 0.4)
                .addWaitAction(0, new ParallelAction(
                        motorActions.intakeArm.Grab(),
                        motorActions.spin.eat(),
                        motorActions.extendo.setTargetPosition(500)
                ));
        tasks.add(pickUpTask2);

        // Deposit task 2.
        PathChainTask depositTask2 = new PathChainTask(depositHP2, 0.2)
                .addWaitAction(0, motorActions.extendo.setTargetPosition(0));
        tasks.add(depositTask2);

        // Pickup task 3.
        PathChainTask pickUpTask3 = new PathChainTask(grabPickup3, 0.4)
                .addWaitAction(0, new ParallelAction(
                        motorActions.intakeArm.Grab(),
                        motorActions.spin.eat(),
                        motorActions.extendo.setTargetPosition(550)
                ));
        tasks.add(pickUpTask3);

        // Deposit task 3.
        PathChainTask depositTask3 = new PathChainTask(depositHP3, 0.2)
                .addWaitAction(0, motorActions.extendo.setTargetPosition(0));
        tasks.add(depositTask3);

        // Intake task 1.
        PathChainTask intakeTask1 = new PathChainTask(intake1, 0.2)
                .addWaitAction(0.05, motorActions.outTakeClaw.Close());
        tasks.add(intakeTask1);

        // Score task 1.
        tasks.add(new PathChainTask(score1, 0.35)
                .addWaitAction(0.01, motorActions.depositSpecimen()));

        // Intake task 2.
        tasks.add(new PathChainTask(intake2, 0.2)
                .addWaitAction(0.01, motorActions.outTakeClaw.Close()));

        // Score task 2.
        tasks.add(new PathChainTask(score2, 0.35)
                .addWaitAction(0.01, motorActions.depositSpecimen()));

        // Intake task 3.
        tasks.add(new PathChainTask(intake2, 0.2)
                .addWaitAction(0.01, motorActions.outTakeClaw.Close()));

        // Score task 3.
        tasks.add(new PathChainTask(score3, 0.4)
                .addWaitAction(0.01, motorActions.depositSpecimen()));

        // Intake task 4.
        tasks.add(new PathChainTask(intake2, 0.2)
                .addWaitAction(0.05, motorActions.outTakeClaw.Close()));

        // Score task 4.
        tasks.add(new PathChainTask(score4, 0.2)
                .addWaitAction(0.01, motorActions.depositSpecimen()));

        // Park task.
        tasks.add(new PathChainTask(parkChain, 0.0));
    }

    // -------- Override dummy follower methods --------
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
        // Cast the task's pathChain to PathChain and command the follower to follow it.
        follower.followPath((PathChain) task.pathChain, false);
    }

    // -------- Standard OpMode Lifecycle Methods --------
    @Override
    public void init() {
        opModeTimer = new Timer();
        opModeTimer.resetTimer();
        pathTimer.resetTimer();

        motorControl = new MotorControl(hardwareMap);
        motorActions = new MotorActions(motorControl);
        Constants.setConstants(FConstants.class, LConstants.class);

        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);
        run(motorActions.outTakeClaw.Close());

        // Build the paths and tasks.
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

package opmodes;

import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

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
@Autonomous(name = "Specimen Auto Push")
public class SpecimenAutoPush extends PathChainAutoOpMode {

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

        // Preload path.
        scorePreload = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Point(startPose),
                        new Point(preloadPose)))
                .setLinearHeadingInterpolation(startPose.getHeading(), preloadPose.getHeading())
                .setZeroPowerAccelerationMultiplier(5)
                .addParametricCallback(0, () -> run(new SequentialAction(
                        motorActions.intakePivot.Transfer(),
                        motorActions.intakeArm.Intake(),
                        motorActions.outtakeSpecimen())))
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
        follower.followPath((PathChain) task.pathChain, true);
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

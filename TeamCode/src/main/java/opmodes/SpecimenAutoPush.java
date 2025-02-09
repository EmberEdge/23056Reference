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
import helpers.hardware.actions.MotorActions;
import helpers.hardware.actions.PathChainAutoOpMode;
import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

/**
 * SpecimenAuto is an autonomous OpMode that uses a single generated PathChain.
 * The path is defined using the new segments (from your GeneratedPath class).
 * A single task is created using that path chain, with a wait-action in its waiting phase.
 */
@Autonomous(name = "Specimen Auto")
public class SpecimenAutoPush extends PathChainAutoOpMode {

    // -------- Hardware & Helper Fields --------
    private Follower follower;
    private MotorActions motorActions;
    private MotorControl motorControl;
    private Timer opModeTimer;

    // -------- Poses --------
    // (You can adjust these as needed; here we use the start and preload poses from your example.)
    private final Pose startPose = new Pose(9, 58, Math.toRadians(0));
    private final Pose preloadPose = new Pose(37, 69, Math.toRadians(0)); // End of Line 1 in GeneratedPath

    // -------- Generated PathChain --------
    private PathChain generatedPathChain;

    // -------- Override buildPathChains() --------
    @Override
    protected void buildPathChains() {
        generatedPathChain = follower.pathBuilder()
                // --- Line 1: A straight BezierLine from (9,58) to (37,69)
                .addPath(
                        new BezierLine(
                                new Point(9.000, 58.000, Point.CARTESIAN),
                                new Point(37.000, 69.000, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                // --- Line 2: A BezierCurve with four points.
                .addPath(
                        new BezierCurve(
                                new Point(37.000, 69.000, Point.CARTESIAN),
                                new Point(-15.000, 25.000, Point.CARTESIAN),
                                new Point(136.000, 33.000, Point.CARTESIAN),
                                new Point(22.000, 23.000, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                // --- Line 3: Another BezierCurve.
                .addPath(
                        new BezierCurve(
                                new Point(22.000, 23.000, Point.CARTESIAN),
                                new Point(70.800, 36.600, Point.CARTESIAN),
                                new Point(94.000, 6.200, Point.CARTESIAN),
                                new Point(22.000, 15.000, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                // --- Line 4: Final BezierCurve segment.
                .addPath(
                        new BezierCurve(
                                new Point(22.000, 15.000, Point.CARTESIAN),
                                new Point(88.500, 15.500, Point.CARTESIAN),
                                new Point(72.000, 7.600, Point.CARTESIAN),
                                new Point(22.000, 9.500, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();
    }

    // -------- Override buildTaskList() --------
    @Override
    protected void buildTaskList() {
        tasks.clear();
        PathChainTask mainTask = new PathChainTask(generatedPathChain, 0.5)
                .addWaitAction(0.2, motorActions.depositSpecimen());
        tasks.add(mainTask);
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
        // Start the path using the generated path chain.
        follower.followPath((PathChain) task.pathChain, false);
    }

    // -------- Standard OpMode Lifecycle Methods --------
    @Override
    public void init() {
        // Initialize timers.
        opModeTimer = new Timer();
        pathTimer.resetTimer();

        // Initialize hardware.
        motorControl = new MotorControl(hardwareMap);
        motorActions = new MotorActions(motorControl);
        Constants.setConstants(FConstants.class, LConstants.class);

        // Initialize the follower and set its starting pose.
        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);

        // Run any starting actions.
        run(motorActions.outTakeClaw.Close());

        // Build the path chain(s) and task list.
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

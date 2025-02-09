package opmodes;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import helpers.data.Enums;
import helpers.hardware.MotorControl;
import helpers.hardware.actions.ActionOpMode;
import helpers.hardware.actions.MotorActions;
import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

import java.util.ArrayList;
import java.util.List;

@Autonomous(name = "Park & Limelight Adjustment Test")
public class ParkDynamicAlignmentTest extends ActionOpMode {

    // --- WaitAction & WaitCondition definitions ---
    private static class WaitAction {
        Double triggerTime; // seconds; may be null
        WaitCondition condition; // may be null
        Action action;
        boolean triggered;
        WaitAction(double triggerTime, Action action) {
            this.triggerTime = triggerTime;
            this.action = action;
            this.condition = null;
            this.triggered = false;
        }
        WaitAction(WaitCondition condition, Action action) {
            this.triggerTime = null;
            this.condition = condition;
            this.action = action;
            this.triggered = false;
        }
        WaitAction(double triggerTime, WaitCondition condition, Action action) {
            this.triggerTime = triggerTime;
            this.condition = condition;
            this.action = action;
            this.triggered = false;
        }
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

    // --- PathChainTask definition ---
    private static class PathChainTask {
        PathChain pathChain;
        double waitTime; // max wait time
        List<WaitAction> waitActions = new ArrayList<>();
        WaitCondition waitCondition; // optional overall condition

        PathChainTask(PathChain pathChain, double waitTime) {
            this.pathChain = pathChain;
            this.waitTime = waitTime;
        }
        PathChainTask addWaitAction(double triggerTime, Action action) {
            waitActions.add(new WaitAction(triggerTime, action));
            return this;
        }
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

    // --- Robot & Timing ---
    private Follower follower;
    private Timer pathTimer, opModeTimer;
    private MotorActions motorActions;
    private MotorControl motorControl;
    private static final double PATH_COMPLETION_T = 0.985;
    private MotorControl.Limelight limelight;

    // --- Poses ---
    private final Pose startPose = new Pose(63 , 110, Math.toRadians(270));
    private final Pose parkPose = new Pose(63, 96, Math.toRadians(270));

    // --- PathChains ---
    private PathChain parkChain;

    // --- Task list ---
    private final List<PathChainTask> tasks = new ArrayList<>();
    private int currentTaskIndex = 0;
    private int taskPhase = 0;


    // --- Build PathChains ---
    private void buildPathChains() {
        // Create a park path from startPose -> parkPose.
        parkChain = follower.pathBuilder()
                .addPath(new BezierLine(new Point(startPose), new Point(parkPose)))
                .setLinearHeadingInterpolation(startPose.getHeading(), parkPose.getHeading())
                .setZeroPowerAccelerationMultiplier(4)
                .build();
    }

    // --- Create dynamic alignment path using limelight data ---
    private PathChain followDynamicAlignmentPath() {
        Pose currentPose = follower.getPose();
        Vector2d limelightAvg = limelight.getAverage();
        double rawOffset = limelightAvg.x;  // horizontal offset from limelight

        double defaultOffset = 0.0;
        if (rawOffset == 99.99) {
            telemetry.addData("Dynamic Align", "No valid limelight data; using fallback offset");
            rawOffset = defaultOffset;
        }

        Pose targetPose = new Pose(
                currentPose.getX() + rawOffset * 1.2,
                currentPose.getY(),
                currentPose.getHeading()
        );

        // Clamp the x coordinate between 60 and 85.
        double clampedX = Math.min(85, Math.max(60, targetPose.getX()));
        targetPose = new Pose(clampedX, currentPose.getY(), currentPose.getHeading());

        telemetry.addData("Dynamic Align: Raw Offset", rawOffset);
        telemetry.addData("Dynamic Align: Target Pose", targetPose);
        telemetry.update();

        return follower.pathBuilder()
                .addPath(new BezierLine(new Point(currentPose), new Point(targetPose)))
                .setConstantHeadingInterpolation(targetPose.getHeading())
                .addParametricCallback(0, () -> run(
                        new SequentialAction(
                                motorActions.intakeArm.Grab(),
                                motorActions.intakePivot.Grab(),
                                motorActions.spin.eatUntil(Enums.DetectedColor.RED, motorControl)
                        )))
                .setZeroPowerAccelerationMultiplier(4)
                .build();
    }

    // --- Build Task List (Park, Dynamic Alignment, then Oscillation) ---
    private void buildTaskList() {
        tasks.clear();

        // --- Park Task ---
        PathChainTask parkTask = new PathChainTask(parkChain, 1)
                .addWaitAction(0, telemetryPacket -> {
                    limelight.startCollectingSamples();
                    return false;
                })
                .addWaitAction(0.1, telemetryPacket -> limelight.collectSamples())
                .setWaitCondition(() -> limelight.collectSamples());
        tasks.add(parkTask);

        // --- Dynamic Alignment Task ---
        // This task sets its pathChain to null so that it’s computed during the wait phase.
        PathChainTask dynamicTask = new PathChainTask(null, 1.5)
                .addWaitAction(0.3, motorActions.extendo.setTargetPosition(50))
                .addWaitAction(0.6, motorActions.extendo.setTargetPosition(200))
                .addWaitAction(0.9, motorActions.extendo.setTargetPosition(450))
                .setWaitCondition(() -> motorControl.getDetectedColor() != Enums.DetectedColor.UNKNOWN);
        tasks.add(dynamicTask);

        PathChainTask oscillationTask = new PathChainTask(null, 3)
                .addWaitAction(0.3, telemetryPacket -> {
                    if (motorControl.getDetectedColor() == Enums.DetectedColor.UNKNOWN) {
                        // Cancel any active path so turning commands can take over.
                        follower.breakFollowing();
                        run(new SequentialAction(
                                // Turn 10 degrees right.
                                t -> { follower.turnToDegrees(10); return false; },
                                new SleepAction(0.5),
                                // Turn 10 degrees left.
                                t -> { follower.turnToDegrees(-10); return false; },
                                new SleepAction(0.5),
                                // Turn 10 degrees right.
                                t -> { follower.turnToDegrees(10); return false; },
                                new SleepAction(0.5),
                                // Turn 10 degrees left.
                                t -> { follower.turnToDegrees(-10); return false; },
                                new SleepAction(0.5),
                                // Final turn 10 degrees right.
                                t -> { follower.turnToDegrees(10); return false; },
                                new SleepAction(0.5)
                        ));
                    }
                    return false;
                })
                // End this task when a valid (non-UNKNOWN) color is finally detected.
                .setWaitCondition(() -> motorControl.getDetectedColor() != Enums.DetectedColor.UNKNOWN);
        tasks.add(oscillationTask);

    }

    // --- Task Runner ---
    private void runTasks() {
        if (currentTaskIndex >= tasks.size()) {
            return;
        }
        PathChainTask currentTask = tasks.get(currentTaskIndex);

        // If the current task's pathChain is null, assume this task has no driving component.
        // For such tasks, simply wait until the wait time has elapsed.
        if (currentTask.pathChain == null) {
            double waitElapsed = pathTimer.getElapsedTimeSeconds();
            if (waitElapsed >= currentTask.waitTime) {
                currentTaskIndex++;
                taskPhase = 0;
                pathTimer.resetTimer();
            }
            // Also run any wait actions
            for (WaitAction wa : currentTask.waitActions) {
                if (!wa.triggered && wa.shouldTrigger(waitElapsed)) {
                    run(wa.action);
                    wa.triggered = true;
                }
            }
            return;
        }

        // For tasks that include a path (like park and dynamic alignment)
        switch (taskPhase) {
            case 0: // DRIVING phase.
                if (!follower.isBusy()) {
                    follower.followPath(currentTask.pathChain, true);
                    pathTimer.resetTimer();
                    currentTask.resetWaitActions();
                }
                double tValue = follower.getCurrentTValue();
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
                    pathTimer.resetTimer();
                }
                break;
        }
    }

    // --- Standard OpMode methods ---
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

        // For example, close the outtake claw.
        run(motorActions.intakeTransfer());

        buildPathChains();
        buildTaskList();
    }

    @Override
    public void start() {
        opModeTimer.resetTimer();
        currentTaskIndex = 0;
        taskPhase = 0;
        pathTimer.resetTimer();
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
        telemetry.addData("LL", limelight.getAverage());
        telemetry.addData("Intake", motorControl.getDetectedColor());
        telemetry.update();
    }
}

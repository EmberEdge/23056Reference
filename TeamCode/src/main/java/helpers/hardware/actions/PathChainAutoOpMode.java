package helpers.hardware.actions;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

import java.util.ArrayList;
import java.util.List;

/**
 * An abstract autonomous OpMode that supports a series of PathChainTasks.
 *
 * Subclasses should override buildPathChains() and buildTaskList() to define:
 *  - The geometry of your paths (via buildPathChains)
 *  - The sequence of tasks with any wait actions/conditions (via buildTaskList)
 *
 * This version also provides overloads for adding wait actions that trigger by time, by condition, or by both.
 */
public abstract class PathChainAutoOpMode extends ActionOpMode {

    // Dashboard instance (if desired)
    protected final FtcDashboard dashboard = FtcDashboard.getInstance();

    // ---------- Task Management Fields ----------
    protected Timer pathTimer = new Timer();
    protected List<PathChainTask> tasks = new ArrayList<>();
    protected int currentTaskIndex = 0;
    // taskPhase: 0 = DRIVING phase, 1 = WAITING phase.
    protected int taskPhase = 0;
    // Parameter value when you consider the path complete (adjust as needed)
    protected double PATH_COMPLETION_T = 0.985;

    // ---------- Abstract Methods ----------
    /**
     * Override this method to build your path geometries.
     */
    protected abstract void buildPathChains();

    /**
     * Override this method to build your sequence of PathChainTasks.
     */
    protected abstract void buildTaskList();

    // ---------- OpMode Lifecycle Methods ----------
    @Override
    public void init() {
        super.init();
        // Subclasses are expected to call buildPathChains() and buildTaskList() later in their init.
    }

    @Override
    public void loop() {
        super.loop();
        runTasks();
    }

    // ---------- Task Runner Logic ----------
    /**
     * runTasks() is called in the loop to drive the current PathChainTask through its DRIVING and WAITING phases.
     */
    protected void runTasks() {
        if (currentTaskIndex >= tasks.size()) {
            return; // All tasks completed.
        }

        PathChainTask currentTask = tasks.get(currentTaskIndex);

        switch (taskPhase) {
            case 0: // DRIVING phase.
                // Subclasses should implement your path following logic.
                // For example, check if the follower is active; if not, start the path.
                if (!isPathActive()) {
                    startPath(currentTask);
                    pathTimer.resetTimer();
                    currentTask.resetWaitActions();
                }
                // Here, we check the progress of the path (e.g., using your follower's progress value).
                double tValue = getCurrentTValue();
                if (tValue >= PATH_COMPLETION_T) {
                    pathTimer.resetTimer();
                    taskPhase = 1;
                }
                break;

            case 1: // WAITING phase.
                double waitElapsed = pathTimer.getElapsedTimeSeconds();

                // Process individual wait actions.
                for (WaitAction wa : currentTask.waitActions) {
                    if (!wa.triggered && wa.shouldTrigger(waitElapsed)) {
                        run(wa.action);
                        wa.triggered = true;
                    }
                }

                // Process overall wait condition and timeout logic.
                if (currentTask.waitCondition != null) {
                    // If the condition hasn't been met yet:
                    if (currentTask.conditionMetTime == null) {
                        if (currentTask.waitCondition.isMet()) {
                            // Record when the condition was met.
                            currentTask.conditionMetTime = waitElapsed;
                        } else if (waitElapsed >= currentTask.maxWaitTime) {
                            // Condition never met within the maximum wait time; move on.
                            currentTaskIndex++;
                            taskPhase = 0;
                        }
                    } else {
                        // Condition has been met; wait for additional waitTime.
                        if (waitElapsed - currentTask.conditionMetTime >= currentTask.waitTime) {
                            currentTaskIndex++;
                            taskPhase = 0;
                        }
                    }
                } else {
                    // No overall wait condition provided; just use the waitTime.
                    if (waitElapsed >= currentTask.waitTime) {
                        currentTaskIndex++;
                        taskPhase = 0;
                    }
                }
                break;
        }
    }

    /**
     * Dummy method to indicate whether the path is active.
     * Subclasses should override this to integrate with your path follower.
     */
    protected boolean isPathActive() {
        return false;
    }

    /**
     * Dummy method to return the current progress along the path.
     * Subclasses should override this with actual progress logic.
     */
    protected double getCurrentTValue() {
        return 0.0;
    }

    /**
     * Dummy method to start a path.
     * Subclasses should override this to command the follower to begin following the path.
     */
    protected void startPath(PathChainTask task) {
    }

    // ---------- Helper Inner Classes ----------

    /**
     * WaitAction allows you to trigger an Action during the WAIT phase either after a given time,
     * when a given condition is met, or when either occurs.
     */
    public static class WaitAction {
        // Use boxed Double so it can be null if not set.
        Double triggerTime; // in seconds; may be null
        WaitCondition condition; // may be null
        Action action;
        boolean triggered;

        // Constructor: time-only trigger.
        public WaitAction(double triggerTime, Action action) {
            this.triggerTime = triggerTime;
            this.action = action;
            this.condition = null;
            this.triggered = false;
        }

        // Constructor: condition-only trigger.
        public WaitAction(WaitCondition condition, Action action) {
            this.triggerTime = null;
            this.condition = condition;
            this.action = action;
            this.triggered = false;
        }

        // Constructor: both a time and condition trigger.
        public WaitAction(double triggerTime, WaitCondition condition, Action action) {
            this.triggerTime = triggerTime;
            this.condition = condition;
            this.action = action;
            this.triggered = false;
        }

        /**
         * Determines whether this wait action should trigger based on the elapsed time.
         */
        public boolean shouldTrigger(double elapsedSeconds) {
            boolean timeMet = (triggerTime != null && elapsedSeconds >= triggerTime);
            boolean conditionMet = (condition != null && condition.isMet());
            return timeMet || conditionMet;
        }
    }

    /**
     * Functional interface for a wait condition.
     */
    @FunctionalInterface
    public interface WaitCondition {
        boolean isMet();
    }

    /**
     * PathChainTask represents one task in your autonomous routine.
     *
     * The waitTime here represents the additional delay after the overall wait condition is met.
     * If a waitCondition is provided, the waiting phase will start its countdown only after the condition is met.
     * A maximum wait time (maxWaitTime) can also be set to serve as a timeout.
     */
    public static class PathChainTask {
        // Replace Object with your actual PathChain type if desired.
        public Object pathChain;
        public double waitTime; // Wait time after condition is met (or overall if no condition)
        public double maxWaitTime = Double.MAX_VALUE; // Maximum overall wait time (timeout) when using a condition
        public List<WaitAction> waitActions = new ArrayList<>();
        public WaitCondition waitCondition; // Optional overall condition.
        public Double conditionMetTime = null; // Recorded time when the overall condition was met.

        public PathChainTask(Object pathChain, double waitTime) {
            this.pathChain = pathChain;
            this.waitTime = waitTime;
        }

        /**
         * Add a wait action that triggers after a given time.
         */
        public PathChainTask addWaitAction(double triggerTime, Action action) {
            waitActions.add(new WaitAction(triggerTime, action));
            return this;
        }

        /**
         * Add a wait action that triggers when a given condition is met.
         */
        public PathChainTask addWaitAction(WaitCondition condition, Action action) {
            waitActions.add(new WaitAction(condition, action));
            return this;
        }

        /**
         * Add a wait action that triggers when either the specified time has elapsed or the condition is met.
         */
        public PathChainTask addWaitAction(double triggerTime, WaitCondition condition, Action action) {
            waitActions.add(new WaitAction(triggerTime, condition, action));
            return this;
        }

        /**
         * Set an overall wait condition for this task.
         */
        public PathChainTask setWaitCondition(WaitCondition condition) {
            this.waitCondition = condition;
            return this;
        }

        /**
         * Set a maximum wait time (timeout) for this task when using an overall condition.
         */
        public PathChainTask setMaxWaitTime(double maxWaitTime) {
            this.maxWaitTime = maxWaitTime;
            return this;
        }

        /**
         * Resets all wait actions and clears the condition-met timer.
         */
        public void resetWaitActions() {
            for (WaitAction wa : waitActions) {
                wa.triggered = false;
            }
            conditionMetTime = null;
        }
    }

    /**
     * A simple Timer helper class.
     */
    public static class Timer {
        private long startTime;

        public Timer() {
            resetTimer();
        }

        public void resetTimer() {
            startTime = System.currentTimeMillis();
        }

        /**
         * Returns the elapsed time in seconds.
         */
        public double getElapsedTimeSeconds() {
            return (System.currentTimeMillis() - startTime) / 1000.0;
        }
    }
}

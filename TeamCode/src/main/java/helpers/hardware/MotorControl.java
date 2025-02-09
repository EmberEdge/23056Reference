package helpers.hardware;

import android.graphics.Color;

import androidx.annotation.NonNull;

import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

import java.util.ArrayList;
import java.util.List;

import dev.frozenmilk.dairy.cachinghardware.CachingDcMotorEx;
import dev.frozenmilk.dairy.cachinghardware.CachingServo;
import helpers.data.Enums.DetectedColor;

public class MotorControl {

    private static PIDController liftController, extendoController;
    public final Servo outtakePivot, turret, armRight, intakePivot, outtakeRotation, outtakeLinkage,outtakeClaw;
    public final CRServo hangr, hangl;

    public final ColorSensor colorSensor;
    public final DcMotorEx spin;


    public final ControlledServo armLeft;
    public final Lift lift;
    public final Extendo extendo;

    public MotorControl(@NonNull HardwareMap hardwareMap) {
        lift = new Lift(hardwareMap);
        extendo = new Extendo(hardwareMap);
        armLeft = new ControlledServo(hardwareMap, "armleft", "intakesensor", 0.03, 0.1, 0.905);
        outtakePivot = hardwareMap.get(Servo.class, "outtakepivot");
        turret = hardwareMap.get(Servo.class, "outtaketurret");
        outtakeRotation = hardwareMap.get(Servo.class, "outtakerotation");
        hangr = hardwareMap.get(CRServo.class,"hangr");
        hangl = hardwareMap.get(CRServo.class,"hangl");
        armRight = hardwareMap.get(Servo.class, "armright");
        intakePivot = hardwareMap.get(Servo.class, "intakepivot");
        outtakeLinkage = hardwareMap.get(Servo.class, "outtakelinkage");
        outtakeClaw = hardwareMap.get(Servo.class, "outtakeclaw");
        colorSensor = hardwareMap.get(ColorSensor.class, "color");
        colorSensor.enableLed(false);
        spin = hardwareMap.get(DcMotorEx.class, "spin");
        armLeft.servo.setDirection(Servo.Direction.REVERSE);
        outtakePivot .setDirection(Servo.Direction.REVERSE);
    }


    public void update() {
        lift.update();
        extendo.update();
    }

    public DetectedColor getDetectedColor() {
        float red = colorSensor.red();
        float green = colorSensor.green();
        float blue = colorSensor.blue();

        // Normalize RGB values to 0–255
        float maxRawValue = Math.max(red, Math.max(green, blue));
        float scale = 255 / maxRawValue;

        red *= scale;
        green *= scale;
        blue *= scale;

        if (red > 255) red = 255;
        if (green > 255) green = 255;
        if (blue > 255) blue = 255;

        float[] hsv = new float[3];
        Color.RGBToHSV((int) red, (int) green, (int) blue, hsv);

        if (red > 100 && red > green && red > blue) {
            return DetectedColor.RED;
        } else if (blue > 100 && blue > red && blue > green) {
            return DetectedColor.BLUE;
        } else if (red > 100 && green > 100 && blue < 100) {
            return DetectedColor.YELLOW;
        } else {
            return DetectedColor.UNKNOWN;
        }
    }







    public static class Extendo extends ControlledDevice {

        private static double p = 0.005, i = 0, d = 0.0002;
        /**
         * This initializes the slide motor. This should be run before any other methods.
         *
         * @param hardwareMap The hardware map to use to get the motors.
         */
        public Extendo(HardwareMap hardwareMap) {
            extendoController = new PIDController(p,i,d);
            extendoController.setPID(p,i,d);
            motor = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class, "extendo"), 0.05);
            motor.setDirection(DcMotorSimple.Direction.REVERSE);
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        /**
         * This stops the slide, sets the state to down, sets the target to 0, and resets the encoder.
         */
        public void reset() {
            motor.setPower(0);
            targetPosition = 0;
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        public void findZero() {
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motor.setPower(-0.5);
            resetting = true;
        }

        /**
         * This updates the slide motor to match the current state. This should be run in a loop.
         */
        public void update() {
            if (resetting){
                if (motor.getCurrent(CurrentUnit.AMPS) > 1.5) {
                    reset();
                    resetting = false;
                }
            }
            else {
                extendoController.setPID(p, i, d);
                int pos = motor.getCurrentPosition();
                double pid = extendoController.calculate(pos, targetPosition);

                motor.setPower(pid);
            }
        }



        /**
         * Checks if the motor is close enough to the target position.
         *
         * @return boolean indicating whether the current position is close to the target.
         */
        public boolean closeEnough() {
            // You might want to check both motors if they need to be in sync
            return Math.abs(motor.getCurrentPosition() - targetPosition) < 20 ;
        }
    }

    public static class Lift extends ControlledDevice {
        public CachingDcMotorEx motor2;

        // PID coefficients
        public static double p = 0.013, i = 0, d = 0.0002;

        public static double GRAVITY_FEEDFORWARD = 0.15;

        /**
         * This initializes the lift motors. This should be run before any other methods.
         *
         * @param hardwareMap The hardware map to use to get the motors.
         */
        public Lift(HardwareMap hardwareMap) {
            liftController = new PIDController(p, i, d);
            liftController.setPID(p, i, d);
            motor = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class, "liftr"), 0.005);
            motor2 = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class, "liftl"), 0.005);
            motor.setDirection(DcMotorSimple.Direction.REVERSE);
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        /**
         * This stops the lift, sets the state to down, sets the target to 0, and resets the encoder.
         */
        public void reset() {
            motor.setPower(0);
            motor2.setPower(0);
            targetPosition = 0;
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        public void findZero() {
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motor.setPower(-0.6);
            motor2.setPower(-0.6);
            resetting = true;
        }

        /**
         * This updates the lift motors to match the current state. This should be run in a loop.
         */
        public void update() {
            if (resetting) {
                if (motor.getCurrent(CurrentUnit.AMPS) > 2.4) {
                    reset();
                    resetting = false;
                }
            } else {
                liftController.setPID(p, i, d);
                int pos = motor.getCurrentPosition();

                double pid = liftController.calculate(pos, targetPosition);

                double motorPower = pid + GRAVITY_FEEDFORWARD;

                motorPower = Math.max(-1, Math.min(1, motorPower));

                motor.setPower(motorPower);
                motor2.setPower(motorPower);
            }
        }

        /**
         * Checks if the motor is close enough to the target position.
         *
         * @return boolean indicating whether the current position is close to the target.
         */
        public boolean closeEnough() {
            return Math.abs(motor.getCurrentPosition() - targetPosition) < 10;
        }
    }


    public abstract static class ControlledDevice {

        public CachingDcMotorEx motor;
        public boolean resetting = false;
        double targetPosition;
        public double getTargetPosition() {
            return targetPosition;
        }
        public void setTargetPosition(double targetPosition) {
            this.targetPosition = targetPosition;
        }
        public boolean isResetting() {
            return resetting;
        }


        public abstract void update();
        public abstract void reset();
        public abstract boolean closeEnough();
        public boolean isOverCurrent() {
            return motor.isOverCurrent();
        }
    }


    public static class ControlledServo {
        public CachingServo servo;
        public AnalogInput positionSensor;
        private double targetPosition;
        private final double TOLERANCE, MIN_ANALOG, MAX_ANALOG;

        public ControlledServo(HardwareMap hardwareMap, String servoName, String analogInputName, double tolerance, double min, double max) {
            TOLERANCE = tolerance;
            MIN_ANALOG = min;
            MAX_ANALOG = max;
            servo = new CachingServo(hardwareMap.get(Servo.class, servoName));
            positionSensor = hardwareMap.get(AnalogInput.class, analogInputName);
        }

        public void setTargetPosition(double position) {
            targetPosition = position;
            servo.setPosition(position);
        }

        public double getCurrentPosition() {
            double voltage = positionSensor.getVoltage() / 3.3;
            return Math.abs((voltage - MIN_ANALOG) / (MAX_ANALOG - MIN_ANALOG));
        }


        public boolean closeEnough() {
            return Math.abs(getCurrentPosition() - targetPosition) < TOLERANCE;
        }

        public double getTargetPosition() {
            return targetPosition;
        }
    }


    public static class Limelight {
        private final Limelight3A limelight;
        private final int maxSamples = 1;
        private final List<Double> horizontal;
        private final List<Double> foward;
        private boolean isCollectingSamples;
        private final Telemetry telemetry;

        public Limelight(HardwareMap hardwareMap, Telemetry telemetry) {
            limelight = hardwareMap.get(Limelight3A.class, "limelight");
            horizontal = new ArrayList<>();
            foward = new ArrayList<>();
            isCollectingSamples = false;
            this.telemetry = telemetry;

            this.telemetry.addData("limelight", "Initialized");
            limelight.start(); // Start polling data
        }

        public void stop() {
            limelight.stop();
        }

        public void resetSamples() {
            horizontal.clear();
            foward.clear();
            isCollectingSamples = false;
        }

        public boolean isCollectingSamples() {
            return isCollectingSamples;
        }

        public void startCollectingSamples() {
            resetSamples();
            isCollectingSamples = true;
        }

        /**
         * Collects samples from the Limelight.
         *
         * @return true if the required number of samples have been collected, false otherwise.
         */
        public boolean collectSamples() {
            telemetry.addData("Collecting Samples", isCollectingSamples);
            LLResult result = limelight.getLatestResult();
            telemetry.addData("Result Exists", result != null);
            telemetry.addData("Python Output Exists", result != null && result.getPythonOutput() != null);

            if (!isCollectingSamples) {
                return false;
            }

            if (result != null && result.getPythonOutput() != null && result.getPythonOutput().length >= 2) {
                double forwardDistance = result.getPythonOutput()[0]; // Ground distance (in inches)
                double horizontalOffset = result.getPythonOutput()[1]; // Horizontal offset (in inches)
                double check = result.getPythonOutput()[3];

                if (check == 1) { // Validate the output
                    horizontal.add(horizontalOffset);
                    foward.add(forwardDistance);

                    // Maintain the size of the sample lists
                    if (horizontal.size() > maxSamples) {
                        horizontal.remove(0);
                    }
                    if (foward.size() > maxSamples) {
                        foward.remove(0);
                    }

                    // Check if enough samples have been collected
                    boolean enoughSamples = horizontal.size() == maxSamples && foward.size() == maxSamples;
                    telemetry.addData("Enough Samples", enoughSamples);
                    return enoughSamples;
                }
            }
            return false;
        }

        /**
         * Computes the average of the collected samples.
         *
         * @return a Vector3d object containing the average x, y, and z in inches.
         */
        public Vector2d getAverage() {
            if (horizontal.isEmpty() || foward.isEmpty()) {
                telemetry.addData("Average Pose", "No samples collected.");
                telemetry.update();
                return new Vector2d(99.99, 99.99); // Handle the case with no samples
            }

            // Compute averages for X and Y
            double hor = horizontal.stream().mapToDouble(Double::doubleValue).average().orElse(0);
            double ver = foward.stream().mapToDouble(Double::doubleValue).average().orElse(0);

            telemetry.addData("Average X", hor);
            telemetry.addData("Average Y", ver);
            telemetry.update();

            return new Vector2d(hor, ver);
        }
    }


}
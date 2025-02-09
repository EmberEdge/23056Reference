package helpers.hardware.actions;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;

import helpers.data.Enums;
import helpers.data.Enums.DetectedColor;
import helpers.hardware.MotorControl;


public class MotorActions {
    public final MotorControl motorControl;
    public Enums.Intake intakePosition = Enums.Intake.Transfer;
    public Enums.OutTake outtakePosition = Enums.OutTake.Transfer;

    public final Extendo extendo;
    public final Lift lift;
    public final Spin spin;
    public final IntakeArm intakeArm;
    public final IntakePivot intakePivot;
    public final OuttakePivot outtakePivot;
    public final OutTakeClaw outTakeClaw;
    public final OuttakeLinkage outTakeLinkage;
    public final OuttakeArm outtakeArm;
    public final OuttakeTurret outtakeTurret;
    public final Hang hang;

    public MotorActions(MotorControl motorControl) {
        this.motorControl = motorControl;
        this.extendo = new Extendo();
        this.lift = new Lift();
        this.intakeArm = new IntakeArm();
        this.intakePivot = new IntakePivot();
        this.outtakePivot = new OuttakePivot();
        this.outTakeClaw = new OutTakeClaw();
        this.spin = new Spin(motorControl);
        this.outTakeLinkage = new OuttakeLinkage();
        this.outtakeArm = new OuttakeArm();
        this.outtakeTurret = new OuttakeTurret();
        this.hang = new Hang();
    }



    public Action outtakeTransfer(){
        return new SequentialAction(
                t -> {
                    outtakePosition = Enums.OutTake.Transfer;
                    return false;
                },
                outTakeClaw.Open(),
                new SleepAction(0.2),
                outTakeLinkage.Transfer(),
                outtakeArm.Transfer(),
                outtakePivot.Transfer(),
                outtakeTurret.down(),
                lift.transfer(),
                lift.waitUntilFinished()
        );
    }



    public Action intakeExtend(double Position) {
        return new SequentialAction(
                t -> {
                    intakePosition = Enums.Intake.Extended;
                    return false;
                },
                intakeArm.Extended(),
                intakePivot.Grab(),
                extendo.setTargetPosition(Position),
                extendo.waitUntilFinished());
    }

    public Action intakeGrabUntil(Enums.DetectedColor allianceColor) {
        return new SequentialAction(
                t -> {
                    intakePosition = Enums.Intake.Spin;
                    return false;
                },
                lift.setTargetPosition(30),
                intakeArm.Grab(),
                intakePivot.Grab(),
                outtakeTransfer(),
                spin.eatUntil(allianceColor, motorControl),
                intakeTransfer()

        );
    }





    public Action intakeTransfer() {
        return new SequentialAction(
                t -> {
                    intakePosition = Enums.Intake.Transfer;
                    return false;
                },
                spin.stop(),
                outtakeTurret.down(),
                intakeArm.Extended(),
                outTakeClaw.PartialOpen(),
                intakePivot.Transfer(),
                extendo.retracted(),
                extendo.waitUntilFinished(),
                outTakeClaw.Open(),
                spin.slow()
        );
    }

    public Action outtakeSample() {
        return new SequentialAction(
                t -> {
                    outtakePosition = Enums.OutTake.Deposit;
                    return false;
                },
                spin.stop(),

                intakeArm.Transfer(),
                new SleepAction(0.25),
                outTakeClaw.Close(),
                outtakePivot.TRANSFER2(),
                new SleepAction(0.2),
                intakeArm.Extended(),
                outTakeLinkage.sample(),
                outtakeArm.sample(),
                outtakePivot.DepositSample(),
                lift.setTargetPosition(780),
                new SleepAction(0.2),
                outtakeTurret.up(),
                lift.waitUntilFinished()
        );
    }



    public Action intakeSpecimen(){
        return new SequentialAction(
                t -> {
                    outtakePosition = Enums.OutTake.wall;
                    return false;
                },
                lift.transfer(),
                outtakeTurret.up(),
                outTakeLinkage.wall(),
                outtakeArm.wall(),
                outtakePivot.wall(),
                outTakeClaw.Open()
        );
    }


    public Action outtakeSpecimen() {
        return new SequentialAction(
                t -> {
                    outtakePosition = Enums.OutTake.Specimen;
                    return false;
                },
                outTakeClaw.Close(),
                intakeArm.Intake(),
                new SleepAction(0.05),
                outtakeTurret.down(),
                intakeArm.Intake(),
                outTakeLinkage.Specimen(),
                outtakeArm.Specimen(),
                outtakePivot.Deposit(),
                lift.secondTruss()
        );
    }

    public Action outtakeSpecimenAuto() {
        return new SequentialAction(
                t -> {
                    outtakePosition = Enums.OutTake.Specimen;
                    return false;
                },
                outTakeClaw.Close(),
                new SleepAction(0.1),
                outtakeTurret.down(),
                intakeArm.Intake(),
                new SleepAction(0.1),
                outTakeLinkage.Specimen(),
                outtakeArm.Specimen(),
                outtakePivot.Deposit(),
                lift.setTargetPosition(370),
                lift.waitUntilFinished()
        );
    }

    public Action depositSpecimen(){
        return new SequentialAction(
                outTakeClaw.Close(),
                outTakeLinkage.Specimen(),
                outtakePivot.Deposit2(),
                lift.setTargetPosition(175),
                new SleepAction(0.1),
                lift.waitUntilFinished(),
                outTakeClaw.Open(),
                intakeSpecimen(),
                intakeTransfer()
        );
    }





    public Action update() {
        return t -> {
            motorControl.update();
            return true; // this returns true to make it loop forever; use RaceParallelCommand
        };
    }

        public class Extendo {
        public Action setTargetPosition(double position) {
            return t -> {
                motorControl.extendo.setTargetPosition(position);
                return false;
            };
        }
        public Action waitUntilFinished() {
            return new Action() {
                @Override
                public boolean run(@NonNull TelemetryPacket t) {
                    return !motorControl.extendo.closeEnough();
                }
            };
        }

        public Action findZero() {
            return new SequentialAction(t -> {motorControl.extendo.findZero();return false;},
                    new ActionHelpers.WaitUntilAction(() -> !motorControl.extendo.isResetting()));
        }



        public Action retracted() {
            return setTargetPosition(10);
        }
        public Action extended() {
            return setTargetPosition(590);
        }
    }

    public class Lift {
        public Action setTargetPosition(double position) {
            return t -> {
                motorControl.lift.setTargetPosition(position);
                return false;
            };
        }
        public Action waitUntilFinished() {
            return new Action() {
                @Override
                public boolean run(@NonNull TelemetryPacket t) {
                    return !motorControl.lift.closeEnough();
                }
            };
        }

        public Action findZero() {
            return new SequentialAction(t -> {motorControl.lift.findZero();return false;},
                    new ActionHelpers.WaitUntilAction(() -> !motorControl.lift.isResetting()));
        }



        //todo: fix positions
        public Action transfer() {
            return setTargetPosition(30);
        }
        public Action secondBuceket() {
            return setTargetPosition(670);
        }
        public Action firstTruss() {
            return setTargetPosition(100);
        }
        public Action secondTruss() {
            return setTargetPosition(460);
        }
    }


    public class IntakeArm {
        private static final double GRAB_POSITION = 0.05;
        private static final double INTAKE_POSITION = 0.2;
        private static final double EXTENDED_POSITION = 0.2;
        private static final double TRANSFER_POSITION = 0.35;

        public Action setTargetPosition(double position) {
            return new Action() {
                @Override
                public boolean run(@NonNull TelemetryPacket t) {
                    motorControl.armLeft.setTargetPosition(position);
                    motorControl.armRight.setPosition(position);
                    new SleepAction(0.1);
                    return false;
                }
            };
        }

        public Action waitUntilFinished() {
            return new Action() {
                @Override
                public boolean run(@NonNull TelemetryPacket t) {
                    return !motorControl.armLeft.closeEnough(); // Returns true when both servos are close enough
                }
            };
        }

        public Action Grab() {
            return setTargetPosition(GRAB_POSITION);
        }

        public Action Extended() {
            return setTargetPosition(EXTENDED_POSITION);
        }

        public Action Intake() {
            return setTargetPosition(INTAKE_POSITION);
        }

        public Action Transfer() {
            return setTargetPosition(TRANSFER_POSITION);
        }
    }

    public class IntakePivot {

        private static final double GRAB_POSITION = 0.66;
        private static final double EXTENDED_POSITION = 0.78;
        private static final double TRANSFER_POSITION = 0.63;

        public Action setTargetPosition(double position) {
            return t -> {
                motorControl.intakePivot.setPosition(position);
                return false;
            };
        }

        public Action Grab() {
            return setTargetPosition(GRAB_POSITION);
        }

        public Action Extend() {
            return setTargetPosition(EXTENDED_POSITION);
        }

        public Action Transfer(){
            return setTargetPosition(TRANSFER_POSITION);
        }

    }




    public class OuttakePivot {
        private static final double TRANSFER_POSITION = 0.23;
        private static final double DEPOSIT = 0.4;
        private static final double DEPOSIT2 = 0.32;
        private static final double DEPOSITSample = 0.64;
        private static final double WALL_INTAKE = 0.51;
        private static final double TRANSFER = 0.21;

        public Action setTargetPosition(double position) {
            return t -> {
                motorControl.outtakePivot.setPosition(position);
                return false;
            };
        }

        public Action Transfer() {
            return setTargetPosition(TRANSFER_POSITION);
        }

        public Action Deposit() {
            return setTargetPosition(DEPOSIT);
        }
        public Action Deposit2() {
            return setTargetPosition(DEPOSIT2);
        }

        public Action wall(){
            return setTargetPosition(WALL_INTAKE);
        }
        public Action TRANSFER2(){
            return setTargetPosition(TRANSFER);
        }

        public Action DepositSample(){
            return setTargetPosition(DEPOSITSample);
        }

    }
    public class Hang {
        public Action up() {
            return t -> {
                motorControl.hangr.setPower(-1);
                motorControl.hangl.setPower(-1);
                return false;
            };
        }

        public Action down() {
            return t -> {
                motorControl.hangr.setPower(1);
                motorControl.hangl.setPower(1);
                return false;
            };
        }

        public Action stop() {
            return t -> {
                motorControl.hangr.setPower(0);
                motorControl.hangl.setPower(0);
                return false;
            };
        }
    }

    public class OuttakeLinkage {
        private static final double TRANSFER_POSITION = 0.64;
        private static final double SPECIMEN_DEPOSIT = 0.7;
        private static final double WALL_INTAKE = 0.5;
        private static final double SAMPLE_DEPOSIT = 0.1;

        public Action setTargetPosition(double position) {
            return t -> {
                motorControl.outtakeLinkage.setPosition(position);
                return false;
            };
        }

        public Action Transfer() {
            return setTargetPosition(TRANSFER_POSITION);
        }

        public Action Specimen() {
            return setTargetPosition(SPECIMEN_DEPOSIT);
        }

        public Action wall(){
            return setTargetPosition(WALL_INTAKE);
        }
        public Action sample(){
            return setTargetPosition(SAMPLE_DEPOSIT);
        }

    }


    public class OuttakeArm {
        private static final double TRANSFER_POSITION = 0.81;
        private static final double SPECIMEN_DEPOSIT = 0.88;
        private static final double WALL_INTAKE = 0;
        private static final double SAMPLE_DEPOSIT = 0.4;

        public Action setTargetPosition(double position) {
            return t -> {
                motorControl.outtakeRotation.setPosition(position);
                return false;
            };
        }

        public Action Transfer() {
            return setTargetPosition(TRANSFER_POSITION);
        }

        public Action Specimen() {
            return setTargetPosition(SPECIMEN_DEPOSIT);
        }

        public Action wall(){
            return setTargetPosition(WALL_INTAKE);
        }
        public Action sample(){
            return setTargetPosition(SAMPLE_DEPOSIT);
        }
    }

    public class OuttakeTurret {
        private static final double UP = 0.1;
        private static final double DOWN = 0.9;

        public Action setTargetPosition(double position) {
            return t -> {
                motorControl.turret.setPosition(position);
                return false;
            };
        }

        public Action up() {
            return setTargetPosition(UP);
        }

        public Action down() {
            return setTargetPosition(DOWN);
        }
    }




    public class OutTakeClaw {
        private static final double CLOSE_POSITION = 0.02;
        private static final double OPEN_POSITION = 0.36;
        private static final double PARTIAL_CLOSE = 0.1;
        private static final double PARTIAL_OPEN = 0.25;

        public Action setTargetPosition(double position) {
            return t -> {
                motorControl.outtakeClaw.setPosition(position);
                return false;
            };
        }

        public Action Close() {
            return setTargetPosition(CLOSE_POSITION);
        }

        public Action Open() {
            return setTargetPosition(OPEN_POSITION);
        }

        public Action PartialClose() {
            return setTargetPosition(PARTIAL_CLOSE);
        }

        public Action PartialOpen() {
            return setTargetPosition(PARTIAL_OPEN);
        }
    }


    public class Spin {
        private final MotorControl motorControl;


        public Spin(MotorControl motorControl) {
            this.motorControl = motorControl;
        }


        public Action eatUntil(DetectedColor allianceColor, MotorControl motorControl) {
            final Enums.IntakeState[] currentState = {Enums.IntakeState.SEARCHING};
            final boolean[] started = {false};
            // Add a variable to store when the rejecting state was entered
            final long[] rejectingStartTime = {0};
            // Define the delay in milliseconds (adjust as needed)
            final long REJECT_DELAY_MS = 500;

            return telemetryPacket -> {
                if (!started[0]) {
                    started[0] = true;
                    motorControl.spin.setPower(1); // Start spinning forward
                }

                // Read the sensor
                DetectedColor color = motorControl.getDetectedColor();

                // If yellow is detected, task is finished
                if (color == Enums.DetectedColor.YELLOW) {
                    motorControl.spin.setPower(0); // Stop motor
                    return false; // Action complete
                }

                boolean correctColorSeen = false;
                if (allianceColor == Enums.DetectedColor.RED) {
                    // Accept RED or YELLOW
                    correctColorSeen = (color == Enums.DetectedColor.RED || color == Enums.DetectedColor.YELLOW);
                } else if (allianceColor == Enums.DetectedColor.BLUE) {
                    // Accept BLUE or YELLOW
                    correctColorSeen = (color == Enums.DetectedColor.BLUE || color == Enums.DetectedColor.YELLOW);
                }

                switch (currentState[0]) {
                    case SEARCHING:
                        if (correctColorSeen) {
                            motorControl.spin.setPower(0); // Stop motor
                            return false; // Action complete
                        }
                        // If an unexpected (rejectable) color is seen, transition to REJECTING
                        if (color != Enums.DetectedColor.BLACK
                                && color != Enums.DetectedColor.UNKNOWN
                                && !correctColorSeen) {
                            currentState[0] = Enums.IntakeState.REJECTING;
                            // Record the time when rejecting starts
                            rejectingStartTime[0] = System.currentTimeMillis();
                            motorControl.spin.setPower(-1.0); // Spin backward
                        }
                        return true; // Continue searching

                    case REJECTING:
                        long now = System.currentTimeMillis();
                        // Only check for resuming forward spinning after the delay
                        if (now - rejectingStartTime[0] >= REJECT_DELAY_MS) {
                            if (color == Enums.DetectedColor.BLACK || color == Enums.DetectedColor.UNKNOWN) {
                                // Done rejecting; resume forward spinning
                                motorControl.spin.setPower(1);
                                currentState[0] = Enums.IntakeState.SEARCHING;
                                // Reset the timer (optional cleanup)
                                rejectingStartTime[0] = 0;
                            }
                        }
                        return true; // Continue in the REJECTING state until conditions are met
                }

                // Default (should not be reached)
                return false;
            };

    }


        /**
         * Simple "spin forward" action that runs indefinitely unless you remove it.
         * You can adapt the return value to end immediately if desired.
         */
        public Action slow() {
            return telemetryPacket -> {
                motorControl.spin.setPower(0.3);
                return false;
            };
        }

        public Action eat() {
            return telemetryPacket -> {
                motorControl.spin.setPower(1);
                return false;
            };
        }

        /**
         * Simple "spin backward" action that runs indefinitely.
         */
        public Action poop() {
            return telemetryPacket -> {
                motorControl.spin.setPower(-1.0);
                return false;
            };
        }

        /**
         * Stop the spin motor (one-shot).
         */
        public Action stop() {
            return telemetryPacket -> {
                motorControl.spin.setPower(0.0);
                return false; // done immediately
            };
        }


    }






}

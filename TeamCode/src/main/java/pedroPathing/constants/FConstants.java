package pedroPathing.constants;

import com.pedropathing.localization.Localizers;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.util.CustomFilteredPIDFCoefficients;
import com.pedropathing.util.CustomPIDFCoefficients;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class FConstants {
    static {
        FollowerConstants.localizers = Localizers.PINPOINT;

        FollowerConstants.leftFrontMotorName = "FL";
        FollowerConstants.leftRearMotorName = "BL";
        FollowerConstants.rightFrontMotorName = "FR";
        FollowerConstants.rightRearMotorName = "BR";
        FollowerConstants.useBrakeModeInTeleOp = true;

        FollowerConstants.leftFrontMotorDirection = DcMotorSimple.Direction.REVERSE;
        FollowerConstants.leftRearMotorDirection = DcMotorSimple.Direction.REVERSE;
        FollowerConstants.rightFrontMotorDirection = DcMotorSimple.Direction.FORWARD;
        FollowerConstants.rightRearMotorDirection = DcMotorSimple.Direction.FORWARD;

        FollowerConstants.mass = 16.3293;

        FollowerConstants.xMovement = 73.92548340257906;
        FollowerConstants.yMovement = 53.05913269055119;

        FollowerConstants.forwardZeroPowerAcceleration = -35.19155522807619;
        FollowerConstants.lateralZeroPowerAcceleration = -79.78510474918826;

        FollowerConstants.translationalPIDFCoefficients.setCoefficients(0.25,
                0,
                0.005,
                0);
        FollowerConstants.translationalPIDFFeedForward = 0.015;

        FollowerConstants.useSecondaryTranslationalPID = true;
        FollowerConstants.secondaryTranslationalPIDFCoefficients.setCoefficients(0.17,
                0,
                0.005,0); // Not being used, @see useSecondaryTranslationalPID

        FollowerConstants.headingPIDFCoefficients.setCoefficients(3.25,0,0.2,0);

        FollowerConstants.useSecondaryHeadingPID = true;
        FollowerConstants.secondaryHeadingPIDFCoefficients.setCoefficients(2.5,0,0.03,0); // Not being used, @see useSecondaryHeadingPID

        FollowerConstants.drivePIDFCoefficients.setCoefficients(0.33 ,0,0.01,0.6,0);
        FollowerConstants.drivePIDFFeedForward = 0.02;


        FollowerConstants.useSecondaryDrivePID = true   ;
        FollowerConstants.secondaryDrivePIDFCoefficients.setCoefficients(0.32,0,0.005,0.6,0); // Not being used, @see useSecondaryDrivePID

        FollowerConstants.zeroPowerAccelerationMultiplier = 4.75;
        FollowerConstants.centripetalScaling = 0.00025;

        FollowerConstants.pathEndTimeoutConstraint = 75;
        FollowerConstants.pathEndTValueConstraint = 0.96;
        FollowerConstants.pathEndVelocityConstraint = 3.2;
        FollowerConstants.useVoltageCompensationInAuto = true;
        FollowerConstants.useVoltageCompensationInTeleOp = true;
        FollowerConstants.nominalVoltage = 13.2 ;
        FollowerConstants.pathEndTranslationalConstraint = 0.1;
        FollowerConstants.pathEndHeadingConstraint = 0.007;
    }
}

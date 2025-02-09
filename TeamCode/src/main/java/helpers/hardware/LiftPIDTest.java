package helpers.hardware;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import dev.frozenmilk.dairy.cachinghardware.CachingDcMotorEx;

@Config
@TeleOp
public class LiftPIDTest extends OpMode {

    private PIDController pidController;

    public static double p = 0.02, i = 0, d = 0.0005;

    public static int target = 0;

    private CachingDcMotorEx motor1, motor2;

    public static double GRAVITY_FEEDFORWARD = 0.15;


    @Override
    public void init() {
        pidController = new PIDController(p,i,d);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        motor1 = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class, "liftr"));
        motor2 = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class, "liftl"));

        motor1.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    @Override
    public void loop() {
        pidController.setPID(p,i,d);
        int pos = motor1.getCurrentPosition();
        double pid = pidController.calculate(pos, target) + GRAVITY_FEEDFORWARD;

        double adjustedFeedforward = pid < 0 ? GRAVITY_FEEDFORWARD : 0;

        double motorPower = pid + adjustedFeedforward;

        motorPower = Math.max(-1, Math.min(1, motorPower));

        motor1.setPower(motorPower);
        motor2.setPower(motorPower);



        telemetry.addData("pid", pid);
        telemetry.addData("pos", pos);
        telemetry.addData("target", target);
        telemetry.update();

    }
}

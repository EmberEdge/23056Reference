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
public class LiftPIDTestSquID extends OpMode {

    private SquIDController pidController;

    public static double p = 0.02, i = 0, d = 0.0005;

    public static int target = 0;

    private CachingDcMotorEx motor1, motor2;

    public static double GRAVITY_FEEDFORWARD = 0.15;


    @Override
    public void init() {
        pidController = new SquIDController(p,i,d);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        motor1 = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class, "liftr"));
        motor2 = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class, "liftl"));

        motor1.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    @Override
    public void loop() {
        pidController.setPID(p);
        int pos = motor1.getCurrentPosition();
        double pid = -pidController.calculate(target, pos) + GRAVITY_FEEDFORWARD;

        motor1.setPower(pid);
        motor2.setPower(pid);



        telemetry.addData("pid", pid);
        telemetry.addData("pos", pos);
        telemetry.addData("target", target);
        telemetry.update();

    }
}

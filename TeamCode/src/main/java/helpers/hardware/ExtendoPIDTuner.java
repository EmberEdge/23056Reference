package helpers.hardware;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import dev.frozenmilk.dairy.cachinghardware.CachingDcMotorEx;

@Config
@TeleOp
public class ExtendoPIDTuner extends OpMode {

    private PIDController pidController;

    public static double p = 0, i = 0, d = 0;

    public static int target = 0;

    private CachingDcMotorEx motor1;


    @Override
    public void init() {
        pidController = new PIDController(p,i,d);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        motor1 = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class, "extendo"), 0.05);

    }

    @Override
    public void loop() {
        pidController.setPID(p,i,d);
        int pos = motor1.getCurrentPosition();
        double pid = pidController.calculate(pos, target);

        motor1.setPower(pid);



        telemetry.addData("pid", pid);
        telemetry.addData("pos", pos);
        telemetry.addData("target", target);
        telemetry.update();

    }
}

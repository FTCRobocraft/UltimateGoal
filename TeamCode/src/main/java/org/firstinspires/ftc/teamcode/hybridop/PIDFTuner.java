package org.firstinspires.ftc.teamcode.hybridop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.hardware.UltimateGoalHardware;
import org.firstinspires.ftc.teamcode.playmaker.GamepadActions;
import org.firstinspires.ftc.teamcode.playmaker.GamepadActions.GamepadButtons;
import org.firstinspires.ftc.teamcode.playmaker.GamepadActions.GamepadType;
import org.firstinspires.ftc.teamcode.playmaker.HybridOp;

@TeleOp(name="PIDF Tuner")
public class PIDFTuner extends UltimateGoalHardware implements HybridOp {

    enum PIDFModifier {
        P,
        I,
        D
    }

    PIDFModifier currentModifier;
    final double INCREMENT = 10;
    double maxTps = 0;

    @Override
    public void autonomous_loop() {

    }

    @Override
    public void teleop_loop() {
        if (gamepadActions.isFirstPress(GamepadType.ONE, GamepadButtons.a)) {
            currentModifier = PIDFModifier.P;
        } else if (gamepadActions.isFirstPress(GamepadType.ONE, GamepadButtons.b)) {
            currentModifier = PIDFModifier.I;
        } else if (gamepadActions.isFirstPress(GamepadType.ONE, GamepadButtons.x)) {
            currentModifier = PIDFModifier.D;
        }

        this.setShooterEnabled(gamepadActions.isToggled(GamepadType.ONE, GamepadButtons.y));
        this.escalator.setPower(gamepad1.right_bumper ? 1 : 0);

        if (gamepadActions.isFirstPress(GamepadActions.GamepadType.ONE, GamepadActions.GamepadButtons.dpad_up)) {
            switch (currentModifier) {
                case P:
                    P += INCREMENT;
                    break;
                case I:
                    I += INCREMENT;
                    break;
                case D:
                    D += INCREMENT;
                    break;
            }
        } else if (gamepadActions.isFirstPress(GamepadActions.GamepadType.ONE, GamepadActions.GamepadButtons.dpad_down)) {
            switch (currentModifier) {
                case P:
                    P -= INCREMENT;
                    break;
                case I:
                    I -= INCREMENT;
                    break;
                case D:
                    D -= INCREMENT;
                    break;
            }
        }

        if (gamepadActions.isToggled(GamepadType.ONE, GamepadButtons.start)) {
            shooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        } else {
            shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        if (shooter.getMode() == DcMotor.RunMode.RUN_WITHOUT_ENCODER) {
            if (gamepadActions.isToggled(GamepadType.ONE, GamepadButtons.back)) {
                shooter.setPower(1);
            } else {
                shooter.setPower(0);
            }
        } else {
            this.shooter.setVelocity(this.spinShooter ? UltimateGoalHardware.SHOOTER_RPM : 0);
        }

        double tps = shooter.getVelocity();
        if (tps > maxTps) {
            maxTps = tps;
        }

        telemetry.addData("[PIDFTuner] Current Run Mode", shooter.getMode());
        telemetry.addData("[PIDFTuner] Current Modifier", currentModifier.toString());
        telemetry.addData("[PIDFTuner] Max TPS", maxTps);
    }

    @Override
    public void hybrid_loop() {

    }

    @Override
    public void run_loop() {

    }
}

package org.firstinspires.ftc.teamcode.hardware;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.teamcode.action.LocalizerMoveAction;
import org.firstinspires.ftc.teamcode.playmaker.Localizer;
import org.firstinspires.ftc.teamcode.playmaker.RobotHardware;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;

@Config
public abstract class UltimateGoalHardware extends RobotHardware {

    private static final String TFOD_MODEL_ASSET = "UltimateGoal.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Quad";
    private static final String LABEL_SECOND_ELEMENT = "Single";

    public static  Double ACCELERATION = null;
    public static  Position SHOOTING_POSITION_RED_NEAR_CENTER = Localizer.createPosition(DistanceUnit.INCH, -3, -12);
    public static  Position SHOOTING_POSITION_RED_ALIGNED_GOAL = Localizer.createPosition(DistanceUnit.INCH, -3, -36);
    public static double SHOOTER_HEADING_OFFSET = -10;
    public static double SHOOTER_POWER = 0.5235;
    public static double SHOOTER_RPM = 2800;
    public static double COUNTS_PER_SHOOTER_REV = 28;
    public static double SHOOTER_RPM_THRESHOLD = 100;
    public static double WOBBLE_GOAL_POWER_ZERO_THRESHOLD = 25;
    public static LocalizerMoveAction.LocalizerMoveActionParameters defaultLocalizerMoveParameters = new LocalizerMoveAction.LocalizerMoveActionParameters(
            LocalizerMoveAction.FollowPathMethod.FAST,
            0.7f,
            0.15,
            0.225);

    public static double OMNI_DRIVE_LAT_MOVEMENT_MULTIPLIER = 1.5;

    public static boolean spinShooter = false;
    public static boolean extendWobbleGoal = false;
    public static boolean useEncodersOnShooter = true;

    double currentShooterPower = 0;
    double currentShooterRPM = 0;
    int wobbleGoalHolderInitPos;

    public enum UltimateGoalStartingPosition  {
        LEFT,
        RIGHT
    }

    public enum WobbleGoalDestination {
        A,
        B,
        C
    }

    public DcMotor frontLeft;
    public DcMotor frontRight;
    public DcMotor backLeft;
    public DcMotor backRight;

    public DcMotorEx shooter;
    public DcMotor collector;
    public DcMotor escalator;

    public DcMotor wobbleGoalHolder;
    public Servo wobbleServo;

    public static double COUNTS_PER_ENCODER_REV = 8192;
    public static double WHEEL_DIAMETER_IN = 4.0;
    public static double COUNTS_PER_WOBBLE_REVOLUTION = 288;

    public static double F = 15;
    public static double P = 150;
    public static double I = 0;
    public static double D = 0;


    @Override
    public void initializeHardware() {
        frontLeft = this.initializeDevice(DcMotor.class, "frontLeft");
        frontRight = this.initializeDevice(DcMotor.class, "frontRight");
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft = this.initializeDevice(DcMotor.class, "backLeft");
        backRight = this.initializeDevice(DcMotor.class, "backRight");
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);
        shooter = this.initializeDevice(DcMotorEx.class, "shooter");
        shooter.setDirection(DcMotorSimple.Direction.REVERSE);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        collector = this.initializeDevice(DcMotor.class, "collector");
        escalator = this.initializeDevice(DcMotor.class, "escalator");
        wobbleGoalHolder = this.initializeDevice(DcMotor.class, "wobble");
        wobbleGoalHolderInitPos = wobbleGoalHolder.getCurrentPosition();
        wobbleGoalHolder.setTargetPosition(wobbleGoalHolderInitPos); // 72 = 90deg
        wobbleGoalHolder.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        wobbleServo = this.initializeDevice(Servo.class, "wobbleServo");
        wobbleServo.setPosition(0);
        revIMU = this.hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        revIMU.initialize(parameters);

//        wobbleGoalLeftClaw = this.initializeDevice(Servo.class, "leftClaw");
//        wobbleGoalLeftClaw.scaleRange(0.375, 0.55);
//        wobbleGoalRightClaw = this.initializeDevice(Servo.class, "rightClaw");
//        wobbleGoalRightClaw.scaleRange(0.8,0.95);

        this.initializeOmniDrive(frontLeft, frontRight, backLeft, backRight);
        this.omniDrive.setCountsPerInch(COUNTS_PER_ENCODER_REV/(Math.PI*WHEEL_DIAMETER_IN));
        this.omniDrive.setAcceleration(ACCELERATION);
        this.omniDrive.horizontalMovementScaleFactor = OMNI_DRIVE_LAT_MOVEMENT_MULTIPLIER;
        omniDrive.setZeroPowerMode(DcMotor.ZeroPowerBehavior.BRAKE);
        telemetry = FtcDashboard.getInstance().getTelemetry();
    }

    @Override
    public void initializeLocalizer() {
        super.initializeLocalizer();
        //this.localizer.setRobotStart(revIMU, 90);
        this.localizer.encodersXScaleFactor = 40.0/48.0; // ANTI JANK
        this.localizer.loadUltimateGoalTrackables(this,
                new Position(DistanceUnit.INCH, -0, 0, 9, 0),
                new Orientation(EXTRINSIC, YZX, DEGREES, 0, 0, 0, 0));

    }

    private double rpmToCPS(double val) {
        return (val * COUNTS_PER_SHOOTER_REV) / 60;
    }

    private double cpsToRPM(double val) {
        return (val / COUNTS_PER_SHOOTER_REV) * 60;
    }

    @Override
    public void hardware_loop() {
        super.hardware_loop();

        if (extendWobbleGoal) {
            wobbleGoalHolder.setTargetPosition(wobbleGoalHolderInitPos + (int) (160 * (COUNTS_PER_WOBBLE_REVOLUTION/360)));
        } else {
            wobbleServo.setPosition(0);
            wobbleGoalHolder.setTargetPosition(wobbleGoalHolderInitPos);
        }

        int countsFromTarget = Math.abs(wobbleGoalHolder.getTargetPosition() - wobbleGoalHolder.getCurrentPosition());
        wobbleGoalHolder.setPower(countsFromTarget <= WOBBLE_GOAL_POWER_ZERO_THRESHOLD ? 0 : 1);

        // TEMPORARY FOR TUNING

        double rpm = cpsToRPM(shooter.getVelocity());
        telemetry.addData("[UltimateGoalHardware] Shooter Power", "%.1f", currentShooterPower);
        telemetry.addData("[UltimateGoalHardware] Shooter TPS", "%.1f", shooter.getVelocity());
        telemetry.addData("[UltimateGoalHardware] Shooter RPM", "%.1f",  rpm);


        if (useEncodersOnShooter) {
            shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            telemetry.addData("[UltimateGoalHardware] PIDF", "%.1f %.1f %.1f %.1f",  P, I, D, F);
            shooter.setVelocityPIDFCoefficients(P, I, D, F);
            shooter.setVelocity(spinShooter ? rpmToCPS(SHOOTER_RPM) : 0);
        } else {
            shooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            shooter.setPower(spinShooter ? SHOOTER_POWER : 0);
        }
    }

    public void setShooterEnabled(boolean enabled) {
        this.spinShooter = enabled;
    }

    public boolean canShoot() {
        double rpm = cpsToRPM(shooter.getVelocity());
        boolean rpmReady = Math.abs(rpm - this.SHOOTER_RPM) <= SHOOTER_RPM_THRESHOLD;
        return rpmReady;
    }

    public boolean wobbleGoalIsInPosition() {
        int countsFromTarget = Math.abs(wobbleGoalHolder.getTargetPosition() - wobbleGoalHolder.getCurrentPosition());
        return countsFromTarget <= WOBBLE_GOAL_POWER_ZERO_THRESHOLD;
    }

    @Override
    public void initTfod() {
        super.initTfod();
        tfod.deactivate();
        tfod.setZoom(2, 16.0/9.0);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
        tfod.activate();
    }
}

package org.firstinspires.ftc.teamcode.playmaker;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.util.ArrayList;
import java.util.List;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;

public class Localizer {

    private long latestAcquisitionTime = 0;
    private EstimatedPosition lastEstimatedPosition;
    private EstimatedOrientation lastEstimatedOrientation;
    private EstimatedPosition estimatedPosition;
    private EstimatedOrientation estimatedOrientation;
    private Position deltaPosition;
    private Velocity estimatedVelocity;
    private AngularVelocity estimatedAngularVelocity;

    // IMU measurements
    private Orientation lastRawIMUOrientation;
    private Orientation lastIMUOrientation;
    private Double imuToWorldRotation;

    // Vuforia measurements
    private OpenGLMatrix cameraMatrix;
    private List<VuforiaTrackable> vuforiaTrackables = new ArrayList<VuforiaTrackable>();
    private VuforiaTransform lastVuforiaTransform;

    // Encoder measurements
    private Integer lastFrontLeft;
    private Integer lastFrontRight;
    private Integer lastBackLeft;
    private Integer lastBackRight;
    private Position lastEncoderPosition = new Position();

    // Vuforia Constants
    private static final float mmPerInch        = 25.4f;
    private static final float mmTargetHeight   = (6) * mmPerInch;          // the height of the center of the target image above the floor
    private static final float halfField = 72 * mmPerInch;
    private static final float quadField  = 36 * mmPerInch;
    private boolean allowVuforiaMeasurement = false;

    public double distanceToAcceptVuforiaInches = 24;
    public double encodersXScaleFactor = 1;
    public double encodersYScaleFactor = 1;

    public boolean vuforiaOverwritesOtherLocations = true;
    public boolean vuforiaOverwritesOtherOrientations = true;
    Telemetry telemetry;

    public void setLatestAcquisitionTime() {
        this.latestAcquisitionTime = System.currentTimeMillis();
    }

    enum PositionSource {
        VUFORIA,
        ENCODERS,
        OTHER
    }

    enum OrientationSource {
        VUFORIA,
        IMU,
        OTHER
    }

    public class EstimatedPosition {
        public PositionSource source;
        public Position position;
        public EstimatedPosition(PositionSource source, Position position) {
            this.source = source;
            this.position = position;
        }
    }

    public class EstimatedOrientation {
        public OrientationSource source;
        public Orientation orientation;
        public EstimatedOrientation(OrientationSource source, Orientation orientation) {
            this.source = source;
            this.orientation = orientation;
        }
    }

    static public class RobotTransform {
        public Position position;
        public double heading;

        public RobotTransform(Position position, double heading) {
            this.position = position;
            this.heading = heading;
        }

        public RobotTransform(DistanceUnit unit, double x, double y, double heading) {
            this.position = new Position(unit, x, y, 0, 0);
            this.heading = heading;
        }

        public RobotTransform copy() {
            return new RobotTransform(new Position(this.position.unit, this.position.x, this.position.y, this.position.z, this.position.acquisitionTime), this.heading);
        }
    }

    public Localizer(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    /**
     * Return the robot's estimated location on the field.
     * X: the axis running from the audience view (negative) to the goals (positive)
     * Y: the axis running from red alliance (negative) to the blue alliance (positive)
     * Z: height of the robot off ground, though not relevant for this game.
     * @return The robot's estimated position
     */

    public EstimatedPosition getEstimatedPosition() {
        return estimatedPosition;
    }

    /**
     * Estimate the robot's orientation on the field.
     * First Angle: Roll
     * Second Angle: Pitch
     * Third Angle: Heading, this is the only value you'll really care about
     * @return The robot's estimated orientation
     */

    public EstimatedOrientation getEstimatedOrientation() {
        return estimatedOrientation;
    } //meeep

    public Velocity estimateVelocity() {
        return estimatedVelocity;
    }

    public AngularVelocity estimateAngularVelocity() {
        return estimatedAngularVelocity;
    }

    public Position getDeltaPosition(DistanceUnit unit) {
        if (deltaPosition != null) {
            return deltaPosition.toUnit(unit);
        }
        return null;
    }

    public Double getDeltaDistance(DistanceUnit unit) {
        if (deltaPosition != null) {
            return Localizer.distance(zero, deltaPosition, unit);
        }
        return null;
    }

    public Long getTimeBetweenEstimatedPositions() {
        if (lastEstimatedPosition != null && estimatedPosition != null) {
            return estimatedPosition.position.acquisitionTime - lastEstimatedPosition.position.acquisitionTime;
        }
        return null;
    }

    public EstimatedPosition getLastEstimatedPosition() {
        return lastEstimatedPosition;
    }

    public EstimatedOrientation getLastEstimatedOrientation() {
        return lastEstimatedOrientation;
    }

    public void telemetry() {
        EstimatedPosition estimatedPosition = getEstimatedPosition();
        EstimatedOrientation orientation = getEstimatedOrientation();
        if (estimatedPosition != null) {
            Position position = estimatedPosition.position.toUnit(DistanceUnit.INCH);
            telemetry.addData("[Loc] Position Source", estimatedPosition.source.toString());
            telemetry.addData("[Loc] Position", String.format("%.1f, %.1f, %.1f", position.x, position.y, position.z));


        } else {
            telemetry.addData("[Loc] Position", "unknown");
        }

        if (estimatedVelocity != null) {
            Velocity velocity = estimatedVelocity.toUnit(DistanceUnit.INCH);
            telemetry.addData("[Loc] Velocity (in/s)", "X: %.1f, Y: %.1f, SPD: %.1f", velocity.xVeloc, velocity.yVeloc, speed(velocity));
        } else {
            telemetry.addData("[Loc] Velocity (in/s)", "unknown");
        }



        if (orientation != null) {
            telemetry.addData("[Loc] Orientation Source", orientation.source.toString());
            Orientation rotation = orientation.orientation;
            telemetry.addData("[Loc] Heading", "%.0f", rotation.thirdAngle);
        } else {
            telemetry.addData("[Loc] Orientation", "unknown");
        }

        if (estimatedAngularVelocity != null) {
            telemetry.addData("[Loc] Angular Velocity (deg/s)", estimatedAngularVelocity.zRotationRate);
        } else {
            telemetry.addData("[Loc] Angular Velocity (deg/s)", "unknown");
        }

        if (imuToWorldRotation != null) {
            telemetry.addData("[Loc] IMU World Angle Offset", imuToWorldRotation);
        } else {
            telemetry.addData("[Loc] IMU World Angle Offset", "not set");
        }
    }

    /**
     * Updates the data needed to estimate position and transform. This should be called in every
     * loop of the OpMode.
     * @param hardware The robot hardware; it will be used to fetch Vuforia and encoder data
     */
    public void updateRobotTransform(RobotHardware hardware) {
        this.setLatestAcquisitionTime();
        // Clear previous delta
        deltaPosition = null;

        boolean foundTarget = this.updateTransformWithVuforia(hardware);
        if (hardware.revIMU != null) {
            this.updateIMUOrientation(hardware.revIMU);
        }
        if (hardware.omniDrive != null) {
            this.updateTransformWithEncodersAndIMU(hardware, !foundTarget);
        }

        boolean newVuforiaInfo = false;
        boolean newEncodersInfo = false;

        if (lastVuforiaTransform != null) {
            newVuforiaInfo = lastVuforiaTransform.acquisitionTime >= latestAcquisitionTime;
        }
        if (lastEncoderPosition != null) {
            newEncodersInfo = lastEncoderPosition.acquisitionTime >= latestAcquisitionTime;
        }

        // Estimate Position
        if (newVuforiaInfo) {
            VectorF translation = lastVuforiaTransform.transform.getTranslation();
            float x = translation.get(0);
            float y = translation.get(1);
            float z = translation.get(2);
            Position position = new Position(DistanceUnit.MM, x, y, z, lastVuforiaTransform.acquisitionTime);
            estimatedPosition = new EstimatedPosition(PositionSource.VUFORIA, position);
        } else if (newEncodersInfo){
            estimatedPosition = new EstimatedPosition(PositionSource.ENCODERS, lastEncoderPosition);
        }

        // Estimate Orientation
        if (lastVuforiaTransform != null) {
            Orientation rotation = Orientation.getOrientation(lastVuforiaTransform.transform, EXTRINSIC, XYZ, DEGREES);
            rotation.acquisitionTime = lastVuforiaTransform.acquisitionTime;
            estimatedOrientation = new EstimatedOrientation(OrientationSource.VUFORIA, rotation);
            if (vuforiaOverwritesOtherOrientations) {
                setIMUToWorldOffset(hardware.revIMU, rotation.thirdAngle);
            }
        } else if (lastIMUOrientation != null) {
            estimatedOrientation = new EstimatedOrientation(OrientationSource.IMU, lastIMUOrientation);
        }

        // Calculate velocity
        if (lastEstimatedPosition != null && estimatedPosition != null) {
            Position currentPosition = estimatedPosition.position.toUnit(DistanceUnit.MM);
            Position lastPosition = lastEstimatedPosition.position.toUnit(DistanceUnit.MM);
            double deltaMs = estimatedPosition.position.acquisitionTime - lastEstimatedPosition.position.acquisitionTime;
            double deltaSeconds = deltaMs / 1000;
            double deltaX = currentPosition.x - lastPosition.x;
            double deltaY = currentPosition.y - lastPosition.y;
            deltaPosition = new Position(DistanceUnit.MM, deltaX, deltaY, 0, currentPosition.acquisitionTime);
            estimatedVelocity = new Velocity(DistanceUnit.MM, deltaX / deltaSeconds, deltaY / deltaSeconds, 0, currentPosition.acquisitionTime);
        }

        // Calculate angular velocity
        if (lastEstimatedOrientation != null && estimatedOrientation != null) {
            double currentHeading = estimatedOrientation.orientation.thirdAngle;
            double lastHeading = lastEstimatedOrientation.orientation.thirdAngle;
            double deltaMs = estimatedOrientation.orientation.acquisitionTime - lastEstimatedOrientation.orientation.acquisitionTime;
            double deltaSeconds = deltaMs / 1000;
            double deltaHeading = Localizer.angularDifferenceInDegrees(lastHeading, currentHeading);
            estimatedAngularVelocity = new AngularVelocity(DEGREES, 0, 0, (float) (deltaHeading / deltaSeconds), estimatedOrientation.orientation.acquisitionTime);
        }

        lastEstimatedPosition = estimatedPosition;
        lastEstimatedOrientation = estimatedOrientation;

    }

    //region Vuforia

    public class VuforiaTransform {
        public long acquisitionTime;
        public OpenGLMatrix transform;

        public VuforiaTransform(OpenGLMatrix transform) {
            this.acquisitionTime = System.currentTimeMillis();
            this.transform = transform;

        }
    }

    public void syncEncodersWithVuforia() {
        this.allowVuforiaMeasurement = true;
    }

    /**
     * Sets the camera matrix for navigation.
     * @param hardware Robot Hardware being used
     * @param newCameraMatrix The new camera matrix to be applied
     */
    public void setCameraMatrix(RobotHardware hardware, OpenGLMatrix newCameraMatrix) {
        this.cameraMatrix = newCameraMatrix;
        for (VuforiaTrackable trackable : vuforiaTrackables) {
            VuforiaTrackableDefaultListener listener = (VuforiaTrackableDefaultListener) trackable.getListener();
            listener.setPhoneInformation(this.cameraMatrix, hardware.vuforiaParameters.cameraDirection);
        }
    }

    /**
     * Sets the camera matrix for navigation using given offset and rotation.
     * @param hardware Robot Hardware being used
     * @param cameraOffset Offset of the camera relative to the robot's center (0,0,0)
     * @param cameraRotation Rotation of the camera relative to the robot's forward direction
     */
    public void setCameraMatrix(RobotHardware hardware, Position cameraOffset, Orientation cameraRotation) {
        Position cameraOffsetMM = cameraOffset.toUnit(DistanceUnit.MM);
        this.cameraMatrix = OpenGLMatrix.translation((float) cameraOffsetMM.x, (float) cameraOffsetMM.y, (float) cameraOffsetMM.z).multiplied(cameraRotation.getRotationMatrix());
        RobotLog.ii("PM Localizer", "phone=%s", this.cameraMatrix.formatAsTransform());
        for (VuforiaTrackable trackable : vuforiaTrackables) {
            VuforiaTrackableDefaultListener listener = (VuforiaTrackableDefaultListener) trackable.getListener();
            listener.setCameraLocationOnRobot(hardware.webcamName, this.cameraMatrix);
        }
    }

    public void loadUltimateGoalTrackables(RobotHardware hardware) {
        this.loadUltimateGoalTrackables(hardware,
                new Position(DistanceUnit.MM, 0,0,0,0),
                new Orientation(EXTRINSIC, XYZ, DEGREES, 0,-90,0,0));
    }

    /**
     * Load the Ultimate Goal Trackables. Largely copied from ConceptVuforiaUltimteGoalNavigationWebcam
     */
    public void loadUltimateGoalTrackables(RobotHardware hardware, Position cameraOffset, Orientation cameraRotation) {
        VuforiaTrackables targetsUltimateGoal = hardware.vuforia.loadTrackablesFromAsset("UltimateGoal");
        VuforiaTrackable blueTowerGoalTarget = targetsUltimateGoal.get(0);
        blueTowerGoalTarget.setName("Blue Tower Goal Target");
        VuforiaTrackable redTowerGoalTarget = targetsUltimateGoal.get(1);
        redTowerGoalTarget.setName("Red Tower Goal Target");
        VuforiaTrackable redAllianceTarget = targetsUltimateGoal.get(2);
        redAllianceTarget.setName("Red Alliance Target");
        VuforiaTrackable blueAllianceTarget = targetsUltimateGoal.get(3);
        blueAllianceTarget.setName("Blue Alliance Target");
        VuforiaTrackable frontWallTarget = targetsUltimateGoal.get(4);
        frontWallTarget.setName("Front Wall Target");

        vuforiaTrackables.addAll(targetsUltimateGoal);

        /**
         * In order for localization to work, we need to tell the system where each target is on the field, and
         * where the phone resides on the robot.  These specifications are in the form of <em>transformation matrices.</em>
         * Transformation matrices are a central, important concept in the math here involved in localization.
         * See <a href="https://en.wikipedia.org/wiki/Transformation_matrix">Transformation Matrix</a>
         * for detailed information. Commonly, you'll encounter transformation matrices as instances
         * of the {@link OpenGLMatrix} class.
         *
         * If you are standing in the Red Alliance Station looking towards the center of the field,
         *     - The X axis runs from your left to the right. (positive from the center to the right)
         *     - The Y axis runs from the Red Alliance Station towards the other side of the field
         *       where the Blue Alliance Station is. (Positive is from the center, towards the BlueAlliance station)
         *     - The Z axis runs from the floor, upwards towards the ceiling.  (Positive is above the floor)
         *
         * Before being transformed, each target image is conceptually located at the origin of the field's
         *  coordinate system (the center of the field), facing up.
         */

        // Set the position of the perimeter targets with relation to origin (center of field)
        redAllianceTarget.setLocation(OpenGLMatrix
                .translation(0, -halfField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 180)));

        blueAllianceTarget.setLocation(OpenGLMatrix
                .translation(0, halfField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 0)));
        frontWallTarget.setLocation(OpenGLMatrix
                .translation(-halfField, 0, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0 , 90)));

        // The tower goal targets are located a quarter field length from the ends of the back perimeter wall.
        blueTowerGoalTarget.setLocation(OpenGLMatrix
                .translation(halfField, quadField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0 , -90)));
        redTowerGoalTarget.setLocation(OpenGLMatrix
                .translation(halfField, -quadField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));

        this.setCameraMatrix(hardware, cameraOffset, cameraRotation);

        targetsUltimateGoal.activate();
    }

    private boolean updateTransformWithVuforia(RobotHardware hardware) {
        // Clear the currently saved vuforia transform
        lastVuforiaTransform = null;
        boolean didFindTarget = false;

        for (VuforiaTrackable trackable : vuforiaTrackables) {
            VuforiaTrackableDefaultListener listener = (VuforiaTrackableDefaultListener) trackable.getListener();
            if (listener.isVisible()) {
                hardware.telemetry.addData("[Localizer] Visible Target:", trackable.getName());
                OpenGLMatrix imageLocationRelRobot = listener.getFtcCameraFromTarget();
                float relX = imageLocationRelRobot.getTranslation().get(0);
                float relY = imageLocationRelRobot.getTranslation().get(1);
                float relZ = imageLocationRelRobot.getTranslation().get(2);
                Position imagePositionRelRobot = new Position(DistanceUnit.MM, relX,relY,relZ, 0);
                double distance = Localizer.distanceXZ(zero, imagePositionRelRobot, DistanceUnit.INCH);
                if (distance < distanceToAcceptVuforiaInches || allowVuforiaMeasurement) {
                    OpenGLMatrix robotTransform = listener.getUpdatedRobotLocation();
                    if (robotTransform != null) {
                        didFindTarget = true;
                        lastVuforiaTransform = new VuforiaTransform(robotTransform);
                        if (vuforiaOverwritesOtherLocations) {
                            VectorF translation = lastVuforiaTransform.transform.getTranslation();
                            float x = translation.get(0);
                            float y = translation.get(1);
                            float z = translation.get(2);
                            Position position = new Position(DistanceUnit.MM, x, y, z, System.currentTimeMillis());
                            lastEncoderPosition = position;
                        }
                        allowVuforiaMeasurement = false;
                    }
                }
            }
        }
        return didFindTarget;
    }

    //endregion

    // region Encoders

    /**
     * Calculates robot transform using encoders and the IMU.
     * The IMU is relied on for rotational measurements.
     * @param hardware The hardware to use
     */

    private void updateTransformWithEncodersAndIMU(RobotHardware hardware, boolean calculateDelta) {
        if (lastEncoderPosition == null || lastIMUOrientation == null) {
            // If there is no previous position data from Vuforia/defined robot start and/or IMU data, location/rotation can not be determined.
            return;
        }

        // Get current encoder positions
        int currentFrontLeft = hardware.omniDrive.frontLeft.getCurrentPosition();
        int currentFrontRight = hardware.omniDrive.frontRight.getCurrentPosition();
        int currentBackLeft = hardware.omniDrive.backLeft.getCurrentPosition();
        int currentBackRight = hardware.omniDrive.backRight.getCurrentPosition();

        // Perform location estimation if there's delta information to work with
        if (lastFrontLeft != null
                && lastFrontRight != null
                && lastBackLeft != null
                && lastBackRight != null
                && calculateDelta) {

            // Get position deltas
            int deltaFrontLeft = currentFrontLeft - lastFrontLeft;
            int deltaFrontRight = currentFrontRight - lastFrontRight;
            int deltaBackLeft = currentBackLeft - lastBackLeft;
            int deltaBackRight = currentBackRight - lastBackRight;

            // Get robot centric delta measurements
            double mmPerCount = 1.0/hardware.omniDrive.getCountsPerUnit(DistanceUnit.MM);
            double xRobotRelMovement = encodersXScaleFactor*(mmPerCount/4)*(deltaFrontLeft+deltaBackRight-deltaFrontRight-deltaBackLeft);
            double yRobotRelMovement = encodersYScaleFactor*(mmPerCount/4)*(deltaFrontLeft+deltaFrontRight+deltaBackLeft+deltaBackRight);

            // Translate robot-centric movements into field deltas and apply
            double robotHeading = Math.toRadians(lastIMUOrientation.thirdAngle);
            double fieldXMovement = -((xRobotRelMovement*Math.cos(robotHeading)) - (yRobotRelMovement*Math.sin(robotHeading)));
            double fieldYMovement = -((xRobotRelMovement*Math.sin(robotHeading)) + (yRobotRelMovement*Math.cos(robotHeading)));
            Position lastMMPosition = lastEncoderPosition.toUnit(DistanceUnit.MM);
            double lastX = lastMMPosition.x;
            double lastY = lastMMPosition.y;
            lastEncoderPosition = new Position(DistanceUnit.MM, lastX + fieldXMovement, lastY + fieldYMovement, 0, System.currentTimeMillis());
        }

        // Set current positions for next transform update.
        lastFrontLeft = currentFrontLeft;
        lastFrontRight = currentFrontRight;
        lastBackLeft = currentBackLeft;
        lastBackRight = currentBackRight;

    }
    //endregion

    //region IMU
    /**
     * This will attempt to find the differences of orientation between the field-centric system and
     * the IMU reference system.
     * @return Whether calibration was successful
     */

    public boolean attemptIMUToWorldCalibration(BNO055IMU imu) {
        if (lastVuforiaTransform != null && imu.isGyroCalibrated()) {
            return true;
        }
        return false;
    }

    public void setRobotStart(BNO055IMU imu, Position startingPosition, double startingHeading) {
        this.lastEncoderPosition = startingPosition;
        this.setIMUToWorldOffset(imu, startingHeading);
    }

    public void setIMUToWorldOffset(BNO055IMU imu, double worldHeading) {
        Orientation imuRotation = imu.getAngularOrientation(EXTRINSIC, XYZ, DEGREES);
        imuToWorldRotation = Localizer.angularDifferenceInDegrees(worldHeading, imuRotation.thirdAngle);
    }

    public Double getImuToWorldRotation() {
        return this.imuToWorldRotation;
    }

    private void updateIMUOrientation(BNO055IMU imu) {
        Orientation imuOrientation = imu.getAngularOrientation(EXTRINSIC, XYZ, DEGREES);
        this.lastRawIMUOrientation = imuOrientation;
        if (this.imuToWorldRotation != null) {
            double imuHeading = imu.getAngularOrientation(EXTRINSIC, XYZ, DEGREES).thirdAngle;
            double worldHeading = Localizer.headingWrapInDegrees(imuHeading + this.imuToWorldRotation);
            this.lastIMUOrientation = new Orientation(EXTRINSIC, XYZ, DEGREES, imuOrientation.firstAngle, imuOrientation.secondAngle, (float) worldHeading, System.currentTimeMillis());
        }
    }

    //endregion

    //region Static Methods

    public static Position mirrorPositionOverTeamLine(Position position) {
        Position copy = new Position(position.unit, position.x, -position.y, position.z, position.acquisitionTime);
        return copy;
    }

    public static RobotTransform mirrorTransformPositionOverTeamLine(RobotTransform transform) {
        return new RobotTransform(mirrorPositionOverTeamLine(transform.position), transform.heading);
    }

    public static RobotTransform mirrorTransformOverTeamLine(RobotTransform transform) {
        double angleDiffFrom90 = transform.heading - 90;
        return new RobotTransform(mirrorPositionOverTeamLine(transform.position), headingWrapInDegrees(90 - angleDiffFrom90));
    }

    public static RobotTransform[] mirrorTransformsOverTeamLine(RobotTransform[] transforms) {
        RobotTransform[] new_transforms = new RobotTransform[transforms.length];
        for (int i = 0; i < new_transforms.length; i++) {
            RobotTransform transform = transforms[i];
            new_transforms[i] = mirrorTransformOverTeamLine(transform);
        }
        return new_transforms;
    }

    public static Position addPositions(Position a, Position b, DistanceUnit unit) {
        Position unit_a = a.toUnit(unit);
        Position unit_b = b.toUnit(unit);
        return new Position(unit, unit_a.x + unit_b.x, unit_a.y + unit_b.y, unit_a.z + unit_b.z, a.acquisitionTime);
    }

    public static Position velocityToDelta(Velocity velocity, double seconds, DistanceUnit unit) {
        Velocity unit_velocity = velocity.toUnit(unit);
        return new Position(unit, unit_velocity.xVeloc * seconds, unit_velocity.yVeloc * seconds, unit_velocity.zVeloc * seconds, 0);
    }

    public static double angVelocityToDelta(AngularVelocity angVelocity, double seconds) {
        return angVelocity.zRotationRate * seconds;
    }

    /**
     * Compute the XY distance between two Positions
     * @param a Position 1
     * @param b Position 2
     * @return XY Distance
     */
    public static double distance(Position a, Position b, DistanceUnit unit) {
        Position unit_a = a.toUnit(unit);
        Position unit_b = b.toUnit(unit);
        return Math.hypot(unit_b.x - unit_a.x, unit_b.y - unit_a.y);
    }

    /**
     * Compute the XY distance between two Positions
     * @param a Position 1
     * @param b Position 2
     * @return XY Distance
     */
    public static double distanceXZ(Position a, Position b, DistanceUnit unit) {
        Position unit_a = a.toUnit(unit);
        Position unit_b = b.toUnit(unit);
        return Math.hypot(unit_b.x - unit_a.x, unit_b.z - unit_a.z);
    }

    /**
     * Returns speed from velocity
     * @param velocity Velocity
     * @return Speed in provided velocity's units
     */
    public static double speed(Velocity velocity) {
        return Math.hypot(velocity.xVeloc, velocity.yVeloc);
    }

    /**
     *
     * @param start Starting angle
     * @param end End angle
     * @return An angle from -180 to 180, where positive angles indicate a rotation to the left and vice versa.
     */
    public static final Position zero = new Position(DistanceUnit.INCH,0,0,0,0);

    public static double angularDifferenceInDegrees(double start, double end) {
        return mod((start-end+180), 360) - 180;
    }

    public static double mod(double x, double n) {
        return (((x % n) + n) % n);
    }

    public static double headingWrapInDegrees(double angle) {
        return Localizer.mod(angle + 180, 360) - 180;
    }

    public static double headingWrapInRadians(double angle) {
        return Localizer.mod(angle + Math.PI, 2*Math.PI) - Math.PI;
    }

    public static double atan2InDegrees(Position a, Position b) {
        Position unit_a = a.toUnit(DistanceUnit.CM);
        Position unit_b = b.toUnit(DistanceUnit.CM);
        return Math.toDegrees(Math.atan2((unit_b.x-unit_a.x),-(unit_b.y-unit_a.y)));
    }
    //endregion

}

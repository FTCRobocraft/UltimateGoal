package org.firstinspires.ftc.teamcode.action;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.teamcode.playmaker.Action;
import org.firstinspires.ftc.teamcode.playmaker.Localizer;
import org.firstinspires.ftc.teamcode.playmaker.Localizer.RobotTransform;

import org.firstinspires.ftc.teamcode.playmaker.RobotHardware;

public class LocalizerMoveAction implements Action {

    RobotTransform[] transforms;
    private int currentTransformIndex = 0;
    private double fullSpeed;
    private double preciseSpeed;
    private FollowPathMethod followPathMethod;

    // Linear Path Following Configuration
    static final double LINEAR_SLOWDOWN_DISTANCE_INCHES = 16;


    // Tolerances
    static final double DISTANCE_TOLERANCE_INCHES = 1;
    static final double HEADING_TOLERANCE_DEGREES = 2;
    static final double PRECISE_ROTATE_SPEED = 0.275;

    public enum FollowPathMethod {
        LINEAR, // not gonna be implemented
        FAST, // only cares about final heading position
    }

    public LocalizerMoveAction(RobotTransform transform, double speed, double preciseSpeed, FollowPathMethod pathMethod) {
        this.transforms = new RobotTransform[] { transform };
        this.fullSpeed = speed;
        this.preciseSpeed = preciseSpeed;
        this.followPathMethod  = pathMethod;
    }

    public LocalizerMoveAction(RobotTransform[] transforms, double speed, double preciseSpeed, FollowPathMethod pathMethod) {
        this.transforms = transforms;
        this.fullSpeed = speed;
        this.preciseSpeed = preciseSpeed;
        this.followPathMethod = pathMethod;
    }

    @Override
    public void init(RobotHardware hardware) {

    }

    @Override
    public boolean doAction(RobotHardware hardware) {
        // Get current target to follow
        RobotTransform currentTarget = transforms[currentTransformIndex];

        // Get current robot position and orientation
        Position currentPosition = hardware.localizer.estimatePosition().position;
        Orientation currentOrientation = hardware.localizer.estimateOrientation().orientation;
        if (currentPosition == null || currentOrientation == null) {
            // If either of these are null, it means that the robot's location is unknown
            // This point shouldn't be reached with input from the encoders
            // Return false so that the robot has the opportunity to localize itself again
            return false;
        }
        double currentHeading = currentOrientation.thirdAngle;

        // Calculate distances / angles to target
        double distanceToTargetInInches = Localizer.distance(currentPosition, currentTarget.position, DistanceUnit.INCH);

        // This is the angular difference between the robot's current position and the target position in the field coordinate space.
        double angDiffBetweenPositionsDegrees = Localizer.atan2InDegrees(currentPosition, currentTarget.position);

        // This is the angular difference between the robot's forward heading and the target heading.
        double angDiffToTargetHeadingDegrees = Localizer.angularDifferenceInDegrees(currentHeading, currentTarget.heading);

        // This is the angular difference between the robot's forward heading and the heading towards the target position.
        double angDiffBetweenForwardAndTargetPosDegrees = Localizer.angularDifferenceInDegrees(currentHeading, angDiffBetweenPositionsDegrees);

        double robotMoveAngleRadians = 0;
        double robotRotation = 0;
        double speed = this.fullSpeed;

        boolean withinDistanceTolerance = distanceToTargetInInches <= DISTANCE_TOLERANCE_INCHES;
        boolean withinHeadingTolerance = angDiffToTargetHeadingDegrees <= HEADING_TOLERANCE_DEGREES;

        switch (followPathMethod) {
            case LINEAR:
                if (distanceToTargetInInches <= LINEAR_SLOWDOWN_DISTANCE_INCHES) {
                    speed = preciseSpeed;
                }

                if (withinDistanceTolerance && withinHeadingTolerance) {
                    currentTransformIndex++;
                } else if (withinDistanceTolerance) {
                    speed = 0;
                    robotRotation = angDiffToTargetHeadingDegrees > 0 ? preciseSpeed : -preciseSpeed;
                } else {
                    robotMoveAngleRadians = angDiffBetweenForwardAndTargetPosDegrees;
                    robotRotation = angDiffToTargetHeadingDegrees / 180;
                }
                break;
            case FAST:
                if (currentTransformIndex < transforms.length - 1) {
                    // For fast operation, we only care about reaching the target heading for the final position.
                    // So, if the robot is heading towards the last target, it will focus on reaching
                    // the target as quickly as possible by going in the forward direction
                    robotMoveAngleRadians = Math.toRadians(angDiffBetweenForwardAndTargetPosDegrees);
                    robotRotation = angDiffBetweenForwardAndTargetPosDegrees / 180;
                    if (distanceToTargetInInches <= DISTANCE_TOLERANCE_INCHES) {
                        currentTransformIndex++;
                    }
                } else {
                    // For the final target, the robot needs to reach the correct target heading.
                    robotMoveAngleRadians = Math.toRadians(angDiffBetweenForwardAndTargetPosDegrees);
                    robotRotation = angDiffToTargetHeadingDegrees / 180;
                    if (withinDistanceTolerance && withinHeadingTolerance) {
                        currentTransformIndex++;
                    } else if (withinDistanceTolerance) {
                        speed = 0;
                        robotRotation = angDiffBetweenForwardAndTargetPosDegrees > 0 ? preciseSpeed : -preciseSpeed;
                    }
                }

                break;
        }

        if (currentTransformIndex >= transforms.length) {
            // Once the robot reached the target position, stop moving and end the action
            hardware.omniDrive.stopDrive();
            return true;
        } else {
            hardware.omniDrive.move(speed, robotMoveAngleRadians, robotRotation);
            return false;
        }
    }

    @Override
    public Double progress() {
        return null;
    }

    @Override
    public String progressString() {
        if (currentTransformIndex < transforms.length) {
            RobotTransform currentTarget = transforms[currentTransformIndex];
            String progress = String.format("Current Target X: %.1f Y: %.1f H: %.1f",
                    currentTarget.position.x,
                    currentTarget.position.y,
                    currentTarget.heading);
            return progress;
        }
        return null;
    }

    @Override
    public Object getActionResult() {
        return null;
    }
}
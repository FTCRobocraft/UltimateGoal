package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class OmniDrive {

    public DcMotor frontLeft;
    public DcMotor frontRight;
    public DcMotor backLeft;
    public DcMotor backRight;
    static final Double DEFAULT_ACCELERATION = null;
    public SpeedController flSpeedController = new SpeedController(DEFAULT_ACCELERATION);
    public SpeedController frSpeedController = new SpeedController(DEFAULT_ACCELERATION);
    public SpeedController blSpeedController = new SpeedController(DEFAULT_ACCELERATION);
    public SpeedController brSpeedController = new SpeedController(DEFAULT_ACCELERATION);
    public double horizontalMovementScaleFactor = 1;


    private Double countsPerInch;
    private Double countsPerDegreee;

    public enum Direction {
        FORWARD,
        LEFT,
        RIGHT,
        BACKWARD,
        FORWARD_LEFT,
        FORWARD_RIGHT,
        BACKWARD_LEFT,
        BACKWARD_RIGHT,
        ROTATE_LEFT,
        ROTATE_RIGHT
    }

    public static class SpeedController {

        Double acceleration;
        double targetPower = 0;
        double currentPower = 0;
        double lastPowerCalculateTime = 0;

        public SpeedController(Double acceleration) {
            setAcceleration(acceleration);
        }

        public void setAcceleration(Double acceleration) {
            this.acceleration = acceleration;
        }

        public void setPower(double power) {
            if (acceleration == null) {
                this.immediatelySetPower(power);
            } else {
                targetPower = power;
                //lastPowerCalculateTime = System.currentTimeMillis();
            }
        }

        public void immediatelySetPower(double power) {
            targetPower = power;
            currentPower = power;
            lastPowerCalculateTime = System.currentTimeMillis();
        }

        public double getPower() {
            if (lastPowerCalculateTime == 0 || acceleration == null) {
                lastPowerCalculateTime = System.currentTimeMillis();
                return currentPower;
            }

            double currentTime = System.currentTimeMillis();
            double deltaTimeS = (currentTime - lastPowerCalculateTime) / 1000;
            double powerDelta = acceleration * deltaTimeS;
            if (currentPower < targetPower) {
                currentPower = Math.min(targetPower, currentPower + powerDelta);
            } else if (currentPower > targetPower) {
                currentPower = Math.max(targetPower, currentPower - powerDelta);
            }

            lastPowerCalculateTime = currentTime;
            return currentPower;
        }


    }


    public OmniDrive(DcMotor frontLeft, DcMotor frontRight,
                     DcMotor backLeft, DcMotor backRight) {
        this.frontLeft = frontLeft;
        this.frontRight = frontRight;
        this.backLeft = backLeft;
        this.backRight = backRight;

    }

    public void setAcceleration(Double acceleration) {
        flSpeedController.setAcceleration(acceleration);
        frSpeedController.setAcceleration(acceleration);
        blSpeedController.setAcceleration(acceleration);
        brSpeedController.setAcceleration(acceleration);
    }

    public Double getCountsPerUnit(DistanceUnit unit) {
        if (countsPerInch != null) {
            return this.countsPerInch/unit.fromInches(1);
        }
        return null;
    }

    public Double getCountsPerInch() {
        return countsPerInch;
    }

    public void setCountsPerInch(double countsPerInch) {
        this.countsPerInch = countsPerInch;
    }

    public Double getCountsPerDegreee() {
        return countsPerDegreee;
    }

    public void setCountsPerDegree(double countsPerDegreee) {
        this.countsPerDegreee = countsPerDegreee;
    }

    public void setWheelMeasurements(double countsPerDegreee, double countsPerInch) {
        this.countsPerDegreee = countsPerDegreee;
        this.countsPerInch = countsPerInch;
    }

    public void moveForward(double power) {
        flSpeedController.setPower(power);
        frSpeedController.setPower(power);
        blSpeedController.setPower(power);
        brSpeedController.setPower(power);
    }

    public void moveBackward(double power) {
        flSpeedController.setPower(-power);
        frSpeedController.setPower(-power);
        blSpeedController.setPower(-power);
        brSpeedController.setPower(-power);
    }

    public void moveLeft(double power) {
        flSpeedController.setPower(-power);
        frSpeedController.setPower(power);
        blSpeedController.setPower(power);
        brSpeedController.setPower(-power);
    }

    public void moveRight(double power) {
        flSpeedController.setPower(power);
        frSpeedController.setPower(-power);
        blSpeedController.setPower(-power);
        brSpeedController.setPower(power);
    }

    public void rotateRight(double power) {
        flSpeedController.setPower(power);
        frSpeedController.setPower(-power);
        blSpeedController.setPower(power);
        brSpeedController.setPower(-power);
    }

    public void rotateLeft(double power) {
        flSpeedController.setPower(-power);
        frSpeedController.setPower(power);
        blSpeedController.setPower(-power);
        brSpeedController.setPower(power);
    }

    public void moveForwardLeft(double power) {
        flSpeedController.setPower(0);
        frSpeedController.setPower(power);
        blSpeedController.setPower(power);
        brSpeedController.setPower(0);
    }

    public void moveForwardRight(double power) {
        flSpeedController.setPower(power);
        frSpeedController.setPower(0);
        blSpeedController.setPower(0);
        brSpeedController.setPower(power);
    }

    public void moveBackwardLeft(double power) {
        flSpeedController.setPower(-power);
        frSpeedController.setPower(0);
        blSpeedController.setPower(0);
        brSpeedController.setPower(-power);
    }

    public void moveBackwardRight(double power) {
        flSpeedController.setPower(0);
        frSpeedController.setPower(-power);
        blSpeedController.setPower(-power);
        brSpeedController.setPower(0);
    }

    public void updateMotorPowers() {
        frontLeft.setPower(flSpeedController.getPower());
        frontRight.setPower(frSpeedController.getPower());
        backLeft.setPower(blSpeedController.getPower());
        backRight.setPower(brSpeedController.getPower());
    }

    /**
     * Move and/or rotate the robot along an axis relative to the robot's center.

     * @param power The power to move everything
     * @param angleInRadians The heading (IN RADIANS) to move the robot in, 0 is forward rotating clockwise.
     * @param rotation How fast to rotate from -1 to 1 where 1 indicates CW rotation and full rotation w/ no movement.
     */
    public void move(double power, double angleInRadians, double rotation, boolean useAcceleration) {

        double pi4 = Math.PI / 4;

        // Get raw powers
        double scaleFactor = (1 + horizontalMovementScaleFactor * Math.sin(Math.abs(angleInRadians)));
        double fl_power = (power * Math.sin(angleInRadians + pi4) * scaleFactor) + rotation;
        double fr_power = (power * Math.cos(angleInRadians + pi4) * scaleFactor) - rotation;
        double bl_power = (power * Math.cos(angleInRadians + pi4) * scaleFactor) + rotation;
        double br_power = (power * Math.sin(angleInRadians + pi4) * scaleFactor) - rotation;

//        double avg_power = (Math.abs(fl_power) + Math.abs(fr_power) + Math.abs(bl_power) + Math.abs(br_power)) / 4.0;
//        if (avg_power < power) {
//            double power_upscale = power/avg_power;
//            fl_power *= power_upscale;
//            fr_power *= power_upscale;
//            bl_power *= power_upscale;
//            br_power *= power_upscale;
//        }

        // Calculate unit vector and apply power magnitude
        double power_max = Math.max(Math.max(fl_power, fr_power), Math.max(bl_power, br_power));

        // Scale power if one motor exceeds 1
        if (power_max > 1.0) {
            fl_power = fl_power / power_max;
            fr_power = fr_power / power_max;
            bl_power = bl_power / power_max;
            br_power = br_power / power_max;
        }
        // Set motor powers
        if (useAcceleration) {
            flSpeedController.setPower(fl_power);
            frSpeedController.setPower(fr_power);
            blSpeedController.setPower(bl_power);
            brSpeedController.setPower(br_power);
        } else {
            flSpeedController.immediatelySetPower(fl_power);
            frSpeedController.immediatelySetPower(fr_power);
            blSpeedController.immediatelySetPower(bl_power);
            brSpeedController.immediatelySetPower(br_power);
        }

    }

    public void stopDrive() {
        flSpeedController.immediatelySetPower(0);
        frSpeedController.immediatelySetPower(0);
        blSpeedController.immediatelySetPower(0);
        brSpeedController.immediatelySetPower(0);
    }

    public void setMode(DcMotor.RunMode runMode) {
        frontLeft.setMode(runMode);
        frontRight.setMode(runMode);
        backLeft.setMode(runMode);
        backRight.setMode(runMode);
    }

    public void setZeroPowerMode(DcMotor.ZeroPowerBehavior behavior) {
        frontLeft.setZeroPowerBehavior(behavior);
        frontRight.setZeroPowerBehavior(behavior);
        backLeft.setZeroPowerBehavior(behavior);
        backRight.setZeroPowerBehavior(behavior);
    }

    public void dpadMove(Gamepad gamepad, float power, boolean reverse) {
        if (reverse) {
            if (gamepad.dpad_up && gamepad.dpad_left) {
                moveBackwardRight(power);
            } else if (gamepad.dpad_up && gamepad.dpad_right) {
                moveBackwardLeft(power);
            } else if (gamepad.dpad_down && gamepad.dpad_left) {
                moveForwardRight(power);
            } else if (gamepad.dpad_down && gamepad.dpad_right) {
                moveForwardLeft(power);
            } else if (gamepad.dpad_up) {
                moveBackward(power);
            } else if (gamepad.dpad_left) {
                moveRight(power);
            } else if (gamepad.dpad_right) {
                moveLeft(power);
            } else if (gamepad.dpad_down) {
                moveForward(power);
            } else {
                stopDrive();
            }
        } else {
            if (gamepad.left_trigger > 0) {
                rotateLeft(gamepad.left_trigger);
            } else if (gamepad.right_trigger > 0) {
                rotateRight(gamepad.right_trigger);
            } else if (gamepad.dpad_up && gamepad.dpad_left) {
                moveForwardLeft(power);
            } else if (gamepad.dpad_up && gamepad.dpad_right) {
                moveForwardRight(power);
            } else if (gamepad.dpad_down && gamepad.dpad_left) {
                moveBackwardLeft(power);
            } else if (gamepad.dpad_down && gamepad.dpad_right) {
                moveBackwardRight(power);
            } else if (gamepad.dpad_up) {
                moveForward(power);
            } else if (gamepad.dpad_left) {
                moveLeft(power);
            } else if (gamepad.dpad_right) {
                moveRight(power);
            } else if (gamepad.dpad_down) {
                moveBackward(power);
            } else {
                stopDrive();
            }
        }



//        if (gamepad.left_bumper) {
//            rotateLeft(0.25f);
//        } else if (gamepad.right_bumper) {
//            rotateRight(0.25f);
//        }
    }
}

/* Copyright (c) 2019 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.util.ArrayList;
import java.util.List;
import java.lang.Math;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XZY;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;

@TeleOp(name="ULTIMATEGOAL Vuforia Nav Webcam", group ="Concept")
public class ConceptVuforiaUltimateGoalNavigationWebcam extends LinearOpMode {

    private BNO055IMU InitImuSensor () {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        BNO055IMU configuredImu = hardwareMap.get(BNO055IMU.class, "imu");
        configuredImu.initialize(parameters);
        return configuredImu;
    }

    private DcMotor InitDcMotor (String motorName, DcMotorSimple.Direction direction, DcMotor.RunMode runMode) {
        DcMotor motor = hardwareMap.get(DcMotor.class, motorName);
        motor.setDirection(direction);
        motor.setMode(runMode);
        return motor;
    }

    DcMotor front_left, front_right, back_left, back_right, shooter;

    public void DriveMecanum(double robotSpeed, double drive, double strafe, double twist) {
        double[] speeds = {
                robotSpeed * (drive + strafe + twist),
                robotSpeed * (drive - strafe - twist),
                robotSpeed * (drive - strafe + twist),
                robotSpeed * (drive + strafe - twist)
        };

        // Loop through all values in the speeds[] array and find the biggest magnitudes
        double max = Math.abs(speeds[0]);
        for(int i = 0; i < speeds.length; i++) {
            if ( max < Math.abs(speeds[i]) ) max = Math.abs(speeds[i]);
        }

        // If and only if the maximum is outside of the range we want it to be,
        // normalize all the other speeds based on the given speed value.
        if (max > 1) {
            for (int i = 0; i < speeds.length; i++) speeds[i] /= max;
        }

        // apply the calculated values to the motors.
        front_left.setPower(speeds[0]);
        front_right.setPower(speeds[1]);
        back_left.setPower(speeds[2]);
        back_right.setPower(speeds[3]);
    }

    private static final String VUFORIA_KEY =
            "ATX1WMH/////AAABmZRCfAn9XENvtus7/8vEDzAjZUqBTtwyQRDZXSSxrKinG+QxvA51PBQf+MmhxVq5dnpIJ5pQcu8NIAo2ZJdWxJis8ws4Mx9efHxDgVdU4bwtKuOPmlUvdtwLHw8QZXpUA3nWj5G6HW2mX/7aE2hpj3sMsjjQKJMd7ify6hnYkVZwx3Ej1mODMH6FQ9UQTQuTJYN4vUGZQ3ggd/Yh+j1n+eldooxfN3G9SGh2eaa2vvfIsAsrZ7yrgHZUzw/fPDFa3MIk6lML0L02lKeksYGXqStHnzVJOiL/v2XNj+LmwpIvwWiijR4eTK3wl6oXjD0NP3i3bO37veiYrA0lTB+MZ18KpHQYiONQQ8GtTaP1eg3r";

    // Since ImageTarget trackables use mm to specifiy their dimensions, we must use mm for all the physical dimension.
    // We will define some constants and conversions here
    private static final float mmPerInch        = 25.4f;
    private static final float mmTargetHeight   = (6) * mmPerInch;          // the height of the center of the target image above the floor

    // Constants for perimeter targets
    private static final float halfField = 72 * mmPerInch;
    private static final float quadField  = 36 * mmPerInch;

    // Class Members
    private OpenGLMatrix lastLocation = null;
    private VuforiaLocalizer vuforia = null;

    WebcamName webcamName = null;

    private boolean targetVisible = false;
    private float phoneXRotate    = 0;
    private float phoneYRotate    = 0;
    private float phoneZRotate    = 0;

    @Override public void runOpMode() {
        webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        parameters.vuforiaLicenseKey = VUFORIA_KEY;

        parameters.cameraName = webcamName;
        parameters.useExtendedTracking = false;

        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        VuforiaTrackables targetsUltimateGoal = this.vuforia.loadTrackablesFromAsset("UltimateGoal");
        VuforiaTrackable redTowerGoalTarget = targetsUltimateGoal.get(1);
        redTowerGoalTarget.setName("Red Tower Goal Target");

        // For convenience, gather together all the trackable objects in one easily-iterable collection */
        List<VuforiaTrackable> allTrackables = new ArrayList<VuforiaTrackable>();
        allTrackables.addAll(targetsUltimateGoal);

        redTowerGoalTarget.setLocation(OpenGLMatrix
                .translation(halfField, -quadField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));

        final float CAMERA_FORWARD_DISPLACEMENT  = 4.0f * mmPerInch;   // eg: Camera is 4 Inches in front of robot-center
        final float CAMERA_VERTICAL_DISPLACEMENT = 8.0f * mmPerInch;   // eg: Camera is 8 Inches above ground
        final float CAMERA_LEFT_DISPLACEMENT     = 0;     // eg: Camera is ON the robot's center line

        OpenGLMatrix cameraLocationOnRobot = OpenGLMatrix
                    .translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT)
                    .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XZY, DEGREES, 90, 90, 0));

        for (VuforiaTrackable trackable : allTrackables) {
            ((VuforiaTrackableDefaultListener) trackable.getListener()).setCameraLocationOnRobot(parameters.cameraName, cameraLocationOnRobot);
        }

        // Init hardware stuff
        BNO055IMU imu = InitImuSensor();
        front_left    = InitDcMotor("front_left", DcMotorSimple.Direction.FORWARD, DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        front_right   = InitDcMotor("front_right", DcMotorSimple.Direction.REVERSE, DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        back_left     = InitDcMotor("back_left", DcMotorSimple.Direction.FORWARD, DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        back_right    = InitDcMotor("back_right", DcMotorSimple.Direction.REVERSE, DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooter    = InitDcMotor("shooter", DcMotorSimple.Direction.REVERSE, DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        targetsUltimateGoal.activate();

        while (!isStopRequested()) {

            double drive  = gamepad1.left_stick_y;
            double strafe = -gamepad1.left_stick_x;
            double twist  = -gamepad1.right_stick_x;

            double robotSpeed = gamepad1.right_trigger != 0 ? 1 : 0.5 ;

            DriveMecanum(robotSpeed, drive, strafe, twist);

            while (gamepad1.a) {
                double currentAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
                DriveMecanum(1, 0.5, 0, -currentAngle * 0.15);
            }
            if (gamepad1.dpad_right) {
                shooter.setPower(1 * robotSpeed);
            }
            else if (gamepad1.dpad_left){
                shooter.setPower(-1 * robotSpeed);
            }
            else {
                shooter.setPower(0);
            }
            // check all the trackable targets to see which one (if any) is visible.
            targetVisible = false;
            for (VuforiaTrackable trackable : allTrackables) {
                if (((VuforiaTrackableDefaultListener)trackable.getListener()).isVisible()) {
                    telemetry.addData("Visible Target", trackable.getName());
                    targetVisible = true;

                    OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener)trackable.getListener()).getLastTrackedPoseVuforiaCamera().rotated(DEGREES,0f,0f,0f, 0f);
                    if (robotLocationTransform == null)
                        continue;

                    VectorF translation = robotLocationTransform.getTranslation();
                    telemetry.addData("Pos (in)", "{X, Y, Z} = %.1f, %.1f, %.1f",
                            translation.get(0) / mmPerInch, translation.get(1) / mmPerInch, translation.get(2) / mmPerInch);

                    if (gamepad1.b) {
                        // Posição: 0 Y30
                        // Strafe - D/E | drive - F/B | twist - Rot
                        DriveMecanum(0.5, -1 * ((0.5 * (translation.get(2) / mmPerInch)) - 30), -1 * ((0.5 * (translation.get(0) / mmPerInch))), 0);
                        double currentAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
                        DriveMecanum(1, 0, 0, -currentAngle * 0.15);
                    }


                    if (robotLocationTransform != null) {
                        lastLocation = robotLocationTransform;
                    }
                }
            }
            telemetry.update();

            if (true)
                continue;
            // Provide feedback as to where the robot is located (if we know).
            if (targetVisible) {
                OpenGLMatrix xyz = lastLocation.transposed();
                // express position (translation) of robot in inches.
                VectorF translation = lastLocation.getTranslation();
                telemetry.addData("Pos (in)", "{X, Y, Z} = %.1f, %.1f, %.1f",
                        translation.get(0) / mmPerInch, translation.get(1) / mmPerInch, translation.get(2) / mmPerInch);

                // express the rotation of the robot in degrees.
                Orientation rotation = Orientation.getOrientation(lastLocation, EXTRINSIC, XYZ, DEGREES);
                double currentAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
                double x1 = -35 - (translation.get(1) / mmPerInch);
                double y1 = 60 - (translation.get(0) / mmPerInch);
                double degs = Math.atan2(y1, x1);
                degs = degs * 180 / 3.14159;
                if (gamepad1.b)
                {
                    // Posição: X2.0 Y77.0


                    double minimumY = 3, maximumY= -34;
                    if (translation.get(1) / mmPerInch < minimumY)
                    {
                        // Go to left
                        DriveMecanum(1, 0, 1, 0);
                    }
                    if (translation.get(1) / mmPerInch > maximumY)
                    {
                        // Go to right
                        DriveMecanum(1, 0, -1, 0);
                    }
                }
                else {
                }
                telemetry.addData("->", degs);
                telemetry.addData("Rot (deg)", currentAngle);
                telemetry.addData("Rot (deg)", "{Roll, Pitch, Heading} = %.0f, %.0f, %.0f", rotation.firstAngle, rotation.secondAngle, rotation.thirdAngle);
            }
            else {
                telemetry.addData("Visible Target", "none");
            }
            telemetry.update();
        }

        // Disable Tracking when we are done;
        targetsUltimateGoal.deactivate();
    }
}

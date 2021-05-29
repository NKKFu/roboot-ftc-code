package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

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
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

@TeleOp(name="RedSide", group="Main")
public class RedSide extends LinearOpMode {
    private static final String TFOD_MODEL_ASSET = "UltimateGoal.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Quad";
    private static final String LABEL_SECOND_ELEMENT = "Single";
    private static final String VUFORIA_KEY =
            "ATX1WMH/////AAABmZRCfAn9XENvtus7/8vEDzAjZUqBTtwyQRDZXSSxrKinG+QxvA51PBQf+MmhxVq5dnpIJ5pQcu8NIAo2ZJdWxJis8ws4Mx9efHxDgVdU4bwtKuOPmlUvdtwLHw8QZXpUA3nWj5G6HW2mX/7aE2hpj3sMsjjQKJMd7ify6hnYkVZwx3Ej1mODMH6FQ9UQTQuTJYN4vUGZQ3ggd/Yh+j1n+eldooxfN3G9SGh2eaa2vvfIsAsrZ7yrgHZUzw/fPDFa3MIk6lML0L02lKeksYGXqStHnzVJOiL/v2XNj+LmwpIvwWiijR4eTK3wl6oXjD0NP3i3bO37veiYrA0lTB+MZ18KpHQYiONQQ8GtTaP1eg3r";
    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;

    private VuforiaLocalizer initVuforia() {
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");
        return ClassFactory.getInstance().createVuforia(parameters);
    }

    private TFObjectDetector initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.8f;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
        return tfod;
    }

    BNO055IMU imu;
    private int AngleTarget = 0;
    private void RobotDrive(double rotations, double velocity) {
        rotations = rotations * 220;

        if (rotations < 0)
        {
            rotations *= -1;
            velocity *= -1;
        }

        double l = leftMotor.getCurrentPosition();
        double r = rightMotor.getCurrentPosition();

        while (Math.abs(leftMotor.getCurrentPosition() - l) < rotations && Math.abs(rightMotor.getCurrentPosition() - r) < rotations) {
            if (isStopRequested())
                break;

            // Absolute angle
            double currentAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;

            double error = currentAngle - AngleTarget;

            // Escala o erro
            error = error * 0.05;

            leftMotor.setPower(velocity + error);   //(velocity + diff);
            rightMotor.setPower(velocity - error); //(velocity - diff);
        }

        leftMotor.setPower(0);
        rightMotor.setPower(0);
    }

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

    DcMotor leftMotor, rightMotor, liftMotor;
    private Servo liftServ;

    @Override public void runOpMode() {
        double currentAngle = 0;
        // Init hardware stuff
        imu = InitImuSensor();
        leftMotor = InitDcMotor("left_drive", DcMotorSimple.Direction.REVERSE, DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightMotor = InitDcMotor("right_drive", DcMotorSimple.Direction.FORWARD, DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftMotor = InitDcMotor("lift_motor", DcMotorSimple.Direction.REVERSE, DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        liftServ = hardwareMap.get(Servo.class, "lift_serv");

        // Wait until we're told to go
        waitForStart();


   //     while (opModeIsActive()) {}







        RobotDrive(1.3, 0.7);

        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightMotor.setPower(-0.3);
        leftMotor.setPower(0.3);
        currentAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
        while (currentAngle > -30 ) {
            currentAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
        }
        leftMotor.setPower(0);
        rightMotor.setPower(0);

        // Verificar a quantidade de Argolas
        vuforia = initVuforia();
        tfod = initTfod();
        tfod.activate();
        sleep(2000);
        List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
        String label = "";
        if (updatedRecognitions != null) {
            int i = 0;
            for (Recognition recognition : updatedRecognitions) {
                label = recognition.getLabel();
                telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
            }
            telemetry.update();
        }
        tfod.shutdown();

        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightMotor.setPower(0.4);
        leftMotor.setPower(-0.4);
        while (currentAngle < 0 ) {
            currentAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
        }
        leftMotor.setPower(0);
        rightMotor.setPower(0);

        RobotDrive(5.2, 0.6);

        sleep(2000);

        switch (label) {
            case "":
                RobotDrive(0.6, 0.7);

                leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                rightMotor.setPower(0.3);
                leftMotor.setPower(-0.3);
                currentAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
                while (currentAngle < 90 ) {
                    currentAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
                }
                leftMotor.setPower(0);
                rightMotor.setPower(0);

                imu = InitImuSensor();
                RobotDrive(-2, 0.7);

                leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                rightMotor.setPower(0.5);
                leftMotor.setPower(-0.5);
                currentAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
                while (currentAngle < 20) {
                    currentAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
                }
                leftMotor.setPower(0);
                rightMotor.setPower(0);

                liftMotor.setPower(-0.5);
                sleep(700);
                liftMotor.setPower(0);
                sleep(700);
                liftServ.setPosition(0.5);

                liftMotor.setPower(0.5);
                sleep(700);
                liftMotor.setPower(0);

                break;
            case "Single":
                leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                rightMotor.setPower(-0.3);
                leftMotor.setPower(0.3);
                currentAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
                while (currentAngle > -44 ) {
                    currentAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
                }
                leftMotor.setPower(0);
                rightMotor.setPower(0);

                imu = InitImuSensor();
                RobotDrive(5, 0.7);

                liftMotor.setPower(-0.5);
                sleep(700);
                liftMotor.setPower(0);
                sleep(700);
                liftServ.setPosition(0.5);

                leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                leftMotor.setPower(0.4);
                rightMotor.setPower(-0.4);
                currentAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
                while (currentAngle > -134 ) {
                    currentAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
                }
                leftMotor.setPower(0);
                rightMotor.setPower(0);

                imu = InitImuSensor();
                RobotDrive(3.8, 1);

                while (opModeIsActive()) {}
                break;
            case "Quad":
                leftMotor.setPower(0);
                rightMotor.setPower(0);

                RobotDrive(2, 0.7);

                leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                rightMotor.setPower(-0.3);
                leftMotor.setPower(0.3);
                currentAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
                while (currentAngle > -44 ) {
                    currentAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
                }
                leftMotor.setPower(0);
                rightMotor.setPower(0);

                imu = InitImuSensor();
                RobotDrive(3, 0.7);

                leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                leftMotor.setPower(0.4);
                currentAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
                while (currentAngle > -134 ) {
                    currentAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
                }
                leftMotor.setPower(0);
                rightMotor.setPower(0);

                liftMotor.setPower(-0.5);
                sleep(700);
                liftMotor.setPower(0);
                liftServ.setPosition(0.5);

                imu = InitImuSensor();
                RobotDrive(3, 1);
                break;
        }
        while (opModeIsActive()) {}
    }
}
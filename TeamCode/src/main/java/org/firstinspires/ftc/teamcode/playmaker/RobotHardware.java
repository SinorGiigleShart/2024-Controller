package org.firstinspires.ftc.teamcode.playmaker;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareDevice;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.util.OmniDrive;

public abstract class RobotHardware extends OpMode {

    public static final String vuforiaKey = "ActI1F//////AAABmS42p5yOnkGis4OjI6bXOlAnHWRg28DHHDgR3ja8s8s9yCGhUmk3wfLPYxAOtfsiSVSi97uAosw46Pu3KQNf7fSqrMOT/PUcG2zW3Lq8tnJHTe/uwhwWgvnwOlrgEovZPA0uhwQ/uHH2zr/U2mFMYOQTTAk6ovbCjARxN+HfP6XWCDHDQ4dhOK+joRlA8u0HqXPzm6uBQWBgCyUno8aESPLQu3QGgEWUWm1tEhUny4rgQXC19nH160f7EGy+YoTR6YAD37xQQxnzP58wHmrX7+cBuiwkai9+g65R3pfBYprNpeRunzEml6m+a792ypI/niKew1VWPSgQSHaE1Ix8+c6uCvqySjcu5mZ1g3/pnU2j";
    public VuforiaLocalizer.Parameters vuforiaParameters;
    public VuforiaLocalizer vuforia;
    public Localizer localizer;
    public TFObjectDetector tfod;
    public OmniDrive omniDrive;
    public static double COUNTS_PER_INCH;
    public static double COUNTS_PER_LAT_INCH;
    public static double COUNTS_PER_DEGREE;
    public WebcamName webcamName;

    public boolean initVuforia = true;

    /**
     * All hardware should initialize sensors and stuff here
     */
    public abstract void initializeHardware();

    public void initializeAutonomous() {}

    public void initializeTeleOp() {}

    public <T extends HardwareDevice> T initializeDevice(Class<? extends T> deviceClass, String name) {
        try {
            return this.hardwareMap.get(deviceClass, name);
        } catch (Exception e) {
            this.telemetry.addLine(String.format("Err: Device \"%s\" cannot be found.", name));
            return null;
        }
    }

    /**
     * Initializes Vuforia. Largely copied from the the navigation example.
     */
    public void initializeVuforia() {
//        WebcamName webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
        this.webcamName = this.initializeDevice(WebcamName.class,"Webcam 1");
        if (this.webcamName == null) {
            telemetry.addData("Robot Hardware", "VUFORIA INIT FAILED, WEBCAM NULL");
            return;
        }
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = vuforiaKey;

        /**
         * We also indicate which camera on the RC we wish to use.
         */
        parameters.cameraName = webcamName;

        // Make sure extended tracking is disabled for this example.
        parameters.useExtendedTracking = false;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

    }

    public void initializeLocalizer() {
        this.localizer = new Localizer();
    }

    public void localize() {
        if (localizer != null) {
            this.localizer.updateLocationWithVuforia(this);
        }
    }

    @Override
    public void init() {
        this.initializeHardware();
        this.initializeLocalizer();
        if (initVuforia) initializeVuforia();
    }

    public void hardware_loop() {}

}
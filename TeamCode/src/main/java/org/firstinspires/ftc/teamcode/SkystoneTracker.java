package org.firstinspires.ftc.teamcode;

import com.vuforia.CameraDevice;
import com.vuforia.HINT;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
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

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;

public class SkystoneTracker {
    // We will define some constants and conversions here
    private static final float mmPerInch        = 25.4f;

    // Constant for Stone Target
    private static final float stoneZ = 2.00f * mmPerInch;

    // Variables to be used for later
    private VuforiaLocalizer vuforiaLocalizer;
    private VuforiaLocalizer.Parameters parameters;
    private VuforiaTrackables visionTargets;
    private VuforiaTrackable target;
    private VuforiaTrackableDefaultListener listener;

    private OpenGLMatrix lastKnownLocation;

    private boolean smooth = false;
    private ExponentialMovingAverage xSmooth;
    private ExponentialMovingAverage ySmooth;
    private ExponentialMovingAverage zSmooth;
    private ExponentialMovingAverage angleSmooth;

    private static final String VUFORIA_KEY =
            "Aau7tAP/////AAABmYcV2J3v00uhvod2qwfSvl11CHWQOos0Vv7o/4cXjJWODdYrxnH3ryRnpGm53SFGYFb7uC7X0ZYswprbG36uSQ6X0ltOW44hgnHt/bHy5Hj+cQOrUZvc921W6rGqdznkVsW2Rs14P8as7MAjg3zcXmRknLqtz2sys7vY84HdvhSVhkz2iGWlG4eZ18tf8gCmcJToytbQ8xUb0dpshqI5DS3xcWOfEdQjdhJFJzMg/4a1T3y0iI+rxg5gOG2ETORrsYiAp1teDQPrkJoGFSCrrTTDmJfrlhlzjE6eaw7u8HqsKKTnWqhQQZDEKVIWYgCfN8I+xUoq0fcAKEfR4go/+3Pk8C4cLASPlYbFVPI6vPLC";

    public SkystoneTracker(double alpha) {
        this.xSmooth = new ExponentialMovingAverage(alpha);
        this.ySmooth = new ExponentialMovingAverage(alpha);
        this.zSmooth = new ExponentialMovingAverage(alpha);
        this.angleSmooth = new ExponentialMovingAverage(alpha);
    }

    public void init()
    {
        setupVuforia();
        visionTargets.activate();
    }


    public Double getAngle() {
        if(lastKnownLocation != null) {
            return angleSmooth.getCurrent();
        }

        return null;
    }

    public Double getX() {
        if(lastKnownLocation != null) {
            return xSmooth.getCurrent();
        }

        return null;
    }

    public Double getY() {
        if(lastKnownLocation != null) {
            return ySmooth.getCurrent();
        }

        return null;
    }

    public Double getZ() {
        if(lastKnownLocation != null) {
            return zSmooth.getCurrent();
        }

        return null;
    }


    public void update()
    {
        if(listener.isVisible()) {
            // Ask the listener for the latest information on where the robot is
            OpenGLMatrix latestLocation = listener.getUpdatedRobotLocation();
            if(latestLocation != null) {
                lastKnownLocation = latestLocation;

                VectorF coords = lastKnownLocation.getTranslation();
                xSmooth.update(coords.get(0) / mmPerInch);
                ySmooth.update(coords.get(1) / mmPerInch);
                zSmooth.update(coords.get(2) / mmPerInch);
                float angle = Orientation.getOrientation(lastKnownLocation, AxesReference.EXTRINSIC,
                        AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle;
                angleSmooth.update(angle);
            }
        }
    }

    public boolean isVisible()
    {
        return listener.isVisible();
    }

    private void setupVuforia()
    {
        // Setup parameters to create localizer
        parameters = new VuforiaLocalizer.Parameters(R.id.cameraMonitorViewId); // To remove the camera view from the screen, remove the R.id.cameraMonitorViewId
        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        parameters.useExtendedTracking = false;
        vuforiaLocalizer = ClassFactory.getInstance().createVuforia(parameters);
        CameraDevice.getInstance().setFlashTorchMode( true );

        // These are the vision targets that we want to use
        // The string needs to be the name of the appropriate .xml file in the assets folder
        visionTargets = vuforiaLocalizer.loadTrackablesFromAsset("Skystone");
        Vuforia.setHint(HINT.HINT_MAX_SIMULTANEOUS_IMAGE_TARGETS, 4);

        // Setup the target to be tracked
        target = visionTargets.get(0); // 0 is the skystone label
        target.setName("Skystone Target");
        target.setLocation(createMatrix(0, 0, stoneZ, 90, 0, 0));

        // Next, translate the camera lens to where it is on the robot.
        // In this example, it is centered (left to right), but forward of the middle of the robot, and above ground level.
        final float CAMERA_FORWARD_DISPLACEMENT  = 4.0f * mmPerInch;   // eg: Camera is 4 Inches in front of robot center
        final float CAMERA_VERTICAL_DISPLACEMENT = 8.0f * mmPerInch;   // eg: Camera is 8 Inches above ground
        final float CAMERA_LEFT_DISPLACEMENT     = 0;     // eg: Camera is ON the robot's center line

//        OpenGLMatrix robotFromCamera = OpenGLMatrix
//                .translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT)
//                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, YZX, DEGREES, -90, 0, 90));

        OpenGLMatrix robotFromCamera = OpenGLMatrix
                .translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, YZX, DEGREES, -90, 0, 0));
        // Setup listener and inform it of phone information
        listener = (VuforiaTrackableDefaultListener) target.getListener();
        listener.setPhoneInformation(robotFromCamera, parameters.cameraDirection);
    }

    // Creates a matrix for determining the locations and orientations of objects
    // Units are millimeters for x, y, and z, and degrees for u, v, and w
    private OpenGLMatrix createMatrix(float x, float y, float z, float u, float v, float w)
    {
        return OpenGLMatrix.translation(x, y, z).
                multiplied(Orientation.getRotationMatrix(
                        AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES, u, v, w));
    }
}

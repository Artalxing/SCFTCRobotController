package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {
    public static FollowerConstants followerConstants = new FollowerConstants().mass(0); //TODO Change this with the mass in kgs,

    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .rightFrontMotorName("frontLeft") //TODO Make sure names are correct!
            .rightRearMotorName("backRight") //TODO Make sure names are correct!
            .leftRearMotorName("backLeft") //TODO Make sure names are correct!
            .leftFrontMotorName("frontLeft") //TODO Make sure names are correct!
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE) //TODO Change this
            .leftRearMotorDirection(DcMotorSimple.Direction.FORWARD) //TODO Change this
            .rightFrontMotorDirection(DcMotorSimple.Direction.REVERSE) //TODO Change this
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD); //TODO Change this

    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(0) //TODO Change this to 0 and use the offset tuner to tune the offset
            .strafePodX(0) //TODO Change this to 0 and use the offset tuner to tune the offset
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName("odometry")
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD);
    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .pinpointLocalizer(localizerConstants)
                .build();
    }
}

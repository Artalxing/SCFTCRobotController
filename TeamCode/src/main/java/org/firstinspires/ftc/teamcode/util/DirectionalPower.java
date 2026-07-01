package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class DirectionalPower {
    //Motor
    DcMotor frontLeft;
    DcMotor frontRight;
    DcMotor backLeft;
    DcMotor backRight;

    public DirectionalPower(DcMotor fl,  DcMotor fr, DcMotor bl, DcMotor br) {
        //Init
        frontLeft = fl;
        frontRight = fr;
        backLeft = bl;
        backRight = br;
    }

    public void drive(Vector2 dir, double power, double rotation, Telemetry telemetry, boolean displayTelemetry) {
        //Normalize dir
        dir.normalize();
        //Multiply by power
        //Clamp power
        power = Range.clip(power, -1, 1);
        dir.multiply(power);
        rotation *= power;
        //Clamp rotation
        rotation = Range.clip(rotation, -1, 1);
        //Apply the directions
        double frontLeftPower = dir.y + dir.x + rotation;
        double frontRightPower = dir.y - dir.x - rotation;
        double backLeftPower = dir.y - dir.x + rotation;
        double backRightPower = dir.y + dir.x - rotation;
        // Normalize the values so no wheel power exceeds 100%
        // This ensures that the robot maintains the desired motion.
        double max = 1;
        max = Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower));
        max = Math.max(max, Math.abs(backLeftPower));
        max = Math.max(max, Math.abs(backRightPower));

        if (max > 1.0) {
            frontLeftPower /= max;
            frontRightPower /= max;
            backLeftPower /= max;
            backRightPower /= max;
        }

        //Apply power
        frontLeft.setPower(frontLeftPower);
        frontRight.setPower(frontRightPower);
        backLeft.setPower(backLeftPower);
        backRight.setPower(backRightPower);

        //Debug
        if(displayTelemetry){
            telemetry.addData("rotation", rotation);
            telemetry.addData("frontLeftPower", frontLeftPower);
            telemetry.addData("frontRightPower", frontRightPower);
            telemetry.addData("backLeftPower", backLeftPower);
            telemetry.addData("backRightPower", backRightPower);
        }
    }

    public void debugTelemetry(Telemetry telemetry){
        telemetry.addData("Front Left Power", frontLeft.getPower());
        telemetry.addData("Front Right Power", frontRight.getPower());
        telemetry.addData("Back Left Power", backLeft.getPower());
        telemetry.addData("Back Right Power", backRight.getPower());
    }
}

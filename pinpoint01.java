package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.checkerframework.checker.units.qual.Speed;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.GoBildaPinpointDriver;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;


import java.util.List;

@Autonomous(name = "pinpoint01")
public class pinpoint01 extends LinearOpMode {
    GoBildaPinpointDriver odo; // Declare OpMode member for the Odometry Computer

    double oldTime = 0;
    private static final boolean USE_WEBCAM = true;
    private AprilTagProcessor aprilTag;

    private DcMotor MotorA;
    private DcMotor MotorB;
    private DcMotor MotorC;
    private DcMotor MotorD;
    private DcMotor MotorF;
    double Turn_odo;
    //double Delta_Turn_odo;
    double x_odo;
    double y_odo;
    //double B_odo;
    //double C_odo;
    //double D_odo;
    //double last_turn = 0;



    /**
     * This function is executed when this OpMode is selected from the Driver Station.
     */
    @Override
    public void runOpMode() {
        MotorA = hardwareMap.get(DcMotor.class, "MotorA");
        MotorB = hardwareMap.get(DcMotor.class, "MotorB");
        MotorC = hardwareMap.get(DcMotor.class, "MotorC");
        MotorD = hardwareMap.get(DcMotor.class, "MotorD");
        MotorF = hardwareMap.get(DcMotor.class, "MotorF");
        /**ตั้งค่า odo */
        odo = hardwareMap.get(GoBildaPinpointDriver.class,"odo");
        odo.setEncoderResolution(13.26291192);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);
        odo.setOffsets(-84.0, -168.0);

        initAprilTag();
        // Put initialization blocks here.
        odo.resetPosAndIMU();
        waitForStart();
        resetRuntime();
        MotorA.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        MotorB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        MotorC.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        MotorD.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        MotorA.setDirection(DcMotor.Direction.REVERSE);
        MotorC.setDirection(DcMotor.Direction.REVERSE);
        MotorA.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        MotorB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        MotorC.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        MotorD.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        if (opModeIsActive()) {
            x_odo = 0;
            y_odo = 0;
            //B_odo = 0;
            //C_odo = 0;
            //D_odo = 0;
            Turn_odo = 0;
            // Put run blocks here.
            move2(0.8, 0.01, 0,0.25,0, 10000, 0, 0);

        }
    }

    /**
     * Describe this function...
     */
    public void move2(double Speed, double kp_Turn, double ki_Turn,  double kd_Turn, int y_positions, int x_positions, int Turn, int intake) {
        double movement;
        double error_sum;
        ElapsedTime myElapsedTime;
        ElapsedTime Time;
        double lastT = 0;
        double lastTime = 0;
        double currentTime = 0;
        double deltaTime = 0;
        double deltaTimeSquared = 0;
        double degree = 0;
        double previous_odoturn = 0;
        double error_turn = 0;
        double errorRate = 0;
        double dt = 0;
        double str = 0;
        double forw = 0;
        double avgTurn = 0;
        double deltaB = 0;
        double deltaC = 0;
        double deltaD = 0;
        double velocityB = 0;
        double velocityC = 0;
        double velocityD = 0;
        double lastVelocityB = 0;
        double lastVelocityC = 0;
        double lastVelocityD = 0;
        double accelerationB = 0;
        double accelerationC = 0;
        double accelerationD = 0;
        double alpha = 0.1;
        double radiusforw;
        double radiusstr;
        double relX;
        double relY;

        MotorA.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        MotorB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        MotorC.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        MotorD.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //y_positions = y_positions * -1;
        lastT = 0;
        previous_odoturn = 0;
        myElapsedTime = new ElapsedTime();
        Time = new ElapsedTime();
        error_sum = 0;
        myElapsedTime.reset();
        Time.reset();
        while (myElapsedTime.seconds() < 30) {
            odo.update();
            double newTime = getRuntime();
            double loopTime = newTime-oldTime;
            double frequency = 1/loopTime;
            currentTime = Time.seconds();
            deltaTime = (currentTime - lastTime);  //deltaTime can be 0? not sure about that, let's try!!! if it is then, good luck
            telemetry.addData("x", x_odo);
            telemetry.addData("y", y_odo);
            telemetry.addData("turn", Turn_odo);
            telemetry.update();
//            telemetry.addData("b_current", MotorB.getCurrentPosition());
//            telemetry.addData("c_current", MotorC.getCurrentPosition());
//            telemetry.addData("TurnBrush",(Turn_odo / 180.0 * Math.PI));
//            telemetry.addData("sin_Turn", Math.sin((Turn_odo / 180.0 * Math.PI)));
//            telemetry.addData("cos_Turn", Math.cos((Turn_odo / 180.0 * Math.PI)));
//            telemetry.addData("Delta_Turn",Delta_Turn_odo);
//            telemetry.addData("Degree", degree);


            //deltaB = -MotorB.getCurrentPosition() - B_odo;
            //deltaC = -MotorC.getCurrentPosition() - C_odo;
            //deltaD = -MotorD.getCurrentPosition() - D_odo;

            //velocityB = deltaB / deltaTime;   //since derivative term of s(t) is equal to v=s/t
            //velocityC = deltaC / deltaTime;   //so i use the formula instead of diff
            //velocityD = deltaD / deltaTime;

//            velocityB = alpha * (deltaB / deltaTime) + (1 - alpha) * lastVelocityB;  //utilize Alpha parameter; controls the amount of smoothing
//            velocityC = alpha * (deltaC / deltaTime) + (1 - alpha) * lastVelocityC;  //not sure about this yet
//            velocityD = alpha * (deltaD / deltaTime) + (1 - alpha) * lastVelocityD;  //maybe more accurate?

            //accelerationB = (velocityB - lastVelocityB) / deltaTime;  //I dont know how to use this thing now
            //accelerationC = (velocityC - lastVelocityC) / deltaTime;  //Just leave it here
            //accelerationD = (velocityD - lastVelocityD) / deltaTime;  //Maybe future?         update: figure it now:D

            //Turn_odo = ((-MotorB.getCurrentPosition()) - (-MotorC.getCurrentPosition()))/ 54;
            //Delta_Turn_odo = Turn_odo - last_turn;
            //avgTurn = (Turn_odo + last_turn) / 2.0;  //Minimizing Error Due to Rapid Changes in Orientation

            //forw = ((velocityB + velocityC) / 2.0) * deltaTime;
            //str = (velocityD * deltaTime - (33.0 * (Delta_Turn_odo)));

            //deltaTimeSquared = deltaTime * deltaTime; //not use yet.... maybe?

//            if (Delta_Turn_odo != 0) {
//                radiusforw = forw / Delta_Turn_odo;
//                radiusstr = str / Delta_Turn_odo;
//
//                relX = radiusforw * Math.sin(Delta_Turn_odo / 180.0 * Math.PI) - radiusstr * (1 - Math.cos(Delta_Turn_odo / 180.0 * Math.PI));  //not sure about this
//                relY = radiusstr * Math.sin(Delta_Turn_odo / 180.0 * Math.PI) + radiusforw * (1 - Math.cos(Delta_Turn_odo / 180.0 * Math.PI));
//
//                x_odo += relX * Math.cos(avgTurn / 180.0 * Math.PI) + (relY * Math.sin(avgTurn / 180.0 * Math.PI));
//                y_odo += relY * Math.cos(avgTurn / 180.0 * Math.PI) - (relX * Math.sin(avgTurn / 180.0 * Math.PI));
//            }
//            else {
//
//                x_odo += (forw * Math.cos(avgTurn / 180.0 * Math.PI)) + ((str * Math.sin(avgTurn / 180.0 * Math.PI)));
//                y_odo += ((str * Math.cos(avgTurn / 180.0 * Math.PI))) - (forw * Math.sin(avgTurn / 180.0 * Math.PI));
//            }

//            x_odo += 0.5 * (accelerationB + accelerationC) * deltaTimeSquared * Math.cos(avgTurn / 180.0 * Math.PI);   //from s = ut + "1/2 at^2"
//            y_odo += 0.5 * (accelerationB + accelerationC) * deltaTimeSquared * Math.sin(avgTurn / 180.0 * Math.PI);   //due to the robot's unconstant acceleration, this might not work well

            x_odo = pos.getX(DistanceUnit.MM);
            y_odo = pos.getY(DistanceUnit.MM);
            Turn_odo = pos.getHeading(AngleUnit.DEGREES));

            error_turn = Turn - Turn_odo;
            errorRate = (error_turn - previous_odoturn) / dt;
            dt = myElapsedTime.milliseconds() - lastT;
            movement = (error_turn * kp_Turn) + (errorRate * kd_Turn) + (error_sum * ki_Turn);

            degree = Math.atan2(x_positions - x_odo, y_positions - y_odo)  - (Turn_odo / 180.0 * Math.PI) ;
            degree = (degree - Math.PI/4.0);

            if ( intake == 1 ) {
                MotorF.setPower(1);
            }
            if (y_positions - y_odo >= 2000 || y_positions - y_odo <= -2000 || x_positions - x_odo >= 2000 || x_positions - x_odo <= -2000 || Math.abs(Turn - Turn_odo) >= 4) {
                telemetry.update();
                MotorA.setPower(Math.min(Math.max((Math.sin(degree) * Speed + (movement)) , -Speed), Speed) );
                MotorB.setPower(Math.min(Math.max((Math.cos(degree) * Speed - (movement)) , -Speed), Speed) );
                MotorC.setPower(Math.min(Math.max((Math.cos(degree) * Speed + (movement)) , -Speed), Speed) );
                MotorD.setPower(Math.min(Math.max((Math.sin(degree) * Speed - (movement)) , -Speed), Speed) );
            }
            else if (y_positions - y_odo >= 130 || y_positions - y_odo <= -130 || x_positions - x_odo >= 130 || x_positions - x_odo <= -130 || Math.abs(Turn - Turn_odo) >= 1) {
                telemetry.update();
                MotorA.setPower(Math.min(Math.max((Math.sin(degree) * 0.2 + (movement)) , -0.2), 0.2) );
                MotorB.setPower(Math.min(Math.max((Math.cos(degree) * 0.2 - (movement)) , -0.2), 0.2) );
                MotorC.setPower(Math.min(Math.max((Math.cos(degree) * 0.2 + (movement)) , -0.2), 0.2) );
                MotorD.setPower(Math.min(Math.max((Math.sin(degree) * 0.2 - (movement)) , -0.2), 0.2) );
            }
            else {
                MotorA.setPower(0);
                MotorB.setPower(0);
                MotorC.setPower(0);
                MotorD.setPower(0);
                MotorF.setPower(0);
                MotorA.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                MotorB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                MotorC.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                MotorD.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                MotorF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                myElapsedTime.reset();
                Time.reset();
                break;
            }

            lastVelocityB = velocityB;
            lastVelocityC = velocityC;
            lastVelocityD = velocityD;

//            B_odo = -MotorB.getCurrentPosition();
//            C_odo = -MotorC.getCurrentPosition();
//            D_odo = -MotorD.getCurrentPosition();
            previous_odoturn = error_turn;
            lastT = myElapsedTime.milliseconds();
            lastTime = Time.seconds();
            oldTime = newTime;
//            last_turn = Turn_odo ;
        }
    }

    private void initAprilTag() {

        // Create the AprilTag processor the easy way.
        aprilTag = AprilTagProcessor.easyCreateWithDefaults();

        // Create the vision portal the easy way.
        //if (USE_WEBCAM) {
        //visionPortal = VisionPortal.easyCreateWithDefaults(
        //hardwareMap.get(WebcamName.class, "Webcam 1"), aprilTag);
        //} else {
        //visionPortal = VisionPortal.easyCreateWithDefaults(
        //BuiltinCameraDirection.BACK, aprilTag);
        //}

    }

    private void telemetryAprilTag() {

        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        telemetry.addData("# AprilTags Detected", currentDetections.size());

        // Step through the list of detections and display info for each one.
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
                telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));

            } else {
                telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
            }

        }   // end for() loop

        // Add "key" information to telemetry
        telemetry.addLine("\nkey:\nXYZ = X (Right), Y (Forward), Z (Up) dist.");
        telemetry.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)");
        telemetry.addLine("RBE = Range, Bearing & Elevation");


    }   // end method telemetryAprilTag()

    private void april(){
        boolean a = true;
        while (opModeIsActive()) {
            while(a == true) {


                //telemetryAprilTag();
                // Push telemetry to the Driver Station.
                telemetry.update();


                List<AprilTagDetection> currentDetections = aprilTag.getDetections();
                telemetry.addData("# AprilTags Detected", currentDetections.size());
                for (AprilTagDetection detection : currentDetections) {

                    // Step through the list of detections and display info for each one.
                    if (detection.metadata != null) {
                        if(detection.id == 4){
                            MotorA.setPower(-0.15);
                            MotorB.setPower(0.15);
                            MotorC.setPower(0.15);
                            MotorD.setPower(-0.15);
                            sleep(300);
                        }
                        else if(detection.id == 5){
                            MotorA.setPower(0);
                            MotorB.setPower(0);
                            MotorC.setPower(0);
                            MotorD.setPower(0);
                            MotorA.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                            MotorB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                            MotorC.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                            MotorD.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                            a = false;

                        }
                        else {
                            MotorA.setPower(-0.15);
                            MotorB.setPower(0.15);
                            MotorC.setPower(0.15);
                            MotorD.setPower(-0.15);
                            sleep(300);


                        }

                    } else {
                        MotorA.setPower(-0.2);
                        MotorB.setPower(0.2);
                        MotorC.setPower(0.2);
                        MotorD.setPower(-0.2);
                        sleep(500);

                    }
                    if (detection.metadata != null) {
                        telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                        telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                        telemetry.update();
                    } else {
                        telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                        telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
                    }


                }

                // Save CPU resources; can resume streaming when needed.

            }
            // Share the CPU.

        }
    }
}
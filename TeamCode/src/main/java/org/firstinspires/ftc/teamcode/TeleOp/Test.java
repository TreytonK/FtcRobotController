package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;


public class Test extends LinearOpMode {
    private DcMotor leftDrive;
    private DcMotor rightDrive;
    private DcMotor arm;
    private Servo claw;
    private CRServo intake;
    private DcMotor wrist;

    boolean USE_WEBCAM;
    AprilTagProcessor myAprilTagProcessor;
    VisionPortal myVisionPortal;

    double lastKnownX;
    double lastKnownY;
    int lastTag;

    List<AprilTagDetection> myAprilTagDetections;
    AprilTagDetection myAprilTagDetection;

    @Override



    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        USE_WEBCAM = true;
        // Initialize AprilTag before waitForStart.
        initAprilTag();
        // Wait for the match to begin.
        telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch START to start OpMode");
        telemetry.update();

        leftDrive  = hardwareMap.get(DcMotor.class, "leftDrive");
        rightDrive = hardwareMap.get(DcMotor.class, "rightDrive");
        arm = hardwareMap.get(DcMotor.class, "arm");
        claw = hardwareMap.get(Servo.class, "claw");
        intake = hardwareMap.get(CRServo.class, "intake");
        wrist = hardwareMap.get(DcMotor.class,"wrist");

        rightDrive.setDirection(DcMotor.Direction.REVERSE);
        leftDrive.setDirection(DcMotor.Direction.FORWARD);

        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wrist.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wrist.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        wrist.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);



        double closed = 0.6;
        double open = 0;
        claw.scaleRange(open,closed);

        //TAGS
        int tag11x = -48;
        int tag11y = 72;

        int tag12x = 0;
        int tag12y = 72;

        int tag13x = 48;
        int tag13y = 72;

        int tag14x = 48;
        int tag14y = -72;

        int tag15x = 0;
        int tag15y = -72;

        int tag16x = -48;
        int tag16y = -72;

        double robotX;
        double robotY;

        telemetry.setMsTransmissionInterval(1);

        waitForStart();


        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            //xy cords test
            if(lastKnownX != 0 && lastKnownY != 0){
                if(lastTag == 11){
                    robotX = tag11x + lastKnownX;
                    robotY = tag11y - lastKnownY;
                    telemetry.addData("X: " , robotX);
                    telemetry.addData("Y: " , robotY);
                }
                if(lastTag == 12){
                    robotX = tag12x + lastKnownX;
                    robotY = tag12y - lastKnownY;
                    telemetry.addData("X: " , robotX);
                    telemetry.addData("Y: " , robotY);
                }
                if(lastTag == 13){
                    robotX = tag13x + lastKnownX;
                    robotY = tag13y - lastKnownY;
                    telemetry.addData("X: " , robotX);
                    telemetry.addData("Y: " , robotY);
                }
                if(lastTag == 14){
                    robotX = lastKnownX + tag14x;
                    robotY = lastKnownY + tag14y;
                    telemetry.addData("X: " , robotX);
                    telemetry.addData("Y: " , robotY);
                }
                if (lastTag == 15){
                    robotX = lastKnownX + tag15x;
                    robotY = lastKnownY + tag15y;
                    telemetry.addData("X: " , robotX);
                    telemetry.addData("Y: " , robotY);
                }
                if(lastTag == 16){
                    robotX = lastKnownX + tag16x;
                    robotY = lastKnownY + tag16y;
                    telemetry.addData("X: " , robotX);
                    telemetry.addData("Y: " , robotY);
                }
            }else{
                telemetry.addLine("Position Unknown:");
            }





            telemetryAprilTag();
            // Push telemetry to the Driver Station.
            telemetry.update();
            if (gamepad1.left_trigger == 1) {
                // Temporarily stop the streaming session. This can save CPU
                // resources, with the ability to resume quickly when needed.
                myVisionPortal.stopStreaming();
            } else if (gamepad1.x) {
                // Resume the streaming session if previously stopped.
                myVisionPortal.resumeStreaming();
            }
            // Share the CPU.
            double fPower = 0;
            double rPower = 0;
            double trigger = 0;
            /*telemetry.addData("Left Pos: ", leftDrive.getCurrentPosition());
            telemetry.addData("Right Pos: ", rightDrive.getCurrentPosition());
            telemetry.addData("Arm Pos: ", arm.getCurrentPosition());
            telemetry.addData("wrist Pos: ", wrist.getCurrentPosition());
            telemetry.addData("Forward Power: " , fPower);
            telemetry.addData("Reverse Power: " , rPower);
            telemetry.addData("Trigger: " , trigger);*/
            telemetry.update();

            /*if(gamepad1.dpad_left){
                bucket();
                while(gamepad1.b != true){
                    intake.setPower(0);
                }
                intake.setPower(-0.5);
                sleep(1000);
                intake.setPower(0);
                retractIntake();
            }*/
            if (gamepad1.y){
                claw.setPosition(0.7                                                                                                                                                                                                                                                                                                                                                                                                                                                                            );
            }
            if (gamepad1.dpad_up){
                Top();
            }
            if (gamepad1.dpad_right){
                Mid();

            }
            if(gamepad1.right_trigger > 0){
                trigger = gamepad1.right_trigger;
                fPower = ((trigger / 2) - (gamepad1.left_stick_y / 2));
                if (fPower == 0){
                    fPower = 1;
                }
                rPower = ((trigger / 2) + (gamepad1.left_stick_y / 2));
                if (rPower == 0){
                    rPower = 1;
                }
                if(fPower < 0){
                    fPower = fPower * -1;
                }
                if(rPower > 0){
                    rPower = rPower * -1;
                }

                //Forward coasting right
                if(gamepad1.right_trigger > 0 && gamepad1.right_stick_x > 0){
                    rightDrive.setPower(fPower - gamepad1.right_stick_x);
                    leftDrive.setPower(fPower);
                }
                if(gamepad1.right_trigger > 0 && gamepad1.right_stick_x < 0){
                    rightDrive.setPower(fPower);
                    leftDrive.setPower(fPower + gamepad1.right_stick_x);
                }
                //forward
                else if(gamepad1.left_stick_y < 0){

                    telemetry.update();
                    leftDrive.setPower(fPower);
                    rightDrive.setPower(fPower);
                    //reverse
                }else if(gamepad1.left_stick_y > 0){

                    telemetry.update();
                    leftDrive.setPower(rPower);
                    rightDrive.setPower(rPower);
                    //left 0 turn
                }else if(gamepad1.right_stick_x < 0){
                    leftDrive.setPower(-0.3);
                    rightDrive.setPower(0.3);
                    //right 0 turn
                }else if(gamepad1.right_stick_x > 0){
                    leftDrive.setPower(0.3);
                    rightDrive.setPower(-0.3);

                }else{
                    leftDrive.setPower(0);
                    rightDrive.setPower(0);
                }
            }else{
                leftDrive.setPower(0);
                rightDrive.setPower(0);
            }
            if(gamepad1.dpad_down){
                deployIntake();

                while(!gamepad1.b){
                    intake.setPower(1);
                    activateDriver();
                }
                intake.setPower(0);
                retractIntake();
            }
        }
    }
    public void Top(){
        ((DcMotorEx) arm).setTargetPositionTolerance(5);
        int armPos = 800;
        arm.setTargetPosition(armPos);

        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while(arm.getCurrentPosition() != arm.getTargetPosition()){
            arm.setPower(0.3);
            telemetry.update();
        }
        while(!gamepad1.b){
            arm.setPower(0.2);
            activateDriver();
            if(gamepad1.dpad_right){
                Mid();
                break;
            }
        }
        if(gamepad1.b){
            score();
        }
        telemetry.update();
    }
    public void Mid(){
        ((DcMotorEx) arm).setTargetPositionTolerance(5);
        int armPos = 400;
        arm.setTargetPosition(armPos);

        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        while(!gamepad1.b){
            arm.setPower(0.2);
            activateDriver();
            if(gamepad1.dpad_up){
                Top();
                break;
            }
        }
        if(gamepad1.b){
            score();
        }
        arm.setPower(0);
        telemetry.update();

    }
    public void score(){
        telemetry.addLine("Called Score()");


        double open = 0.1;

        claw.setPosition(open);
        telemetry.addLine("Calling Motors");
        Drive(0.2, 0.2, -100, -100);

        arm.setPower(0);

        telemetry.update();
    }
    public void Drive(double Left_speed, double Right_speed, int leftPos, int rightPos) {
        ((DcMotorEx) rightDrive).setTargetPositionTolerance(10);
        ((DcMotorEx) leftDrive).setTargetPositionTolerance(10);
        rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftDrive.setTargetPosition(leftPos);
        rightDrive.setTargetPosition(rightPos);
        leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        while (leftDrive.getCurrentPosition() != leftPos && rightDrive.getCurrentPosition() != rightPos) {
            if(rightDrive.getCurrentPosition() > rightPos && leftDrive.getCurrentPosition() > leftPos){
                rightDrive.setPower(-Right_speed);
                leftDrive.setPower(-Left_speed);
            }else{
                rightDrive.setPower(Right_speed);
                leftDrive.setPower(Left_speed);
            }
            telemetry.update();
            telemetry.addLine("Driving Motors");

        }
        telemetry.addLine("Stopping Motors");
        telemetry.update();
        rightDrive.setPower(0);
        leftDrive.setPower(0);
        rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    public void shooter(){
        while(!gamepad1.start){
            if(gamepad1.right_trigger == 1){
                wrist.setPower(0.5);
            }else{
                wrist.setPower(0);
            }
        }
        while(!gamepad1.start){
            if(gamepad1.right_trigger == 1){
                wrist.setPower(-0.5);
            }else{
                wrist.setPower(0);
            }
        }
    }

    public void deployIntake(){
        wrist.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        int wristPos = 217;
        wrist.setTargetPosition(wristPos);
        wrist.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while (wrist.getCurrentPosition() != wrist.getTargetPosition()){
            if(wrist.getCurrentPosition() > wrist.getTargetPosition()){
                wrist.setPower(-0.2);
            }else{
                wrist.setPower(0.2);
            }
            telemetry.addData("Wrist Pos: ", wrist.getCurrentPosition());
        }
        telemetry.update();
    }
    public void retractIntake(){
        int wristPos = 0;
        wrist.setTargetPosition(wristPos);
        wrist.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while (wrist.getCurrentPosition() != wrist.getTargetPosition()){
            wrist.setPower(-0.2);
            telemetry.addData("Wrist Pos: ", wrist.getCurrentPosition());
        }
        telemetry.update();
    }
    public void activateDriver(){
        if(gamepad1.left_stick_y < 0){
            leftDrive.setPower(0.2);
            rightDrive.setPower(0.2);
        }else if(gamepad1.left_stick_y > 0){
            leftDrive.setPower(-0.2);
            rightDrive.setPower(-0.2);
        }else if(gamepad1.right_stick_x > 0){
            leftDrive.setPower(0.2);
            rightDrive.setPower(-0.2);
        }else if(gamepad1.right_stick_x < 0){
            leftDrive.setPower(-0.2);
            rightDrive.setPower(0.2);
        }else{
            leftDrive.setPower(0);
            rightDrive.setPower(0);
        }
    }




    public void bucket(){
        ((DcMotorEx) arm).setTargetPositionTolerance(10);
        int armPos = 766;
        arm.setTargetPosition(armPos);


        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while(arm.getCurrentPosition() != arm.getTargetPosition()){
            arm.setPower(0.3);
            telemetry.addData("Arm Pos: ", arm.getCurrentPosition());
            telemetry.update();
        }
        deployIntake();


        while(!gamepad1.b){
            arm.setPower(0.2);

            activateDriver();

        }
        score();
    }
    private void initAprilTag() {
        AprilTagProcessor.Builder myAprilTagProcessorBuilder;
        VisionPortal.Builder myVisionPortalBuilder;

        // First, create an AprilTagProcessor.Builder.
        myAprilTagProcessorBuilder = new AprilTagProcessor.Builder();
        // Create an AprilTagProcessor by calling build.
        myAprilTagProcessor = myAprilTagProcessorBuilder.build();
        // Next, create a VisionPortal.Builder and set attributes related to the camera.
        myVisionPortalBuilder = new VisionPortal.Builder();
        if (USE_WEBCAM) {
            // Use a webcam.
            myVisionPortalBuilder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        } else {
            // Use the device's back camera.
            myVisionPortalBuilder.setCamera(BuiltinCameraDirection.BACK);
        }
        // Add myAprilTagProcessor to the VisionPortal.Builder.
        myVisionPortalBuilder.addProcessor(myAprilTagProcessor);
        // Create a VisionPortal by calling build.
        myVisionPortal = myVisionPortalBuilder.build();
    }

    /**
     * Display info (using telemetry) for a recognized AprilTag.
     */
    private void telemetryAprilTag() {
        List<AprilTagDetection> myAprilTagDetections;
        AprilTagDetection myAprilTagDetection;

        // Get a list of AprilTag detections.
        myAprilTagDetections = myAprilTagProcessor.getDetections();
        telemetry.addData("# AprilTags Detected", JavaUtil.listLength(myAprilTagDetections));
        // Iterate through list and call a function to display info for each recognized AprilTag.
        for (AprilTagDetection myAprilTagDetection_item : myAprilTagDetections) {
            myAprilTagDetection = myAprilTagDetection_item;
            // Display info about the detection.
            telemetry.addLine("");
        /*if (myAprilTagDetection.metadata != null) {
            telemetry.addLine("==== (ID " + myAprilTagDetection.id + ") " + myAprilTagDetection.metadata.name);
            telemetry.addLine("XYZ " + JavaUtil.formatNumber(myAprilTagDetection.ftcPose.x, 6, 1) + " " + JavaUtil.formatNumber(myAprilTagDetection.ftcPose.y, 6, 1) + " " + JavaUtil.formatNumber(myAprilTagDetection.ftcPose.z, 6, 1) + "  (inch)");
            telemetry.addLine("PRY " + JavaUtil.formatNumber(myAprilTagDetection.ftcPose.pitch, 6, 1) + " " + JavaUtil.formatNumber(myAprilTagDetection.ftcPose.roll, 6, 1) + " " + JavaUtil.formatNumber(myAprilTagDetection.ftcPose.yaw, 6, 1) + "  (deg)");
            telemetry.addLine("RBE " + JavaUtil.formatNumber(myAprilTagDetection.ftcPose.range, 6, 1) + " " + JavaUtil.formatNumber(myAprilTagDetection.ftcPose.bearing, 6, 1) + " " + JavaUtil.formatNumber(myAprilTagDetection.ftcPose.elevation, 6, 1) + "  (inch, deg, deg)");
        } else {
            telemetry.addLine("==== (ID " + myAprilTagDetection.id + ") Unknown");
            telemetry.addLine("Center " + JavaUtil.formatNumber(myAprilTagDetection.center.x, 6, 0) + "" + JavaUtil.formatNumber(myAprilTagDetection.center.y, 6, 0) + " (pixels)");
        }*/
            lastKnownX = myAprilTagDetection.ftcPose.x;
            lastKnownY = myAprilTagDetection.ftcPose.y;
            lastTag = myAprilTagDetection.id;
        }
    }
}

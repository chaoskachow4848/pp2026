package org.firstinspires.ftc.teamcode.driver;/*package org.firstinspires.ftc.teamcode.driver;

import static org.firstinspires.ftc.teamcode.hardware.SampleHardware.state;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.hardware.SampleHardware;
import org.firstinspires.ftc.teamcode.hardware.button;
import org.firstinspires.ftc.teamcode.hardware.kaze;


@TeleOp(name="Blakes's Teleop", group="4848")
@Disabled
public class BlakesTeleop extends RobotCentricDriverSample {

    @Override public void runOpMode() {


        boolean isMoving = false;

        button wasPressed = null;

        robot.init(hardwareMap);
        gamepad2.runLedEffect(robot.redled);
        gamepad1.runLedEffect(robot.blueled);
        telemetry.update();
        robot.leds.setPattern(robot.pattern);

        waitForStart();
        robot.runtime.reset();//NEW//////////////////////////////////////////////////////////////////////////////////

        while (opModeIsActive()) {
            update();
            telemetry.addData("magnet: ", robot.magnet.getState());
            telemetry.addData("magnet button release: ", gamePad2.justMagnet.wasJustReleased());


            switch (State){

                case driving:
                    isReady = false;
                    updateDrive(DriveSpeed);

                        //home all
                    if(robot.stateTime.seconds()<.6){
                        robot.shoulder.setPosition(clawShoulderBack);
                        if(robot.lastState == state.outTaking){
                            robot.bucket.setPosition(bucketIntakepose);
                            robot.slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                            robot.slide.setTargetPosition(slideBuffer);
                            robot.slide.setPower(1);
                            robot.slide2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                            robot.slide2.setTargetPosition(slideBuffer);
                            robot.slide2.setPower(1);
                            if(!clipping) {
                                robot.IntakePivotLeft.setPosition(intakePivotBuffer);
                                robot.IntakePivotRight.setPosition(intakePivotBuffer);
                            } else {
                                robot.IntakePivotLeft.setPosition(intakePivotClips);
                                robot.IntakePivotRight.setPosition(intakePivotClips);
                            }
                        } if(robot.lastState == state.intaking){
                            robot.IntakeSlideRight.setPosition(intakeSlideIn);
                            robot.IntakeSlideLeft.setPosition(intakeSlideIn);
                        }
                        if((robot.lastState != state.intaking) && (robot.lastState != state.rotating)){//THIS MIGHT BE THE SAME AS  if(robot.lastState == state.outTaking){
                            robot.IntakeSlideRight.setPosition(intakeSlideMid);
                            robot.IntakeSlideLeft.setPosition(intakeSlideMid);
                        }
                    } else if ((robot.stateTime.seconds()>.6) && (robot.stateTime.seconds() < 1.5)) {
                        if((robot.lastState != state.intaking) && (robot.lastState != state.rotating)){//THIS MIGHT BE THE SAME AS  if(robot.lastState == state.outTaking){
                            robot.IntakeSlideRight.setPosition(intakeSlideMid);
                            robot.IntakeSlideLeft.setPosition(intakeSlideMid);
                            if(clipping){//NEW////////////////////////////////////////////////////////////////////////////
                                robot.slide.setTargetPosition(slideWallHeight);
                                robot.slide2.setTargetPosition(slideWallHeight);
                            } else {
                                robot.slide.setTargetPosition(slideBuffer);
                                robot.slide2.setTargetPosition(slideBuffer);
                            }
                            robot.slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                            robot.slide.setPower(1);
                            robot.slide2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                            robot.slide2.setPower(1);
                        }
                        //robot.intake.setPower(0);
                        if(!clipping){//NEW//////////////////////////////////////////////////////////////////////////
                            robot.claw.setPosition(clawClose);//
                            robot.IntakePivotLeft.setPosition(intakePivotUp);
                            robot.IntakePivotRight.setPosition(intakePivotUp);
                        } else {
                            robot.claw.setPosition(clawOpen);
                            robot.IntakePivotLeft.setPosition(intakePivotClips);
                            robot.IntakePivotRight.setPosition(intakePivotClips);
                        }
                        robot.bucket.setPosition(bucketIntakepose);//
                    } else {
                        if(clipping){//NEW////////////////////////////////////////////////////////////////////////////
                            robot.slide.setTargetPosition(slideWallHeight);
                            robot.slide2.setTargetPosition(slideWallHeight);
                        } else {
                            robot.slide.setTargetPosition(slideDown);
                            robot.slide2.setTargetPosition(slideDown);
                        }
                        robot.slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        robot.slide.setPower(1);
                        robot.slide2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        robot.slide2.setPower(1);
                        if(robot.slide.getCurrentPosition()<20 || clipping){//change for clips to bring slides in
                            robot.IntakeSlideRight.setPosition(intakeSlideIn);
                            robot.IntakeSlideLeft.setPosition(intakeSlideIn);
                            robot.IntakePivotLeft.setPosition(intakePivotUp);
                            robot.IntakePivotRight.setPosition(intakePivotUp);
                        }
                        //robot.intake.setPower(0);
                    }

                    /*Servo Hang
                    if(gamepad1.dpad_up){
                        robot.hang1.setPower(1);
                        robot.hang2.setPower(1);
                    } else if(gamepad1.dpad_down){
                        robot.hang1.setPower(-1);
                        robot.hang2.setPower(-1);
                    } else {
                        robot.hang1.setPower(0);
                        robot.hang2.setPower(0);
                    }

                    //spin intake or claw if clipping//NEW///////////////////////////////////////////////////////////////////////////
                    if(!clipping){
                        if(gamePad2.Right_Trigger.isDown()){
                            //robot.intake.setPower(1);
                        } else if(gamePad2.Left_Trigger.isDown() && robot.slide.getCurrentPosition()<100){
                            //robot.intake.setPower(-1);
                        }
                    } else {
                        if(gamePad2.Right_Trigger.isDown()){
                            robot.claw.setPosition(clawClose);
                        } else if(gamePad2.Left_Trigger.isDown() || (gamePad2.Left_Trigger.isDown() && gamePad2.Right_Trigger.isDown())){
                            robot.claw.setPosition(clawOpen);
                        } if (gamePad2.Right_Trigger.wasJustReleased() && !gamePad2.Left_Trigger.isDown()){
                            robot.claw.setPosition(clawClose);
                            wasPressed = gamePad2.Dpad_Up;
                            changeStateTo(state.outTaking);
                        }
                    }

                    //switch to intaking or move slide in
                    if(gamepad2.left_bumper){
                        robot.IntakeSlideRight.setPosition(robot.IntakeSlideRight.getPosition() - intakeSlideIncrementsRight);
                        robot.IntakeSlideLeft.setPosition(robot.IntakeSlideLeft.getPosition() - intakeSlideIncrementsLeft);
                    } else if(gamepad2.right_bumper && (robot.IntakeSlideRight.getPosition() < intakeSlideOutLegal) && (robot.IntakeSlideLeft.getPosition() < intakeSlideOutLegal)){
                        robot.IntakeSlideRight.setPosition(robot.IntakeSlideRight.getPosition() + intakeSlideIncrementsRight);
                        robot.IntakeSlideLeft.setPosition(robot.IntakeSlideLeft.getPosition() + intakeSlideIncrementsLeft);
                        robot.IntakePivotLeft.setPosition(intakePivotHalfDown);
                        robot.IntakePivotRight.setPosition(intakePivotHalfDown);
                        changeStateTo(state.intaking);
                    }

                    //control claw
                    if(gamePad2.right_Stick_Button.wasJustPressed() && robot.claw.getPosition() >= clawClose-.1){
                        robot.claw.setPosition(clawOpen);
                    } else if(gamePad2.right_Stick_Button.wasJustPressed() && robot.claw.getPosition() <= clawOpen+.1){
                        robot.claw.setPosition(clawClose);
                    }

                    //out taking
                    if (gamePad2.triangle.wasJustPressed()){
                        wasPressed = gamePad2.triangle;
                        changeStateTo(state.outTaking);
                    }
                    if (gamePad2.square.wasJustPressed()){
                        wasPressed = gamePad2.square;
                        changeStateTo(state.outTaking);
                    }
                    if (gamePad2.circle.wasJustPressed()){
                        wasPressed = gamePad2.circle;
                        changeStateTo(state.outTaking);
                    }
                    if (gamePad2.x.wasJustPressed()){
                        wasPressed = gamePad2.x;
                        changeStateTo(state.outTaking);
                    }
                    if (gamePad2.Dpad_Up.wasJustPressed()){
                        wasPressed = gamePad2.Dpad_Up;
                        changeStateTo(state.outTaking);
                    }
                    if (gamePad2.Dpad_Down.wasJustPressed()){
                        wasPressed = gamePad2.Dpad_Down;
                        changeStateTo(state.outTaking);
                    }
                    if (gamePad2.Dpad_Right.wasJustPressed()){
                        wasPressed = gamePad2.Dpad_Right;
                        changeStateTo(state.outTaking);
                    }
                    if (gamePad2.Dpad_Left.wasJustPressed()){
                        wasPressed = gamePad2.Dpad_Left;
                        changeStateTo(state.outTaking);
                    }

                    //joystick
                    if(Math.abs(gamepad2.right_stick_y) > .2){
                        bucketPose = bucketIntakepose;
                        slideHeight = robot.slide.getCurrentPosition();
                        isReady = false;
                        changeStateTo(state.outTaking);
                    }

                    //rotate 180deg
                    if(gamePad1.right_Stick_Button.wasJustPressed()){
                        changeStateTo(state.rotating);
                        break;
                    }

                    //reset imu
                    if(gamePad1.left_Stick_Button.wasJustPressed()){
                        robot.imu.resetYaw();
                    }
                    //reset encoder
                    if (gamepad2.touchpad){
                        robot.slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        robot.slide2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    }
                    break;
                    

                case intaking:

                    //cheat//NEW///////////////////////////////////////////////////////////////////////////////////////////
                    if(!gamePad2.TouchPad.getToggleState()){
                        intakeSlideOutLegal = .2;
                    } else if(gamePad2.TouchPad.getToggleState()){
                        intakeSlideOutLegal = .4;
                    }

                    updateDrive(DriveSpeed);
                    //control spin
                    robot.claw.setPosition(clawClose);//
                    robot.bucket.setPosition(bucketIntakepose);//
                    if(Math.abs(gamepad2.right_stick_y)>.15){
                        robot.slide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                        robot.slide.setPower(-gamepad2.right_stick_y);
                        robot.slide2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                        robot.slide2.setPower(-gamepad2.right_stick_y);
                    } else{
                        if(clipping){//NEW/////////////////////////////////////////////////////////////////////
                            slideHeight = slideWallHeight;
                        } else{
                            slideHeight = slideDown;
                        }
                        robot.slide.setTargetPosition(slideHeight);
                        robot.slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        robot.slide.setPower(1);
                        robot.slide2.setTargetPosition(slideHeight);
                        robot.slide2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        robot.slide2.setPower(1);
                    }
                    //robot.intake.setPower(0);
                    if(gamePad2.Right_Trigger.isDown()){
                        //robot.intake.setPower(1);
                    } else if(gamePad2.Left_Trigger.isDown() || (gamePad2.Left_Trigger.isDown() && gamePad2.Right_Trigger.isDown())){
                        //robot.intake.setPower(-1);
                    } if (gamePad2.Right_Trigger.wasJustReleased() && !gamePad2.Left_Trigger.isDown()){
                        //robot.intake.setPower(1);
                        if(!clipping){//NEW/////////////////////////////////////////////////////////////////////////////
                            robot.IntakePivotLeft.setPosition(intakePivotBuffer);
                            robot.IntakePivotRight.setPosition(intakePivotBuffer);
                        } else {
                            robot.IntakePivotLeft.setPosition(intakePivotClips);
                            robot.IntakePivotRight.setPosition(intakePivotClips);
                        }
                        intakeSlideOutLegal = .2;
                        //gamePad2.TouchPad.currToggleState = false;
                        changeStateTo(state.driving);
                    }

                    //Intake Pivot
                        if (gamepad2.left_stick_y < -.1) {
                            robot.IntakePivotLeft.setPosition(robot.IntakePivotLeft.getPosition() - intakePivotIncrements);
                            robot.IntakePivotRight.setPosition(robot.IntakePivotRight.getPosition() - intakePivotIncrements);
                        }
                        if ((gamepad2.left_stick_y > .1)) {
                            if (robot.IntakePivotLeft.getPosition() < .545) {
                                robot.IntakePivotLeft.setPosition(robot.IntakePivotLeft.getPosition() + intakePivotIncrements);
                                robot.IntakePivotRight.setPosition(robot.IntakePivotRight.getPosition() + intakePivotIncrements);
                            } else{
                                robot.IntakePivotRight.setPosition(.545+intakePivotIncrements); //NEW////////////////////////////////////////////////////////
                                robot.IntakePivotLeft.setPosition(.545+intakePivotIncrements);
                            }
                        }
                    telemetry.addData("servo", robot.IntakePivotLeft.getPosition());

                    //control slide
                    if(gamepad2.left_bumper){
                        robot.IntakeSlideRight.setPosition(robot.IntakeSlideRight.getPosition() - intakeSlideIncrementsRight);
                        robot.IntakeSlideLeft.setPosition(robot.IntakeSlideLeft.getPosition() - intakeSlideIncrementsLeft);
                    } else if(gamepad2.right_bumper && (robot.IntakeSlideRight.getPosition() < intakeSlideOutLegal) && (robot.IntakeSlideLeft.getPosition() < intakeSlideOutLegal)){
                        robot.IntakeSlideRight.setPosition(robot.IntakeSlideRight.getPosition() + intakeSlideIncrementsRight);
                        robot.IntakeSlideLeft.setPosition(robot.IntakeSlideLeft.getPosition() + intakeSlideIncrementsLeft);
                    } else if((robot.IntakeSlideRight.getPosition() > intakeSlideOutLegal) && (robot.IntakeSlideLeft.getPosition() > intakeSlideOutLegal)){
                        robot.IntakeSlideRight.setPosition(intakeSlideOutLegal);//NEW///////////////////////////////////////////////////////////////////////////
                        robot.IntakeSlideLeft.setPosition(intakeSlideOutLegal);
                    }
                    telemetry.addData("intake slide right: ", robot.IntakeSlideRight.getPosition());
                    telemetry.addData("intake slide left: ", robot.IntakeSlideLeft.getPosition());
                    telemetry.addData("gamepadbumper: ", gamepad2.right_bumper);
                    break;


                case outTaking:
                    //robot.intake.setPower(0);
                    if(gamePad2.right_Stick_Button.wasJustPressed() && robot.claw.getPosition() >= clawClose-.1){
                        robot.claw.setPosition(clawOpen);
                    } else if(gamePad2.right_Stick_Button.wasJustPressed() && robot.claw.getPosition() <= clawOpen+.1){
                        robot.claw.setPosition(clawClose);
                    }
                    updateDrive(DriveSpeed);
                    if(wasPressed == gamePad2.triangle || gamePad2.triangle.isDown()){
                        wasPressed = gamePad2.triangle;
                        slideHeight = slideHighBucket;
                        bucketPose = bucketHorizontal;
                        shoulderPose = clawShoulderBack;
                        isMoving = true;
                    }
                    if(wasPressed == gamePad2.square || gamePad2.square.isDown()){
                        wasPressed = gamePad2.square;
                        slideHeight = slideLowBucket;
                        bucketPose = bucketHorizontal;
                        shoulderPose = clawShoulderBack;
                        isMoving = true;
                    }
                    if(wasPressed == gamePad2.circle || gamePad2.circle.isDown()){
                        wasPressed = gamePad2.circle;
                        slideHeight = slideLowBucket;
                        bucketPose = bucketHorizontal;
                        shoulderPose = clawShoulderBack;
                        isMoving = true;
                    }
                    if(wasPressed == gamePad2.x || gamePad2.x.isDown()){
                        wasPressed = gamePad2.x;
                        slideHeight = slideWallHeight;
                        bucketPose = bucketHorizontal;
                        shoulderPose = clawShoulderBack;
                        isMoving = true;
                    }
                    if(wasPressed == gamePad2.Dpad_Up || gamePad2.Dpad_Up.isDown()){
                        wasPressed = gamePad2.Dpad_Up;
                        slideHeight = slideHighBar;
                        bucketPose = bucketClipScore;
                        isMoving = true;
                    }
                    if(wasPressed == gamePad2.Dpad_Down || gamePad2.Dpad_Down.isDown()){
                        wasPressed = gamePad2.Dpad_Down;
                        slideHeight = slideWallHeight;
                        bucketPose = bucketIntakepose;
                        isMoving = true;
                    }
                    if(wasPressed == gamePad2.Dpad_Left || gamePad2.Dpad_Left.isDown()){
                        wasPressed = gamePad2.Dpad_Left;
                        slideHeight = slideHighBar;
                        bucketPose = bucketClipScore;
                        isMoving = true;
                    }
                    if(wasPressed == gamePad2.Dpad_Right || gamePad2.Dpad_Right.isDown()){
                        wasPressed = gamePad2.Dpad_Right;
                        slideHeight = slideHighBar;
                        bucketPose = bucketClipScore;
                        isMoving = true;
                    }

                    //bucket/claw NEW///////////////////////////////////////////////////////////////////////
                    if(clipping){
                        if (gamePad2.Left_Trigger.isDown() || gamePad2.Right_Trigger.isDown()){
                            robot.claw.setPosition(clawOpen);
                            if(gamePad2.Right_Trigger.isDown()){
                                robot.bucket.setPosition(bucketAfterClipPose);
                            }
                        }
                    } else {
                        if (gamePad2.Left_Trigger.isDown() || gamePad2.Right_Trigger.isDown()) {
                            if (gamePad2.Left_Trigger.isDown() && gamePad2.Right_Trigger.isDown()) {
                                robot.bucket.setPosition(bucketHorizontal);
                            } else {
                                robot.bucket.setPosition(bucketDump);
                            }
                        } else {
                            if (robot.slide.getCurrentPosition() >= slideBuffer) {
                                robot.bucket.setPosition(bucketPose);
                            }
                        }
                    }

                    if(gamePad2.Left_Trigger.wasJustReleased()|| gamePad2.Right_Trigger.wasJustReleased()){
                        if ((!gamePad2.Left_Trigger.isDown() && !gamePad2.Right_Trigger.isDown())) {
                            robot.IntakeSlideRight.setPosition(intakeSlideMid);
                            robot.IntakeSlideLeft.setPosition(intakeSlideMid);
                            changeStateTo(state.driving);
                        }

                    }
                    //reset encoder
                    if(gamepad2.touchpad){
                        robot.slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        robot.slide2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    }
                    if(slideHeight <= slideBuffer && !isReady){
                        robot.slide.setTargetPosition(slideBuffer);
                        robot.slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        robot.slide.setPower(1);
                        robot.slide2.setTargetPosition(slideBuffer);
                        robot.slide2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        robot.slide2.setPower(1);
                        if(robot.slide.getCurrentPosition() >= slideBuffer-2) {
                            if(robot.stateTime.seconds()>1) {
                                robot.bucket.setPosition(bucketPose);
                                isReady = true;
                            }
                        }
                    } else {
                        //move preset
                        if (isMoving) {
                            //slide
                            robot.slide.setTargetPosition(slideHeight);
                            robot.slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                            robot.slide.setPower(1);
                            robot.slide2.setTargetPosition(slideHeight);
                            robot.slide2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                            robot.slide2.setPower(1);
                            //servo

                            if (robot.slide.getCurrentPosition() > slideHeight - 50 && robot.slide.getCurrentPosition() < slideHeight + 50) {
                                //keep slide up
                                robot.bucket.setPosition(robot.bucket.getPosition() + (gamepad2.left_stick_x / 50));
                                bucketPose = robot.bucket.getPosition();
                                //rotate
                                if (-gamepad2.right_stick_y != 0) {
                                    isMoving = false;
                                }
                                if (robot.slide.getCurrentPosition() >= slideHeight) {
                                    robot.IntakeSlideRight.setPosition(intakeSlideIn);
                                    robot.IntakeSlideLeft.setPosition(intakeSlideIn);
                                }
                                wasPressed = null;
                                //magnet sensor
                            } else{
                                robot.IntakeSlideRight.setPosition(intakeSlideMid);
                                robot.IntakeSlideLeft.setPosition(intakeSlideMid);

                            }
                        } else { //manual control
                            manualSlide();
                            robot.bucket.setPosition(robot.bucket.getPosition() + (gamepad2.left_stick_x / 50));//changed it from 400; see if it works
                            telemetry.addData("slide", robot.slide.getCurrentPosition());
                        }
                        if (gamePad2.Left_Bumper.wasJustPressed() || gamePad2.Right_Bumper.wasJustPressed()) {
                            if (shoulderPose == clawShoulderFoward) {
                                shoulderPose = clawShoulderBack;
                            } else {
                                shoulderPose = clawShoulderFoward;
                            }
                        }
                    }
                    break;

                case rotating:
                    //home all //I removed a lot of the homing safety features/////////////////////////////////////////////
                    robot.IntakeSlideRight.setPosition(intakeSlideIn);
                    robot.IntakeSlideLeft.setPosition(intakeSlideIn);

                    robot.kachow.rotate(robot.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS), gamepad1, gamepad2, robot);
                    changeStateTo(state.driving);
                    break;

                /**case turnto:
                    telemetry.addData("heading IMU", robot.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
                    robot.kachow.rotateTo(Math.toRadians(0), gamepad1, gamepad2, robot, .4);
                    robot.kachow.drive_pause(robot);
                    sleep(100);
                    robot.kachow.rotateTo(Math.toRadians(0), gamepad1, gamepad2, robot, .15);
                    State = state.driving;
                    robot.stateUpdate(State);
                    break;
                case aimbot:
                    robot.kachow.aimbot(new Vector2d(0,0), gamepad1, gamepad2, robot, .1);
                    if(Math.abs(gamepad1.right_stick_x)>.15){
                        State = state.driving;
                    }
                    telemetry.addData("X", robot.kachow.roadRunner.pose.position.x);
                    telemetry.addData("y", robot.kachow.roadRunner.pose.position.y);
                    telemetry.addData("DEG", Math.toDegrees(robot.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS)));
                    telemetry.addLine();
                    telemetry.addData("LEDS pattern", robot.pattern);
                    telemetry.addData("State", State);
                    telemetry.addData("State time", robot.stateTime);
                    telemetry.addData("current state", robot.currentState);
                    telemetry.addData("last state", robot.lastState);
                    telemetry.addData("bot angle", Math.toDegrees(robot.kachow.bot_angle));
                    telemetry.addData("wall angle", Math.toDegrees(robot.kachow.wall_angle));
                    telemetry.addData("target angle", Math.toDegrees(robot.kachow.target_angle));
                    telemetry.addData("bot to target", Math.toDegrees(robot.kachow.bot_to_target));
                    telemetry.addData("bot to wall", Math.toDegrees(robot.kachow.bot_to_wall));
                    telemetry.addData("wall to target", Math.toDegrees(robot.kachow.wall_to_target));
                    telemetry.update();  */
            /*}
        }
    }
    public void updateEnd() { //place once at end// will have a the set positions for the refactored code
    }

}*/
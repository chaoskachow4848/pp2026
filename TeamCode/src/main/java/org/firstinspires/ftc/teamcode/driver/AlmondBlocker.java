package org.firstinspires.ftc.teamcode.driver;

import static org.firstinspires.ftc.teamcode.driver.RobotCentricDriverSample.ShoulderBuckets;
import static org.firstinspires.ftc.teamcode.driver.RobotCentricDriverSample.ShoulderClips;
import static org.firstinspires.ftc.teamcode.driver.RobotCentricDriverSample.ShoulderTransfer;
import static org.firstinspires.ftc.teamcode.driver.RobotCentricDriverSample.bucket45Score;
import static org.firstinspires.ftc.teamcode.driver.RobotCentricDriverSample.bucketClipScore;
import static org.firstinspires.ftc.teamcode.driver.RobotCentricDriverSample.bucketIntakepose;
import static org.firstinspires.ftc.teamcode.driver.RobotCentricDriverSample.bucketVertical;
import static org.firstinspires.ftc.teamcode.driver.RobotCentricDriverSample.clawClose;
import static org.firstinspires.ftc.teamcode.driver.RobotCentricDriverSample.clawOpen;
import static org.firstinspires.ftc.teamcode.driver.RobotCentricDriverSample.hangClipping;
import static org.firstinspires.ftc.teamcode.driver.RobotCentricDriverSample.hangIncrements;
import static org.firstinspires.ftc.teamcode.driver.RobotCentricDriverSample.hangInit;
import static org.firstinspires.ftc.teamcode.driver.RobotCentricDriverSample.hangReady;
import static org.firstinspires.ftc.teamcode.driver.RobotCentricDriverSample.hangStandby;
import static org.firstinspires.ftc.teamcode.driver.RobotCentricDriverSample.hanging;
import static org.firstinspires.ftc.teamcode.driver.RobotCentricDriverSample.intakeClawClose;
import static org.firstinspires.ftc.teamcode.driver.RobotCentricDriverSample.intakeClawOpen;
import static org.firstinspires.ftc.teamcode.driver.RobotCentricDriverSample.intakeClawPrecise;
import static org.firstinspires.ftc.teamcode.driver.RobotCentricDriverSample.intakePivotBuffer;
import static org.firstinspires.ftc.teamcode.driver.RobotCentricDriverSample.intakePivotDown;
import static org.firstinspires.ftc.teamcode.driver.RobotCentricDriverSample.intakePivotHalfDown;
import static org.firstinspires.ftc.teamcode.driver.RobotCentricDriverSample.intakePivotHoverPrecise;
import static org.firstinspires.ftc.teamcode.driver.RobotCentricDriverSample.intakePivotIncrements;
import static org.firstinspires.ftc.teamcode.driver.RobotCentricDriverSample.intakePivotUp;
import static org.firstinspires.ftc.teamcode.driver.RobotCentricDriverSample.intakeRotateHori;
import static org.firstinspires.ftc.teamcode.driver.RobotCentricDriverSample.intakeRotateVert;
import static org.firstinspires.ftc.teamcode.driver.RobotCentricDriverSample.intakeSlideIn;
import static org.firstinspires.ftc.teamcode.driver.RobotCentricDriverSample.intakeSlideIncrementsLeft;
import static org.firstinspires.ftc.teamcode.driver.RobotCentricDriverSample.intakeSlideIncrementsRight;
import static org.firstinspires.ftc.teamcode.driver.RobotCentricDriverSample.intakeSlideMid;
import static org.firstinspires.ftc.teamcode.driver.RobotCentricDriverSample.intakeSlideOutLegal;
import static org.firstinspires.ftc.teamcode.driver.RobotCentricDriverSample.intakeSlideOutMax;
import static org.firstinspires.ftc.teamcode.driver.RobotCentricDriverSample.intakeWristDown;
import static org.firstinspires.ftc.teamcode.driver.RobotCentricDriverSample.intakeWristHover;
import static org.firstinspires.ftc.teamcode.driver.RobotCentricDriverSample.intakeWristHoverPrecise;
import static org.firstinspires.ftc.teamcode.driver.RobotCentricDriverSample.intakeWristStandby;
import static org.firstinspires.ftc.teamcode.driver.RobotCentricDriverSample.intakeWristTransfer;
import static org.firstinspires.ftc.teamcode.driver.RobotCentricDriverSample.slideBuffer;
import static org.firstinspires.ftc.teamcode.driver.RobotCentricDriverSample.slideDown;
import static org.firstinspires.ftc.teamcode.driver.RobotCentricDriverSample.slideHeight;
import static org.firstinspires.ftc.teamcode.driver.RobotCentricDriverSample.slideHighBar;
import static org.firstinspires.ftc.teamcode.driver.RobotCentricDriverSample.slideHighBucket;
import static org.firstinspires.ftc.teamcode.driver.RobotCentricDriverSample.slideLowBucket;
import static org.firstinspires.ftc.teamcode.driver.RobotCentricDriverSample.slideWallHeight;
import static org.firstinspires.ftc.teamcode.hardware.SampleHardware.state;

import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.hardware.SampleHardware;
import org.firstinspires.ftc.teamcode.hardware.button;

@TeleOp(name="Almond Blocker", group="4848")
public class AlmondBlocker extends LinearOpMode {
    public static boolean isReady = false;
    public static int hangPose = hangStandby;//start at standby
    public static double bucketPose = bucketIntakepose;//default to intakepose
    //public static double bucketDump = bucketIntakepose;//useless
    public static double shoulderPose = 0;//useless
    public boolean transferred = false;
    public double timecapture = 0;
    boolean clawUp = false;
    SampleHardware robot = new SampleHardware();
    button.ButtonReader gamePad1 = new button.ButtonReader();
    button.ButtonReader gamePad2 = new button.ButtonReader();
    state State = state.driving;
    boolean SlowModeIsOn = false;
    double DriveSpeed = 1;
    boolean isClipping = false;
    boolean Precise = false;

    @Override public void runOpMode() {


        boolean isMoving = false;

        button wasPressed = null;

        robot.init(hardwareMap);
        gamepad2.runLedEffect(robot.redled);
        gamepad1.runLedEffect(robot.blueled);
        telemetry.update();
        robot.pattern = RevBlinkinLedDriver.BlinkinPattern.BLACK;
        robot.leds.setPattern(robot.pattern);
        robot.spinny1.setTargetPosition(hangInit);
        robot.spinny1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.spinny1.setPower(1);
        robot.spinny2.setTargetPosition(hangInit);
        robot.spinny2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.spinny2.setPower(1);
        waitForStart();
        hangPose = hangStandby;//start at standby
        robot.runtime.reset();//NEW//////////////////////////////////////////////////////////////////////////////////

        while (opModeIsActive()) {
            update();
            telemetry.addData("magnet: ", robot.magnet.getState());
            telemetry.addData("magnet button release: ", gamePad2.justMagnet.wasJustReleased());


            switch (State){

                case driving:
                    isReady = false;
                    updateDrive(DriveSpeed);
                    //gamePad2.TouchPad.currToggleState = false;
                    if(robot.stateChanged){
                        transferred = false;
                    }

                    //home all
                    if(robot.stateTime.seconds()<.7){
                        if(robot.lastState == state.outTaking){
                            if(shoulderPose != ShoulderClips){
                                robot.shoulder.setPosition(ShoulderTransfer);
                                robot.bucket.setPosition(bucketIntakepose);
                            }
                            robot.slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                            robot.slide.setTargetPosition(slideBuffer);
                            robot.slide.setPower(1);
                            robot.slide2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                            robot.slide2.setTargetPosition(slideBuffer);
                            robot.slide2.setPower(1);
                            robot.IntakePivotLeft.setPosition(intakePivotBuffer);
                            robot.IntakePivotRight.setPosition(intakePivotBuffer);
                            robot.IntakeWrist.setPosition(intakeWristHover);

                        } if(robot.lastState == state.intaking){
                            robot.shoulder.setPosition(ShoulderTransfer);
                            robot.IntakeClaw.setPosition(.76);
                            robot.claw.setPosition(clawOpen);
                            robot.IntakeRotate.setPosition(intakeRotateHori);


                            if(isClipping){
                                robot.IntakeWrist.setPosition(intakeWristDown);
                                robot.IntakePivotLeft.setPosition(intakePivotBuffer);
                                robot.IntakePivotRight.setPosition(intakePivotBuffer);
                            }else{
                                //updateDrive(0);
                                robot.IntakeWrist.setPosition(intakeWristTransfer);
                                robot.IntakePivotLeft.setPosition(intakePivotUp);
                                robot.IntakePivotRight.setPosition(intakePivotUp);
                            }

                            robot.IntakeSlideLeft.setPosition(.05);
                            robot.IntakeSlideRight.setPosition(.05);

                        }
                        if((robot.lastState != state.intaking) && (robot.lastState != state.rotating)){//THIS MIGHT BE THE SAME AS  if(robot.lastState == state.outTaking){
                            robot.IntakeSlideRight.setPosition(intakeSlideMid/3);
                            robot.IntakeSlideLeft.setPosition(intakeSlideMid/3);
                            robot.IntakeWrist.setPosition(intakeWristHover);
                        }
                    } else if ((robot.stateTime.seconds()>.7) && (robot.stateTime.seconds() < 1.25)) {
                        if((robot.lastState != state.intaking) && (robot.lastState != state.rotating)){//THIS MIGHT BE THE SAME AS  if(robot.lastState == state.outTaking){
                            robot.shoulder.setPosition(ShoulderTransfer);
                            robot.IntakeSlideRight.setPosition(intakeSlideMid/3);
                            robot.IntakeSlideLeft.setPosition(intakeSlideMid/3);
                                robot.slide.setTargetPosition(slideBuffer);
                                robot.slide2.setTargetPosition(slideBuffer);
                            robot.slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                            robot.slide.setPower(1);
                            robot.slide2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                            robot.slide2.setPower(1);
                            robot.claw.setPosition(clawClose);

                            robot.bucket.setPosition(bucketIntakepose);
                        }
                        if(robot.lastState == state.intaking){
                            robot.shoulder.setPosition(ShoulderTransfer);
                            if(isClipping){
                                robot.IntakeWrist.setPosition(intakeWristDown);
                                robot.IntakePivotLeft.setPosition(intakePivotBuffer);
                                robot.IntakePivotRight.setPosition(intakePivotBuffer);
                            }else{
                                robot.IntakeWrist.setPosition(intakeWristTransfer);
                                robot.IntakePivotLeft.setPosition(intakePivotUp);
                                robot.IntakePivotRight.setPosition(intakePivotUp);
                                robot.IntakeClaw.setPosition(intakeClawClose);
                            }

                            robot.IntakeSlideLeft.setPosition(intakeSlideIn);
                            robot.IntakeSlideRight.setPosition(intakeSlideIn);

                        }
                        //robot.intake.setPower(0);
                        robot.bucket.setPosition(bucketIntakepose);//
                    } else {
                        robot.shoulder.setPosition(ShoulderTransfer);
                        robot.bucket.setPosition(bucketIntakepose);
                        robot.slide.setTargetPosition(slideDown);
                        robot.slide2.setTargetPosition(slideDown);
                        robot.slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        robot.slide.setPower(1);
                        robot.slide2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        robot.slide2.setPower(1);
                        if(robot.slide.getCurrentPosition()<20){//if slides are down
                            robot.IntakeSlideRight.setPosition(intakeSlideIn);
                            robot.IntakeSlideLeft.setPosition(intakeSlideIn);
                            /*if(!transferred){//NEW//////////////////////////////////////////////////////////////////////////
                                robot.claw.setPosition(clawOpen);//
                                robot.IntakePivotLeft.setPosition(intakePivotUp);
                                robot.IntakePivotRight.setPosition(intakePivotUp);
                                robot.IntakeWrist.setPosition(intakeWristTransfer);
                            } else {
                                robot.claw.setPosition(clawClose);
                                robot.IntakePivotLeft.setPosition(intakePivotClips);
                                robot.IntakePivotRight.setPosition(intakePivotClips);
                            }*/
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
                    }*/

                    //spin intake or claw if clipping//NEW///////////////////////////////////////////////////////////////////////////


                    }/* else {
                        if(gamePad2.Right_Trigger.isDown()){
                            robot.claw.setPosition(clawClose);
                        } else if((gamePad2.Left_Trigger.isDown() && gamePad2.Right_Trigger.isDown())){
                            robot.claw.setPosition(clawOpen);
                        } if (gamePad2.Right_Trigger.wasJustReleased() && !gamePad2.Left_Trigger.isDown()){
                            robot.claw.setPosition(clawClose);
                            wasPressed = gamePad2.Dpad_Up;
                            bucketPose = bucketClipScore;
                            changeStateTo(state.outTaking);
                        }
                    }*/

                    if(!transferred){
                        if(gamePad2.Right_Trigger.wasJustPressed() || (gamePad2.Left_Trigger.wasJustPressed())){
                            robot.IntakeWrist.setPosition(intakeWristTransfer);
                            robot.IntakeRotate.setPosition(intakeRotateHori);
                            robot.claw.setPosition(clawClose);
                            robot.IntakeClaw.setPosition(intakeClawClose);
                        } else if(gamePad2.Right_Trigger.wasJustReleased() || (gamePad2.Left_Trigger.wasJustReleased())){
                            if(robot.stateTime.seconds()>2){
                                robot.IntakeClaw.setPosition(intakeClawOpen);
                                robot.IntakeWrist.setPosition(intakeWristStandby);
                                robot.IntakePivotLeft.setPosition(intakePivotBuffer);
                                robot.IntakePivotRight.setPosition(intakePivotBuffer);
                                robot.claw.setPosition(clawClose);
                                transferred = true;
                            }
                        }
                        //robot.intake.setPower(0);
                    }

                    //switch to intaking or move slide in
                    if(gamepad2.left_bumper){
                        robot.IntakeSlideRight.setPosition(robot.IntakeSlideRight.getPosition() - intakeSlideIncrementsRight);
                        robot.IntakeSlideLeft.setPosition(robot.IntakeSlideLeft.getPosition() - intakeSlideIncrementsLeft);
                    } else if(gamepad2.right_bumper && (robot.IntakeSlideRight.getPosition() < intakeSlideOutLegal) && (robot.IntakeSlideLeft.getPosition() < intakeSlideOutLegal)){
                        robot.IntakeSlideRight.setPosition(robot.IntakeSlideRight.getPosition() + intakeSlideIncrementsRight);
                        robot.IntakeSlideLeft.setPosition(robot.IntakeSlideLeft.getPosition() + intakeSlideIncrementsLeft);
                        //robot.IntakeClaw.setPosition(intakeClawPrecise);
                        robot.IntakeWrist.setPosition(intakeWristHover);
                        changeStateTo(state.intaking);
                    }

                    //reset position when isClipping changes
                    if(gamePad2.Guide.stateJustChanged()){
                        robot.lastState = state.intaking;
                        changeStateTo(state.driving);
                        robot.stateTime.reset();
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
                        robot.IntakeClaw.setPosition(intakeClawOpen);
                        changeStateTo(state.outTaking);
                    }
                    if (gamePad2.square.wasJustPressed()){
                        wasPressed = gamePad2.square;
                        robot.IntakeClaw.setPosition(intakeClawOpen);
                        changeStateTo(state.outTaking);
                    }
                    if (gamePad2.circle.wasJustPressed()){
                        wasPressed = gamePad2.circle;
                        robot.IntakeClaw.setPosition(intakeClawOpen);
                        changeStateTo(state.outTaking);
                    }
                    if (gamePad2.x.wasJustPressed()){
                        wasPressed = gamePad2.x;
                        robot.IntakeClaw.setPosition(intakeClawOpen);
                        changeStateTo(state.outTaking);
                    }
                    if (gamePad2.Dpad_Up.wasJustPressed()){
                        wasPressed = gamePad2.Dpad_Up;
                        robot.IntakeClaw.setPosition(intakeClawOpen);
                        changeStateTo(state.outTaking);
                    }
                    if (gamePad2.Dpad_Down.wasJustPressed()){
                        wasPressed = gamePad2.Dpad_Down;
                        robot.IntakeClaw.setPosition(intakeClawOpen);
                        changeStateTo(state.outTaking);
                    }
                    if (gamePad2.Dpad_Right.wasJustPressed()){
                        wasPressed = gamePad2.Dpad_Right;
                        robot.IntakeClaw.setPosition(intakeClawOpen);
                        changeStateTo(state.outTaking);
                    }
                    if (gamePad2.Dpad_Left.wasJustPressed()){
                        wasPressed = gamePad2.Dpad_Left;
                        robot.IntakeClaw.setPosition(intakeClawOpen);
                        changeStateTo(state.outTaking);
                    }

                    //joystick
                    if(Math.abs(gamepad2.right_stick_y) > .2){
                        bucketPose = bucketIntakepose;
                        shoulderPose = ShoulderTransfer;
                        slideHeight = robot.slide.getCurrentPosition();
                        isReady = false;
                        changeStateTo(state.outTaking);
                        robot.IntakeClaw.setPosition(intakeClawPrecise);
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
                    /*if (gamepad2.touchpad){
                        robot.slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        robot.slide2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    }

                     */
                    break;
                    

                case intaking:
                    if(robot.stateChanged){
                        //robot.IntakeClaw.setPosition(intakeClawPrecise);
                        robot.IntakePivotLeft.setPosition((intakePivotHalfDown+intakePivotBuffer)/2);
                        robot.IntakePivotRight.setPosition((intakePivotHalfDown+intakePivotBuffer)/2);
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
                            slideHeight = slideDown;
                        robot.slide.setTargetPosition(slideHeight);
                        robot.slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        robot.slide.setPower(1);
                        robot.slide2.setTargetPosition(slideHeight);
                        robot.slide2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        robot.slide2.setPower(1);
                    }

                    if(gamePad2.Right_Trigger.wasJustPressed()){
                        if(Precise){
                            robot.IntakeClaw.setPosition(intakeClawPrecise);
                            robot.IntakePivotLeft.setPosition(intakePivotHoverPrecise);
                            robot.IntakePivotRight.setPosition(intakePivotHoverPrecise);
                            robot.IntakeWrist.setPosition(intakeWristHoverPrecise);
                        } else{
                            robot.IntakeClaw.setPosition(intakeClawOpen);
                            robot.IntakePivotLeft.setPosition(intakePivotHalfDown);
                            robot.IntakePivotRight.setPosition(intakePivotHalfDown);
                            robot.IntakeWrist.setPosition(intakeWristHover);
                        }
                    } else if(gamePad2.Right_Trigger.wasJustReleased()){
                        robot.IntakeClaw.setPosition(.76);
                        robot.IntakePivotLeft.setPosition(intakePivotDown);
                        robot.IntakePivotRight.setPosition(intakePivotDown);
                        robot.IntakeWrist.setPosition(intakeWristDown);
                    }


                    if (gamePad2.Left_Trigger.isDown()){
                        robot.IntakeClaw.setPosition(.76);
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

                    if(gamePad2.Dpad_Down.wasJustPressed() || gamePad2.Dpad_Up.wasJustPressed() || gamePad2.Dpad_Right.wasJustPressed() || gamePad2.Dpad_Left.wasJustPressed()){
                        if(robot.IntakeRotate.getPosition() > ((double) 195/300)){ //this is assuming that horizontal position is zero// if horizontal
                            robot.IntakeRotate.setPosition(intakeRotateVert);
                        } else if(robot.IntakeRotate.getPosition() < ((double) 195/300)){ //this is assuming that horizontal position is zero// if horizontal
                            robot.IntakeRotate.setPosition(intakeRotateHori);
                        }
                    }
                    telemetry.addData("rotate", robot.IntakeRotate.getPosition());


                    if(gamePad2.circle.wasJustPressed() || gamePad2.square.wasJustPressed() || gamePad2.triangle.wasJustPressed() || gamePad2.x.wasJustPressed()){
                        if(robot.IntakeClaw.getPosition() < intakeClawClose-.2){ //this is assuming that open position is zero// if claw is open
                            robot.IntakeClaw.setPosition(intakeClawClose);
                        } else {
                            if(Precise){
                                robot.IntakeClaw.setPosition(intakeClawPrecise);
                            } else{
                                robot.IntakeClaw.setPosition(intakeClawOpen);
                            }
                        }
                    }

                    //control slide
                    if(gamePad2.Right_Trigger.isDown()){ //cheat
                        if(gamepad2.left_bumper){
                            robot.IntakeSlideRight.setPosition(robot.IntakeSlideRight.getPosition() - intakeSlideIncrementsRight);
                            robot.IntakeSlideLeft.setPosition(robot.IntakeSlideLeft.getPosition() - intakeSlideIncrementsLeft);
                        } else if(gamepad2.right_bumper && (robot.IntakeSlideRight.getPosition() < intakeSlideOutMax) && (robot.IntakeSlideLeft.getPosition() < intakeSlideOutMax)){
                            robot.IntakeSlideRight.setPosition(robot.IntakeSlideRight.getPosition() + intakeSlideIncrementsRight);
                            robot.IntakeSlideLeft.setPosition(robot.IntakeSlideLeft.getPosition() + intakeSlideIncrementsLeft);
                        } else if((robot.IntakeSlideRight.getPosition() > intakeSlideOutMax) && (robot.IntakeSlideLeft.getPosition() > intakeSlideOutMax)){
                            robot.IntakeSlideRight.setPosition(intakeSlideOutMax);//NEW///////////////////////////////////////////////////////////////////////////
                            robot.IntakeSlideLeft.setPosition(intakeSlideOutMax);
                        }
                    } else { //legal
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
                    }
                    telemetry.addData("intake slide right: ", robot.IntakeSlideRight.getPosition());
                    telemetry.addData("intake slide left: ", robot.IntakeSlideLeft.getPosition());
                    telemetry.addData("gamepadbumper: ", gamepad2.right_bumper);
                    break;


                case outTaking:
                    updateDrive(DriveSpeed);
                    if(robot.stateChanged){
                        if(Precise){
                            robot.IntakeClaw.setPosition(intakeClawPrecise);
                        }else{
                            robot.IntakeClaw.setPosition(intakeClawOpen);
                        }
                        robot.claw.setPosition(clawClose);
                        robot.shoulder.setPosition(ShoulderTransfer);

                    }
                    if (isClipping){
                        if(Precise){
                            robot.IntakeWrist.setPosition(intakeWristHoverPrecise);
                        }else{
                            robot.IntakeWrist.setPosition(intakeWristHover);
                        }
                    } else{
                        robot.IntakeWrist.setPosition(intakeWristTransfer);
                    }
                    robot.IntakePivotLeft.setPosition(intakePivotBuffer);
                    robot.IntakePivotRight.setPosition(intakePivotBuffer);
                    if(gamePad2.right_Stick_Button.wasJustPressed() && robot.claw.getPosition() >= clawClose-.1){
                            robot.claw.setPosition(clawOpen);
                        } else if(gamePad2.right_Stick_Button.wasJustPressed() && robot.claw.getPosition() <= clawOpen+.1){
                            robot.claw.setPosition(clawClose);
                        }
                        if(wasPressed == gamePad2.triangle || gamePad2.triangle.isDown()){
                            wasPressed = gamePad2.triangle;
                            slideHeight = slideHighBucket;
                            bucketPose = bucketVertical;
                            shoulderPose = ShoulderTransfer;
                            isMoving = true;
                        }
                        if(wasPressed == gamePad2.square || gamePad2.square.isDown()){
                            wasPressed = gamePad2.square;
                            slideHeight = slideHighBucket;
                            bucketPose = bucketVertical;
                            shoulderPose = ShoulderTransfer;
                            isMoving = true;
                        }
                        if(wasPressed == gamePad2.circle || gamePad2.circle.isDown()){
                            wasPressed = gamePad2.circle;
                            slideHeight = slideHighBucket;
                            bucketPose = bucket45Score;
                            shoulderPose = ShoulderTransfer;
                            isMoving = true;
                        }
                        if(wasPressed == gamePad2.x || gamePad2.x.isDown()){
                            wasPressed = gamePad2.x;
                            slideHeight = slideLowBucket;
                            bucketPose = bucket45Score;
                            shoulderPose = ShoulderTransfer;
                            isMoving = true;
                        }
                        if(wasPressed == gamePad2.Dpad_Up || gamePad2.Dpad_Up.isDown()){
                            wasPressed = gamePad2.Dpad_Up;
                            slideHeight = slideHighBar;
                            bucketPose = bucketClipScore;
                            shoulderPose = ShoulderClips;
                            isMoving = true;
                        }
                        if(wasPressed == gamePad2.Dpad_Down || gamePad2.Dpad_Down.isDown()){
                            wasPressed = gamePad2.Dpad_Down;
                            slideHeight = slideWallHeight;
                            bucketPose = bucketIntakepose;
                            shoulderPose = ShoulderClips;
                            isMoving = true;
                        }
                        if(wasPressed == gamePad2.Dpad_Left || gamePad2.Dpad_Left.isDown()){
                            wasPressed = gamePad2.Dpad_Left;
                            slideHeight = slideHighBar;
                            bucketPose = bucketClipScore;
                            shoulderPose = ShoulderClips;
                            isMoving = true;
                        }
                        if(wasPressed == gamePad2.Dpad_Right || gamePad2.Dpad_Right.isDown()){
                            wasPressed = gamePad2.Dpad_Right;
                            slideHeight = slideHighBar;
                            bucketPose = bucketClipScore;
                            shoulderPose = ShoulderClips;
                            isMoving = true;
                        }

                        //bucket/claw NEW///////////////////////////////////////////////////////////////////////
                    if(shoulderPose == ShoulderClips){
                        if (gamePad2.Left_Trigger.isDown() || gamePad2.Right_Trigger.isDown()) {
                            if (gamePad2.Left_Trigger.isDown() && gamePad2.Right_Trigger.isDown()) {
                                //robot.bucket.setPosition(bucketVertical);
                                robot.claw.setPosition(clawOpen);
                                telemetry.addLine("CLAW OPENING ggggggggggggggggggggggggggggggggg");
                            } else {
                                if(gamePad2.Right_Trigger.wasJustPressed() || gamePad2.Left_Trigger.wasJustPressed()){
                                    robot.shoulder.setPosition(shoulderPose);
                                }
                            }
                        }
                        if (robot.slide.getCurrentPosition() >= slideBuffer) {
                            robot.bucket.setPosition(bucketPose);
                        }
                    }else{
                        robot.shoulder.setPosition(shoulderPose);
                        if (gamePad2.Left_Trigger.isDown() || gamePad2.Right_Trigger.isDown()) {
                            if (gamePad2.Left_Trigger.isDown() && gamePad2.Right_Trigger.isDown()) {
                                //robot.bucket.setPosition(bucketVertical);
                                robot.claw.setPosition(clawOpen);
                                telemetry.addLine("CLAW OPENING ggggggggggggggggggggggggggggggggg");
                            } else {
                                bucketPose = bucket45Score;
                                robot.bucket.setPosition(bucketPose);
                                shoulderPose = ShoulderBuckets;
                                if(gamePad2.Right_Trigger.wasJustPressed() || gamePad2.Left_Trigger.wasJustPressed()){
                                    slideHeight = slideHeight+10;
                                }
                            }
                        } else {
                            if (robot.slide.getCurrentPosition() >= slideBuffer) {
                                robot.bucket.setPosition(bucketPose);
                                robot.shoulder.setPosition(shoulderPose);
                            }
                        }
                    }


                        if(gamePad2.Left_Trigger.wasJustReleased()|| gamePad2.Right_Trigger.wasJustReleased()){
                            if ((!gamePad2.Left_Trigger.isDown() && !gamePad2.Right_Trigger.isDown())) {
                                robot.IntakeSlideRight.setPosition(intakeSlideMid);
                                robot.IntakeSlideLeft.setPosition(intakeSlideMid);
                                robot.claw.setPosition(clawClose);
                                robot.shoulder.setPosition(ShoulderTransfer);
                                changeStateTo(state.driving);
                            }

                        }
                        //reset encoder
                        /*if(gamepad2.touchpad){
                            robot.slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                            robot.slide2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        }

                         */
                        if(slideHeight <= slideBuffer && !isReady){
                            if(robot.slide.getCurrentPosition() >= slideBuffer-2) {
                                if(robot.stateTime.seconds()>2) {
                                    //robot.shoulder.setPosition(shoulderPose);
                                    isReady = true;
                                }
                            } else {
                                robot.slide.setTargetPosition(slideBuffer);
                                robot.slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                                robot.slide.setPower(1);
                                robot.slide2.setTargetPosition(slideBuffer);
                                robot.slide2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                                robot.slide2.setPower(1);
                                robot.bucket.setPosition(bucketVertical);
                                robot.shoulder.setPosition(ShoulderTransfer);
                                robot.bucket.setPosition(bucketPose);
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

                                if (robot.slide.getCurrentPosition() > slideHeight - 20 && robot.slide.getCurrentPosition() < slideHeight + 20) {
                                    //keep slide up
                                    robot.bucket.setPosition(robot.bucket.getPosition() + (gamepad2.left_stick_x / 50));
                                    bucketPose = robot.bucket.getPosition();
                                    if(robot.stateTime.seconds()>1.25){
                                        robot.shoulder.setPosition(shoulderPose);
                                    }
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
                                }
                            } else { //manual control
                                manualSlide();
                                robot.bucket.setPosition(robot.bucket.getPosition() + (gamepad2.left_stick_x / 50));//changed it from 400; see if it works
                                telemetry.addData("slide", robot.slide.getCurrentPosition());
                            }
                            if (gamePad2.Left_Bumper.wasJustPressed() || gamePad2.Right_Bumper.wasJustPressed()) {
                                if (shoulderPose == ShoulderClips) {
                                    shoulderPose = ShoulderTransfer;
                                } else {
                                    shoulderPose = ShoulderClips;
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
            }

            robot.bucket1.setPosition(robot.bucket.getPosition());

        }
    }
    public void update(){ //place once on start of loop
        //LED lights NEW//////////////////////////////////////////////////////////
        transferred = false;
        if(transferred){
            robot.pattern = RevBlinkinLedDriver.BlinkinPattern.COLOR_WAVES_OCEAN_PALETTE;
        } else{
                robot.pattern = RevBlinkinLedDriver.BlinkinPattern.COLOR_WAVES_LAVA_PALETTE;
        }
        robot.leds.setPattern(robot.pattern);
        //gamePad2.update(gamepad2, robot);
        //gamePad1.update(gamepad1, robot);
        if(gamepad1.share && gamepad1.options){
            robot.kachow.roadRunner.setPose(new Pose(60, 60, Math.toRadians(robot.kachow.roadRunner.getPose().getHeading())));
        }
        isClipping = gamePad2.Guide.getToggleState();
        Precise = gamePad2.TouchPad.getToggleState();


        //Magnet
        if(!gamePad2.justMagnet.isDown()){
            robot.slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.slide2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }

        //Hanging New////////////////////////////////////
        if(gamePad1.Guide.getToggleState()){
            hangPose = hangClipping;
        } else if (gamePad1.Guide.stateJustChanged()){
            hangPose = hangStandby;
        }
            if ((robot.runtime.seconds() > 109) && (robot.runtime.seconds() < 112) && (!gamePad1.Dpad_Up.isDown() && !gamePad1.Dpad_Down.isDown() && !gamePad1.Dpad_Left.isDown() && !gamePad1.Dpad_Right.isDown())) {
                hangPose = hangReady;
            }
            if (gamePad1.Dpad_Up.isDown()) {
                hangPose = hanging;
            }
            if (gamePad1.Dpad_Down.isDown()) {
                hangPose = hangReady;
            }
            if (gamePad1.Dpad_Left.isDown()) {
                hangPose = hangPose - hangIncrements;
            }
            if (gamePad1.Dpad_Right.isDown()) {
                hangPose = hangPose + hangIncrements;
            }

        robot.spinny1.setTargetPosition(hangPose);
        robot.spinny1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.spinny1.setPower(1);
        robot.spinny2.setTargetPosition(hangPose);
        robot.spinny2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.spinny2.setPower(1);

        telemetry.addData("slide: ", robot.slide.getCurrentPosition());
        telemetry.addData("slide2: ", robot.slide2.getCurrentPosition());
        telemetry.addData("Precise: ", Precise);



        /**telemetry.addData("X", robot.kachow.roadRunner.pose.position.x);
        telemetry.addData("y", robot.kachow.roadRunner.pose.position.y);
        telemetry.addData("DEG", Math.toDegrees(robot.kachow.roadRunner.pose.heading.toDouble()));
        telemetry.addLine();
        telemetry.addData("LEDS pattern", robot.pattern);
        telemetry.addData("State", State);
        telemetry.addData("State time", robot.stateTime);
        telemetry.update();*/

    }
    public void manualSlide(){
        if(gamepad2.right_stick_y<-.1 && robot.slide.getCurrentPosition()<=3150) {
            robot.slide.setTargetPosition(3150);
            robot.slide2.setTargetPosition(3150);
            robot.slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.slide2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.slide.setPower(Math.abs(gamepad2.right_stick_y));
            robot.slide2.setPower(Math.abs(gamepad2.right_stick_y));
        } else if(gamepad2.right_stick_y>.1 && robot.slide.getCurrentPosition()>=0) {
            robot.slide.setTargetPosition(0);
            robot.slide2.setTargetPosition(0);
            robot.slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.slide2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.slide.setPower(Math.abs(gamepad2.right_stick_y));
            robot.slide2.setPower(Math.abs(gamepad2.right_stick_y));
        } else {
            robot.slide.setTargetPosition(robot.slide.getCurrentPosition());
            robot.slide2.setTargetPosition(robot.slide.getCurrentPosition());
            robot.slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.slide2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.slide.setPower(1);
            robot.slide2.setPower(1);
        }
    }
    public void changeStateTo(state tostate){
        State = tostate;
        robot.stateUpdate(State);
    }
    public void updateDrive(double DriveSpeed){
        robot.kachow.rrdrive(gamepad1, gamepad2, DriveSpeed, robot);
        robot.stateUpdate(State);
        robot.leds.setPattern(robot.pattern);
        /* Old Hanging
        if(gamePad1.Dpad_Down.isDown()){
            robot.spinny2.setPower(-1);
            robot.spinny1.setPower(-1);
        } else if(gamePad1.Dpad_Up.isDown()){
            robot.spinny2.setPower(1);
            robot.spinny1.setPower(1);
        } else{
            robot.spinny2.setPower(0);
            robot.spinny1.setPower(0);
        }
         */
        /*telemetry.addData("X", robot.kachow.roadRunner.pose.position.x);
        telemetry.addData("y", robot.kachow.roadRunner.pose.position.y);
        telemetry.addData("DEG", Math.toDegrees(robot.kachow.roadRunner.pose.heading.toDouble()));
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
        telemetry.update();*/
        /*telemetry.addData("otos Y: ", robot.otos.getPosition().y);
        telemetry.addData("otos X: ", robot.otos.getPosition().x);
        telemetry.addData("otos Heading: ", robot.otos.getPosition().h);

         */
        telemetry.addData("time: ", robot.runtime.seconds());
        telemetry.addData("state: ", robot.currentState);


        telemetry.update();

    }

}
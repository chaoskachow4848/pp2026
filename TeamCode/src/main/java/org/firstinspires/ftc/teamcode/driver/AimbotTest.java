/* Copyright (c) 2017 FIRST. All rights reserved.
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

package org.firstinspires.ftc.teamcode.driver;

//import com.qualcomm.hardware.rev.Rev2mDistanceSensor;

import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.hardware.KachowHardware;
import org.firstinspires.ftc.teamcode.hardware.Vector2d;
import org.firstinspires.ftc.teamcode.hardware.button;
import org.firstinspires.ftc.teamcode.hardware.kaze;

//@Config
@TeleOp(name="Aimbot Test", group="4848")
//@Disabled
public class AimbotTest extends LinearOpMode {

    double distance;

    button.ButtonReader gamePad1 = new button.ButtonReader();
    button.ButtonReader gamePad2 = new button.ButtonReader();
    /* Declare OpMode members. */
    KachowHardware robot = new KachowHardware();
    ElapsedTime runtime = new ElapsedTime();
    @Override
    public void runOpMode() {

        robot.init(hardwareMap);
        gamepad2.runLedEffect(robot.redled);
        gamepad1.runLedEffect(robot.blueled);
        telemetry.update();
        robot.pattern = RevBlinkinLedDriver.BlinkinPattern.BLACK;
        kaze.init(new Pose(72,72,0));//kaze init before robotinit
        waitForStart();
        runtime.reset();
        Vector2d target = new Vector2d(0,144);

        // run until the end of
        // the match (driver presses STOP)
        while (opModeIsActive()) {
            update();

            //Slide
            //drive_robotCentric(DriveSpeed, SlowModeIsOn);

 /*           double error;
            double errorPower;
            double x;
            double y;
            double turn;
            double heading;

            Vector2d botPoint = new Vector2d(kaze.robotPose.getPose().getX(), kaze.robotPose.getPose().getY());
            Vector2d wallReference = new Vector2d(144, botPoint.y);
            bot_to_target = Math.abs(Math.sqrt((botPoint.x-target.x)*(botPoint.x-target.x) + (botPoint.y-target.y)*(botPoint.y-target.y)));
            bot_to_wall = Math.abs(Math.sqrt(pow(botPoint.x-wallReference.x, 2) + pow(botPoint.y-wallReference.y, 2)));
            wall_to_target = Math.abs(Math.sqrt(pow(wallReference.x-target.x, 2) + pow(wallReference.y-target.y, 2)));

            bot_angle = Math.abs(Math.acos((pow(bot_to_wall,2) + pow(bot_to_target,2) - pow(wall_to_target,2))/
                    (2 * bot_to_wall * bot_to_target)));
            target_angle = Math.abs(Math.asin((bot_to_wall * Math.sin(bot_angle))/wall_to_target));
            wall_angle = Math.abs(Math.toRadians(180) - bot_angle - target_angle);

            if(botPoint.y > target.y){
                bot_angle = -bot_angle;
                target_angle = -target_angle;
                wall_angle = -wall_angle;
            }

            if (robot.runtime.seconds() > 84.8 && robot.runtime.seconds() < 85) {
                gamepad1.rumble(5000);
                gamepad2.rumble(5000);
            }
            if (robot.runtime.seconds() > 109 && robot.runtime.seconds() < 110) {
                gamepad1.rumble(10000);
                gamepad2.rumble(10000);
            }
            heading = robot.kachow.roadRunner.getHeading()+Math.toRadians(kaze.headingOffset);

            x = gamepad1.left_stick_x;
            y = -gamepad1.left_stick_y;

            error = angleDifference(heading, bot_angle);

            errorPower = error / 90;
            if(Math.abs(errorPower)>1){
                errorPower = Math.abs(errorPower)/errorPower;
            }
            if(Math.abs(errorPower)<.1){
                errorPower = (Math.abs(errorPower)/errorPower)*.1;
            }
            if (Math.abs(error) < 1) {
                errorPower = 0;
            }
            turn = -errorPower;


            double botHeading = heading;
            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(turn), 1);
            double frontLeftPower = (rotY + rotX + turn) / denominator;
            double backLeftPower = (rotY - rotX + turn) / denominator;
            double frontRightPower = (rotY - rotX - turn) / denominator;
            double backRightPower = (rotY + rotX - turn) / denominator;


            robot.frontright.setPower(frontRightPower);
            robot.frontleft.setPower(frontLeftPower);
            robot.backleft.setPower(backLeftPower);
            robot.backright.setPower(backRightPower);
            //roadRunner.setPose(new Pose(roadRunner.getPose().getX(), roadRunner.getPose().getY(), Math.toRadians(robot.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES)+kaze.headingOffset)));
            kaze.update(robot.kachow.roadRunner);
*/


            robot.kachow.drive.updatePose();
            robot.kachow.aimbot(target, gamepad1, gamepad2, robot, .13);



            telemetry.addData("heading: ", Math.toDegrees(robot.kachow.drive.getHeading()));
            telemetry.addData("BotAngle: ", Math.toDegrees(robot.kachow.bot_angle));
            telemetry.addData("TargetAngle: ", Math.toDegrees(robot.kachow.target_angle));
            telemetry.addData("Bot to wall: ", robot.kachow.bot_to_wall);
            telemetry.addData("Bot to Target: ", robot.kachow.bot_to_target);
            telemetry.addData("Wall to Target: ", robot.kachow.wall_to_target);
            telemetry.addLine("BotPoint: " + "x:"+robot.kachow.drive.getPose().getX() + "y:"+robot.kachow.drive.getPose().getY());




        }
        }

    public void robot_robotCentric(double DriveSpeed, boolean slow) {
        double FrontLeft;
        double FrontRight;
        double BackLeft;
        double BackRight;
        telemetry.addLine(String.valueOf(distance));
        telemetry.addData(">", runtime.seconds());
        telemetry.update();
        if(runtime.seconds() > 84.8 && runtime.seconds() < 85.2){
            gamepad1.rumble(5000);
            gamepad2.rumble(5000);
        }
        if(runtime.seconds() > 109 && runtime.seconds() < 110){
            gamepad1.rumble(10000);
            gamepad2.rumble(10000);
        }
        //StrafeRight
        if (opModeIsActive()) {
            double x = -gamepad1.left_stick_x;
            double y = gamepad1.left_stick_y;
            double turn = -gamepad1.right_stick_x;
            double theta = Math.atan2(y, x);
            double power = Math.hypot(x, y);

            double sin = Math.sin(theta - Math.PI/4);
            double cos = Math.cos(theta - Math.PI/4);
            double max = Math.max(Math.abs(sin), Math.abs(cos));

            FrontLeft = power * cos/max + turn;
            FrontRight = power * sin/max - turn;
            BackLeft = power * sin/max + turn;
            BackRight = power * cos/max - turn;

            if ((power + Math.abs(turn)) > 1){
                FrontLeft /= power + turn;
                FrontRight /= power + turn;
                BackLeft /= power + turn;
                BackRight /= power + turn;
            }

                robot.backleft.setPower(Range.clip(BackLeft, -DriveSpeed, DriveSpeed));
                robot.backright.setPower(Range.clip(BackRight, -DriveSpeed, DriveSpeed));
                robot.frontleft.setPower(Range.clip(FrontLeft, -DriveSpeed, DriveSpeed));
                robot.frontright.setPower(Range.clip(FrontRight, -DriveSpeed, DriveSpeed));
            }

        }
    public void update(){ //place once on start of loop
        gamePad2.update(gamepad2, robot);
        gamePad1.update(gamepad1, robot);
        if(gamepad1.share && gamepad1.options){
            robot.kachow.drive.setPose(new Pose(60, 60, Math.toRadians(robot.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES))));
        }
//        kaze.update(robot.kachow.roadRunner);
        telemetry.update();
        kaze.drawCurrentAndHistory(robot.kachow.drive);
    }

}


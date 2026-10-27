package org.firstinspires.ftc.teamcode.opmodes.teleops;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.commands.DriveCommand;
import org.firstinspires.ftc.teamcode.commands.IntakeCommand;
import org.firstinspires.ftc.teamcode.commands.ShootAssistCommand;
import org.firstinspires.ftc.teamcode.subsystems.drive.AutoAimController;
import org.firstinspires.ftc.teamcode.subsystems.drive.Drive;
import org.firstinspires.ftc.teamcode.subsystems.intake.Intake;
import org.firstinspires.ftc.teamcode.subsystems.shooter.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.transit.Transit;
import org.firstinspires.ftc.teamcode.subsystems.vision.Alliance;
import org.firstinspires.ftc.teamcode.subsystems.vision.CellCluster;
import org.firstinspires.ftc.teamcode.subsystems.vision.Vision;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

public abstract class TeleOpBase extends CommandOpMode {
    private Drive drive;
    private Shooter shooter;
    private Intake intake;
    private Transit transit;
    private Vision vision;
    private AutoAimController autoAim;
    private ShootAssistCommand shootAssistCommand;
    private boolean started;
    private boolean aimRequested;
    private boolean stopLatched;

    protected abstract Alliance getAlliance();

    protected boolean isAutoAimEnabled() {
        return true;
    }

    protected Vision createVision() {
        return new Vision(hardwareMap, getAlliance());
    }

    @Override
    public void initialize() {
        CommandScheduler.getInstance().reset();
        started = false;
        aimRequested = false;
        stopLatched = false;
        drive = new Drive(hardwareMap);
        shooter = new Shooter(hardwareMap);
        intake = new Intake(hardwareMap);
        transit = new Transit(hardwareMap);
        vision = createVision();
        autoAim = new AutoAimController();
        GamepadEx gamepadEx = new GamepadEx(gamepad1);

        shootAssistCommand = new ShootAssistCommand(
                drive,
                shooter,
                intake,
                transit,
                vision,
                autoAim,
                gamepadEx,
                isAutoAimEnabled()
        );
        drive.setDefaultCommand(new DriveCommand(drive, gamepadEx));
        intake.setDefaultCommand(new IntakeCommand(intake, transit, gamepadEx));
        shooter.setDefaultCommand(new RunCommand(() -> {
            if (gamepad1.left_bumper) {
                shooter.stop();
            } else {
                shooter.idle();
            }
        }, shooter));

        // The scheduler refreshes sensors before buttons and command execution.
        CommandScheduler.getInstance().addButton(() -> vision.updateAimTarget(
                drive, stopLatched || gamepad1.b, aimRequested));

        new Trigger(
                () -> gamepadEx.getButton(GamepadKeys.Button.LEFT_STICK_BUTTON)
        ).whenActive(drive::resetDriverHeading);

        new Trigger(
                () -> gamepadEx.getButton(GamepadKeys.Button.RIGHT_BUMPER)
        ).whenActive(intake::toggleFlower);

        new Trigger(
                () -> gamepadEx.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER)
                        > TeleOpConstants.triggerThreshold
        ).whileActiveContinuous(shootAssistCommand);
    }

    @Override
    public void run() {
        aimRequested = gamepad1.right_trigger > TeleOpConstants.triggerThreshold;

        if (gamepad1.b) {
            stopLatched = true;
        } else if (!aimRequested) {
            stopLatched = false;
        }

        if (stopLatched) {
            shootAssistCommand.cancel();
            autoAim.reset();
            stopMechanisms();
            drive.periodic();
            vision.update();
            vision.updateAimTarget(drive, true, aimRequested);
        } else {
            super.run();
        }

        showTelemetry();
    }

    private void showTelemetry() {
        Pose2D pose = drive.getPose();
        boolean located = drive.isFieldPoseInitialized()
                && Double.isFinite(pose.getX(DistanceUnit.INCH))
                && Double.isFinite(pose.getY(DistanceUnit.INCH))
                && Double.isFinite(pose.getHeading(AngleUnit.DEGREES));
        if (isAutoAimEnabled()) {
            double yawError = vision.getYawErrorDegrees();
            double headingVelocity = drive.getHeadingVelocityDegreesPerSecond();
            telemetry.addData("X/Y/H", "%.1f cm / %.1f cm / %+.1f°",
                    pose.getX(DistanceUnit.CM), pose.getY(DistanceUnit.CM),
                    pose.getHeading(AngleUnit.DEGREES));
            telemetry.addData("程序应转", !vision.hasValidAimingTarget() || !Double.isFinite(yawError)
                    ? "无有效目标" : yawError > 0.0 ? "左" : yawError < 0.0 ? "右" : "无需转向");
            telemetry.addData("实际转向（IMU）", !Double.isFinite(headingVelocity) ? "未知"
                    : headingVelocity > 0.0 ? "左" : headingVelocity < 0.0 ? "右" : "未转动");
            telemetry.update();
            return;
        }
        CellCluster expected = located ? CellCluster.forFieldX(getAlliance(), pose.getX(DistanceUnit.INCH)).orElse(null) : null;
        CellCluster selected = vision.getSelectedCluster().orElse(null);
        String target = cellLabel(expected);
        if (selected != null && selected != expected) {
            target += "；当前选择：" + cellLabel(selected);
        }
        telemetry.addData("应瞄准框", target);
        boolean aligned = isAutoAimEnabled() && started && !stopLatched && aimRequested
                && vision.hasValidAimingTarget() && autoAim.isLocked();
        String aimState = !started ? "等待START" : stopLatched ? "已停止"
                : !isAutoAimEnabled() ? "手动瞄准（右摇杆转向，R2射击）"
                : !aimRequested ? "未启动（按R2瞄准）"
                : !vision.hasValidAimingTarget() ? "无有效瞄准目标"
                : aligned ? "已对准" : "调整中";
        if (isAutoAimEnabled() && started && aimRequested && !stopLatched && vision.hasValidAimingTarget()) {
            aimState += String.format(java.util.Locale.US, "（误差 %.1f°）",vision.getYawErrorDegrees());
        }
        telemetry.addData("瞄准", aimState);
        boolean allowed = started && !stopLatched && aimRequested && shootAssistCommand.canFire();
        telemetry.addData("允许射击", allowed ? "是" : "否：" + shootingBlockReason());
        // The same fused Pinpoint field pose used for CELL selection and aiming.
        telemetry.addData("场地坐标（中心0/0，X朝观众）", "X %.1f cm / Y %.1f cm / 朝向 %.1f°（%s）",
                pose.getX(DistanceUnit.CM),pose.getY(DistanceUnit.CM),pose.getHeading(AngleUnit.DEGREES),
                located ? "已校准" : "未校准，尚非可靠场地坐标");
        telemetry.addData("校准", vision.getCalibrationStatus());
        telemetry.addData("校准等待", "%.2f s", vision.getCalibrationCooldownRemainingSeconds());
        double shotDistanceCm = DistanceUnit.INCH.toCm(vision.getTargetHorizontalDistanceInches());
        telemetry.addData("射击距离（到当前目标框）", "%.1f cm", shotDistanceCm);
        if (located && vision.hasValidAimingTarget()
                && Double.isFinite(shotDistanceCm) && shotDistanceCm > 0.0) {
            telemetry.addData("在center位置抄这一行", String.format(java.util.Locale.US,
                    "centerDistanceCm = %.2f;", shotDistanceCm));
            telemetry.addData("在wall位置抄这一行", String.format(java.util.Locale.US,
                    "wallDistanceCm = %.2f;", shotDistanceCm));
            telemetry.addLine("距离标定：停车并松开RT，抄对应位置的一行（单位cm）。");
        } else {
            telemetry.addData("距离标定", "等待有效场地定位，暂不能填写距离");
        }
        telemetry.addLine("center/wall的目标转速（ticks/s）和Pitch：在ShooterConstants填写，用Shooter Test验证。");
        telemetry.addData("两点射击参数", Shooter.areDistanceShotValuesValid() ? "已填写" : "未填完整或超出限值");
        telemetry.addData("射手转速", "%.0f / %.0f ticks/s",
                shooter.getLeftVelocity(), shooter.getRightVelocity());
        telemetry.addData("射手目标转速", "%.0f ticks/s", shooter.getTargetVelocity());
        telemetry.addData("射手模式/功率", "%s / %.2f / %.2f",
                shooter.getControlMode(), shooter.getLeftPower(), shooter.getRightPower());
        telemetry.addData("Pitch位置", "%.3f", shooter.getPitchPosition());
        telemetry.update();
    }

    private static String cellLabel(CellCluster cell) {
        if (cell == null) return "未知（等待位置校准）";
        switch (cell) {
            case RED_AUDIENCE: return "红方·观众侧框（34–37）";
            case RED_OPPOSITE_AUDIENCE: return "红方·远观众侧框（30–33）";
            case BLUE_AUDIENCE: return "蓝方·观众侧框（38–41）";
            case BLUE_OPPOSITE_AUDIENCE: return "蓝方·远观众侧框（42–45）";
            default: return "未知";
        }
    }

    private String shootingBlockReason() {
        if (!started) return "等待START";
        if (stopLatched) return "B已停止";
        if (!aimRequested) return "未按R2";
        if (gamepad1.left_bumper) return "正在排堵";
        if (!Shooter.areDistanceShotValuesValid()) return "两点射击参数未填完整或超出限值";
        if (!vision.hasValidAimingTarget()) return "定位或瞄准目标无效";
        if (shootAssistCommand.getFiringState() == Transit.FiringState.REVERSING)
            return "已放行，送球前反转中";
        if (!shooter.isReady()) return "射手转速尚未稳定";
        if (isAutoAimEnabled() && !autoAim.isLocked()) return "尚未对准";
        if (!shooter.isPitchReady()) return "射手角度未就绪";
        if (!transit.areLimitPositionsCalibrated()) return "挡板位置未校准";
        return "等待射击状态就绪";
    }

    private void stopMechanisms() {
        if (drive != null) drive.stop();
        if (shooter != null) shooter.stop();
        if (intake != null) intake.stop();
        if (transit != null) {
            transit.cancelFeeding();
        }
    }

    @Override
    public void runOpMode() throws InterruptedException {
        try {
            initialize();
            while (opModeInInit() && !isStopRequested()) {
                drive.periodic();
                vision.update();
                vision.updateAimTarget(drive, gamepad1.b,
                        gamepad1.right_trigger > TeleOpConstants.triggerThreshold);
                showTelemetry();
                idle();
            }
            if (isStopRequested()) {
                return;
            }
            waitForStart();
            drive.periodic();
            drive.resetDriverHeading();
            started = true;
            while (!isStopRequested() && opModeIsActive()) {
                run();
            }
        } finally {
            if (shootAssistCommand != null) shootAssistCommand.cancel();
            reset();
            stopMechanisms();
            if (intake != null) intake.raiseFlower();
            if (vision != null) vision.stop();
        }
    }
}

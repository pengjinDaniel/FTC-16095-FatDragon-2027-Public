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
import org.firstinspires.ftc.teamcode.subsystems.drive.DriveConstants;
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
    private static final long AUTO_AIM_DISABLE_HOLD_NANOS = 3_000_000_000L;

    private Drive drive;
    private Shooter shooter;
    private Intake intake;
    private Transit transit;
    private Vision vision;
    private AutoAimController autoAim;
    private ShootAssistCommand shootAssistCommand;
    private GamepadEx gamepadEx2;
    private long autoAimDisableHoldStartedAtNanos;
    private boolean waitForAimDisableRelease;
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
        autoAimDisableHoldStartedAtNanos = -1;
        waitForAimDisableRelease = false;
        drive = new Drive(hardwareMap,
                isAutoAimEnabled() && DriveConstants.fusedTeleOpLocalizationEnabled);
        shooter = new Shooter(hardwareMap);
        intake = new Intake(hardwareMap);
        transit = new Transit(hardwareMap);
        vision = createVision();
        autoAim = new AutoAimController();
        GamepadEx gamepadEx = new GamepadEx(gamepad1);
        gamepadEx2 = new GamepadEx(gamepad2);

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
            if (gamepad1.right_bumper || !vision.hasValidAimingTarget()) {
                shooter.stop();
            } else {
                shooter.setDistanceTarget(
                        DistanceUnit.INCH.toCm(vision.getTargetHorizontalDistanceInches()),
                        false); // Pre-spin only; pitch follows the shot when RT is held.
            }
        }, shooter));

        // The scheduler refreshes sensors before buttons and command execution.
        CommandScheduler.getInstance().addButton(() -> vision.updateAimTarget(
                drive, stopLatched || gamepad1.b, aimRequested));
        CommandScheduler.getInstance().addButton(() -> intake.setReverseRequested(
                gamepadEx.getButton(GamepadKeys.Button.LEFT_BUMPER)));

        new Trigger(
                () -> gamepadEx.getButton(GamepadKeys.Button.LEFT_STICK_BUTTON)
        ).whenActive(drive::resetDriverHeading);

        new Trigger(
                () -> gamepadEx.getButton(GamepadKeys.Button.RIGHT_STICK_BUTTON)
        ).whenActive(intake::toggleFlower);

        new Trigger(() -> aimRequested).whileActiveContinuous(shootAssistCommand);
    }

    @Override
    public void run() {
        if (started) updateSecondGamepad(System.nanoTime());
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

    private boolean areSecondGamepadTriggersHeld() {
        return gamepadEx2.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > TeleOpConstants.triggerThreshold
                && gamepadEx2.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > TeleOpConstants.triggerThreshold;
    }

    private void startSecondGamepad() {
        // Treat inputs already held at START as the initial state.
        gamepadEx2.readButtons();
        waitForAimDisableRelease = areSecondGamepadTriggersHeld();
    }

    private void updateSecondGamepad(long nowNanos) {
        gamepadEx2.readButtons();
        int velocitySteps = (gamepadEx2.wasJustPressed(GamepadKeys.Button.DPAD_UP) ? 1 : 0)
                - (gamepadEx2.wasJustPressed(GamepadKeys.Button.DPAD_DOWN) ? 1 : 0);
        if (velocitySteps != 0) shooter.adjustDistanceVelocityOffset(10.0 * velocitySteps);

        int headingSteps = (gamepadEx2.wasJustPressed(GamepadKeys.Button.DPAD_LEFT) ? 1 : 0)
                - (gamepadEx2.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT) ? 1 : 0);
        if (headingSteps != 0) shootAssistCommand.adjustAimHeadingOffset(headingSteps);

        if (!areSecondGamepadTriggersHeld()) {
            autoAimDisableHoldStartedAtNanos = -1;
            waitForAimDisableRelease = false;
        } else if (shootAssistCommand.isAutoAimEnabled() && !waitForAimDisableRelease) {
            if (autoAimDisableHoldStartedAtNanos == -1) {
                autoAimDisableHoldStartedAtNanos = nowNanos;
            } else if (nowNanos - autoAimDisableHoldStartedAtNanos >= AUTO_AIM_DISABLE_HOLD_NANOS) {
                shootAssistCommand.disableAutoAim();
            }
        }
    }

    private void showTelemetry() {
        Pose2D pose = drive.getPose();
        boolean located = drive.isFieldPoseInitialized()
                && Double.isFinite(pose.getX(DistanceUnit.INCH))
                && Double.isFinite(pose.getY(DistanceUnit.INCH))
                && Double.isFinite(pose.getHeading(AngleUnit.DEGREES));
        boolean autoAimEnabled = shootAssistCommand.isAutoAimEnabled();
        boolean aligned = autoAimEnabled && started && !stopLatched && aimRequested
                && vision.hasValidAimingTarget() && vision.isAimCalibrationReady() && autoAim.isLocked();
        String aimState = !started ? "等待START" : stopLatched ? "已停止"
                : !autoAimEnabled ? "手动瞄准（右摇杆转向，R2射击）"
                : !aimRequested ? "未启动（按R2瞄准）"
                : !vision.isAimCalibrationReady() ? vision.getRtCalibrationStatus()
                : !vision.hasValidAimingTarget() ? "无有效瞄准目标"
                : aligned ? "已对准并稳定" : "未对准或尚未稳定";
        if (autoAimEnabled && started && aimRequested && !stopLatched && vision.hasValidAimingTarget()) {
            aimState += String.format(java.util.Locale.US, "（误差 %.1f° / 放行 ±%.1f°）",
                    shootAssistCommand.getAdjustedYawErrorDegrees(), DriveConstants.aimLockToleranceDegrees);
        }
        telemetry.addData("瞄准", aimState);
        telemetry.addData("手柄2微调", "自瞄%s / 转速 %+.0f ticks/s / 瞄准 %+.0f°（左正）",
                autoAimEnabled ? "开" : "关", shooter.getDistanceVelocityOffsetTicksPerSecond(),
                shootAssistCommand.getAimHeadingOffsetDegrees());
        boolean allowed = started && !stopLatched && aimRequested && shootAssistCommand.canFire();
        telemetry.addData("允许射击", allowed ? "是" : "否：" + shootingBlockReason());
        boolean shooterHasTarget = shooter.hasTargetVelocity() && Double.isFinite(shooter.getTargetVelocity());
        String shooterTarget = shooterHasTarget
                ? String.format(java.util.Locale.US, "%.0f", shooter.getTargetVelocity()) : "未设置";
        telemetry.addData("射手（ticks/s）", "左 %.0f / 右 %.0f / 目标 %s（%s）",
                shooter.getLeftVelocity(), shooter.getRightVelocity(), shooterTarget,
                !shooterHasTarget ? "未设目标" : shooter.isReady() ? "已达速" : "未达速");
        if (isAutoAimEnabled()) {
            Pose2D visualPose = vision.getCorrectedRobotFieldPose().orElse(null);
            String visualCoordinates = visualPose == null ? vision.getPoseStatus()
                    : String.format(java.util.Locale.US, "X %.1f / Y %.1f cm / H %+.1f°",
                    visualPose.getX(DistanceUnit.CM), visualPose.getY(DistanceUnit.CM),
                    visualPose.getHeading(AngleUnit.DEGREES));
            String poseDifference = !located || visualPose == null ? "无法比较"
                    : String.format(java.util.Locale.US, "ΔX %+.1f / ΔY %+.1f cm / ΔH %+.1f°",
                    pose.getX(DistanceUnit.CM) - visualPose.getX(DistanceUnit.CM),
                    pose.getY(DistanceUnit.CM) - visualPose.getY(DistanceUnit.CM),
                    Math.IEEEremainder(pose.getHeading(AngleUnit.DEGREES)
                            - visualPose.getHeading(AngleUnit.DEGREES), 360.0));
            Pose2D rawOdometry = drive.getRawOdometryPose();
            boolean finiteOdometry = rawOdometry != null
                    && Double.isFinite(rawOdometry.getX(DistanceUnit.CM))
                    && Double.isFinite(rawOdometry.getY(DistanceUnit.CM))
                    && Double.isFinite(rawOdometry.getHeading(AngleUnit.DEGREES));
            String odometryCoordinates = !finiteOdometry ? "无有效设备坐标"
                    : String.format(java.util.Locale.US, "X %.1f / Y %.1f cm / H %+.1f°",
                    rawOdometry.getX(DistanceUnit.CM), rawOdometry.getY(DistanceUnit.CM),
                    rawOdometry.getHeading(AngleUnit.DEGREES));
            telemetry.addData("OD", "%s（%s）", odometryCoordinates, located ? "场地有效" : "场地不可用");
            telemetry.addData("LL", visualCoordinates);
            telemetry.addData("OD−LL", poseDifference);
            telemetry.addData("Pinpoint读包", drive.getPinpointReadDiagnostics());
            telemetry.addData("上次坏读", drive.getLastPinpointReadFailure());
            telemetry.addData("定位", drive.getLocalizationStatus());
            telemetry.addData("视觉校准", vision.getCalibrationStatus());
            if (autoAimEnabled && aimRequested) {
                double yawError = shootAssistCommand.getAdjustedYawErrorDegrees();
                double headingVelocity = drive.getHeadingVelocityDegreesPerSecond();
                String measuredTurn = Double.isFinite(headingVelocity)
                        ? String.format(java.util.Locale.US, "%+.1f°/s", headingVelocity) : "IMU不可用";
                telemetry.addData("转向", "目标%s / 输出%+.3f / IMU %s",
                        !vision.hasValidAimingTarget() || !Double.isFinite(yawError) ? "不可用"
                                : yawError > 0 ? "左" : yawError < 0 ? "右" : "无需转动",
                        autoAim.getRawTurnPower(), measuredTurn);
                if (!vision.hasValidAimingTarget()) telemetry.addData("瞄准目标", vision.getTargetStatus());
            }
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
        // The field pose used for CELL selection and aiming.
        telemetry.addData("场地坐标（中心0/0，X朝观众）", located
                ? String.format(java.util.Locale.US, "X %.1f cm / Y %.1f cm / 朝向 %.1f°",
                        pose.getX(DistanceUnit.CM), pose.getY(DistanceUnit.CM), pose.getHeading(AngleUnit.DEGREES))
                : "等待有效场地定位");
        telemetry.addData("校准", vision.getCalibrationStatus());
        if (drive.usesPoseFusion()) {
            telemetry.addData("融合定位", drive.getPoseFusionDiagnostics());
        }
        double shotDistanceCm = DistanceUnit.INCH.toCm(vision.getTargetHorizontalDistanceInches());
        telemetry.addData("射击距离（车中心到CELL开口中心）", Double.isFinite(shotDistanceCm)
                ? String.format(java.util.Locale.US, "%.1f cm", shotDistanceCm) : "等待有效距离");
        if (located && vision.hasValidAimingTarget()
                && Double.isFinite(shotDistanceCm) && shotDistanceCm > 0.0) {
            telemetry.addData("停车后记录标定距离", "%.2f cm", shotDistanceCm);
        } else {
            telemetry.addData("距离标定", "等待有效场地定位，暂不能填写距离");
        }
        telemetry.addLine("在ShooterConstants的nearShotPoints或farShotPoints填写距离、转速和Pitch。");
        telemetry.addData("射击标定表", Shooter.areDistanceShotValuesValid() ? "已填写" : "未填完整或超出限值");
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
        if (gamepad1.right_bumper) return "正在排堵";
        if (!Shooter.areDistanceShotValuesValid()) return "射击标定表未填完整或超出限值";
        boolean autoAimEnabled = shootAssistCommand.isAutoAimEnabled();
        if (autoAimEnabled && !vision.isAimCalibrationReady())
            return vision.getRtCalibrationStatus();
        if (!vision.hasValidAimingTarget()) return "定位或瞄准目标无效";
        if (shootAssistCommand.getFiringState() == Transit.FiringState.REVERSING)
            return "已放行，送球前反转中";
        if (!shooter.isReady()) return "射手转速尚未稳定";
        if (autoAimEnabled && !autoAim.isLocked()) return "尚未对准";
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
            startSecondGamepad();
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

package org.firstinspires.ftc.teamcode

import com.acmerobotics.dashboard.config.Config
import com.qualcomm.robotcore.hardware.Gamepad
import com.qualcomm.robotcore.hardware.HardwareMap
import com.seattlesolvers.solverslib.gamepad.GamepadEx
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.alonlib.units.Alliance
import org.firstinspires.ftc.teamcode.subsystems.Shooter.ShooterSubsystem
import org.firstinspires.ftc.teamcode.subsystems.vision.ApriltagCamera

@Config

class RobotContainer(
    val hardwareMap: HardwareMap,
    val telemetry: Telemetry,
    val alliance: Alliance,
    gamepad1: Gamepad,
    gamepad2: Gamepad
) {
    // --- Controller decleration ---
    val controllerA = GamepadEx(gamepad1)

    val controllerB = GamepadEx(gamepad2)

    // --- Subsystem decleration
    lateinit var apriltagCamera: ApriltagCamera
    lateinit var shooter: ShooterSubsystem

    init {
        initializeSubsystems()
        configureButtonBindings()
        setDefaultCommands()
    }


    fun initializeSubsystems() {
        apriltagCamera = ApriltagCamera(hardwareMap, telemetry, alliance)
        shooter = ShooterSubsystem(hardwareMap,telemetry,alliance)
    }

    fun configureButtonBindings() {
        with(controllerA) {

        }

    }

    fun setDefaultCommands() {
        shooter.defaultCommand =
    }

    companion object {
        @JvmField
        var input = 0.5
    }

}
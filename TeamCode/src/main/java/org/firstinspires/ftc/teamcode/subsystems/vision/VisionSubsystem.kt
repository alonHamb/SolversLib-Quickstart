package org.firstinspires.ftc.teamcode.subsystems.vision

import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName
import org.firstinspires.ftc.robotcore.external.navigation.Position
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles
import org.firstinspires.ftc.teamcode.RobotMap.Vision.WEBCAM_ID
import org.firstinspires.ftc.teamcode.subsystems.vision.VisionConstants.CAMERA_ANGLES
import org.firstinspires.ftc.teamcode.subsystems.vision.VisionConstants.CAMERA_POSITION
import org.firstinspires.ftc.vision.VisionPortal
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor


class ApriltagCamera(hardwareMap: HardwareMap, telemetry: Telemetry) {
    val cameraPosition: Position = CAMERA_POSITION
    val cameraAngles: YawPitchRollAngles = CAMERA_ANGLES
    val processor: AprilTagProcessor = AprilTagProcessor.Builder().setCameraPose(cameraPosition, cameraAngles).build()
    val builder: VisionPortal.Builder = VisionPortal.Builder()


    init {

        builder.setCamera(hardwareMap.get(WebcamName::class.java, WEBCAM_ID))

        builder.addProcessor(processor)

        val portal: VisionPortal = builder.build()
    }

    var currentDetections
        get() = null
        set(value) {
            processor.detections
        }


}
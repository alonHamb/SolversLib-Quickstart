package org.firstinspires.ftc.teamcode.subsystems.vision

import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.util.Range
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName
import org.firstinspires.ftc.teamcode.RobotMap.Vision.WEBCAM_ID
import org.firstinspires.ftc.teamcode.alonlib.units.Alliance
import org.firstinspires.ftc.vision.VisionPortal
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor
import java.net.IDN
import java.util.EnumSet.range


class ApriltagCamera(hardwareMap: HardwareMap, telemetry: Telemetry, alliance: Alliance) {
    val processor: AprilTagProcessor = AprilTagProcessor.Builder().build()
    val builder: VisionPortal.Builder = VisionPortal.Builder().apply {
        addProcessor(processor)
        setCamera(hardwareMap.get(WebcamName::class.java, WEBCAM_ID))

    }
    val portal: VisionPortal = builder.build()

    val detectedTags: ArrayList<AprilTagDetection> get() = processor.detections

    var detectedTagIds = listOf<Int>()
        get() {detectedTags.forEach {
            field +=(it.id)
        }


    fun angleToGoalTag(){
        if (detectedTags)
    }


}
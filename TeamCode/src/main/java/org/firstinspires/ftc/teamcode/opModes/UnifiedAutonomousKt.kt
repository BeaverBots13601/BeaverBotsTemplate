package org.firstinspires.ftc.teamcode.opModes

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.SequentialAction
import com.acmerobotics.roadrunner.Vector2d
import com.acmerobotics.roadrunner.ftc.runBlocking
import com.qualcomm.hardware.limelightvision.LLResultTypes.FiducialResult
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.robotcontroller.teamcode.TeamColor
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive
import org.firstinspires.ftc.teamcode.sensor.LimelightKt
import org.firstinspires.ftc.teamcode.sensor.SensorDeviceKt
import kotlin.math.PI

@Autonomous(name="Automatic Autonomous", group="Competition")
@Disabled
open class UnifiedAutonomousKt : LinearOpMode() {
    //private val propLocation: WebcamPropIdentificationKt.PropLocation
    //private val propId: WebcamPropIdentificationKt

    // Subclass variables
    protected open val pathToFollow = Path.Standard
    protected open val currentLocation = Locations.Unknown

    @Suppress("PROPERTY_HIDES_JAVA_FIELD") // intentional
    private val telemetry = MultipleTelemetry(super.telemetry, FtcDashboard.getInstance().telemetry)
    private val roadrunnerDrive by lazy { MecanumDrive(hardwareMap, currentLocation.startPose) }
    private val limelight by lazy { LimelightKt.getInstance(hardwareMap, SensorDeviceKt.SensorInitData(
        currentLocation.teamColor,
        FtcDashboard.getInstance().isEnabled,
    ), telemetry) ?: throw Exception() }

    override fun runOpMode() {
        initBulkReads(hardwareMap)
        blackboard.remove("robotHeading")

        // Example autonomous code that can be used. Don't be afraid to expand or remodel it as needed

        /*if(currentLocation == Locations.Unknown) {
            // limelight apriltag
            var tags = limelight.poll()
            var iterations = 0
            while (tags?.isEmpty() ?: false && iterations < 500) {
                sleep(10)
                iterations++
                tags = limelight.poll()
            }

            var importantTag: FiducialResult? = null
            for (tag in tags ?: emptyList()) {
                if(tag.fiducialId == 12 || tag.fiducialId == 15) importantTag = tag; break
            }

            if (importantTag == null) {
                // Panic case
            } else if(importantTag.fiducialId == 12) {
                // Do something based off the tag
            } else if(importantTag.fiducialId == 15) {
                // Do something different with the information
            }
        }*/

        val originTAB = roadrunnerDrive.actionBuilder(currentLocation.startPose)

        val path = originTAB
            .strafeTo(Vector2d(-6.0, -30.5))

        val path2 = path
            .strafeToLinearHeading(Vector2d(-12.0, -31.0), PI)

        val path3 = path
            .strafeToLinearHeading(Vector2d(-12.0, -31.0), -PI)

        val action = when(currentLocation) {
            Locations.BlueFar,
            Locations.BlueClose -> {
                SequentialAction(
                    path.build(),
                    path2.build(),
                )
            }
            Locations.RedFar,
            Locations.RedClose -> {
                SequentialAction(
                    path.build(),
                    path3.build(),
                )
            }
            else -> { return }
        }

        telemetry.addData("INIT STATUS", "READY")
        telemetry.update()

        waitForStart() // setup done actually do things

        runBlocking(action)

        blackboard["robotHeading"] = roadrunnerDrive.localizer.pose.heading.toDouble()
    }

    protected enum class Locations(val teamColor: TeamColor, val startPose: Pose2d) {
        BlueClose(TeamColor.BLUE, Pose2d(0.0, 0.0, 0.0)),
        BlueFar(TeamColor.BLUE, Pose2d(0.0, 0.0, 0.0)),
        RedClose(TeamColor.RED, Pose2d(0.0, 0.0, 0.0)),
        RedFar(TeamColor.RED, Pose2d(0.0, 0.0, 0.0)),
        Unknown(TeamColor.UNKNOWN, Pose2d(0.0, 0.0, 0.0))
    }

    protected enum class Path {
        Standard,
        Alternate,
    }
}

@Autonomous(name = "Blue Close Autonomous", group = "Competition")
class BlueCloseAutonomousKt : UnifiedAutonomousKt() {
    override val currentLocation = Locations.BlueClose
}

@Autonomous(name = "Blue Far Autonomous", group = "Competition")
class BlueFarAutonomousKt : UnifiedAutonomousKt() {
    override val currentLocation = Locations.BlueFar
}

@Autonomous(name = "Red Close Autonomous", group = "Competition")
class RedCloseAutonomousKt : UnifiedAutonomousKt() {
    override val currentLocation = Locations.RedClose
}

@Autonomous(name = "Red Far Autonomous", group = "Competition")
class RedFarAutonomousKt : UnifiedAutonomousKt() {
    override val currentLocation = Locations.RedFar
}

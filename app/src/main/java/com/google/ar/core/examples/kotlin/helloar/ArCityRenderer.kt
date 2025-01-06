/*
 * Copyright 2021 Google LLC
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
package com.google.ar.core.examples.kotlin.helloar

import android.opengl.GLES30
import android.opengl.Matrix
import android.util.Log
import androidx.lifecycle.DefaultLifecycleObserver
import androidx.lifecycle.LifecycleOwner
import com.google.ar.core.Anchor
import com.google.ar.core.Camera
import com.google.ar.core.Frame
import com.google.ar.core.InstantPlacementPoint
import com.google.ar.core.LightEstimate
import com.google.ar.core.Plane
import com.google.ar.core.Pose
import com.google.ar.core.Session
import com.google.ar.core.Trackable
import com.google.ar.core.TrackingFailureReason
import com.google.ar.core.TrackingState
import com.google.ar.core.examples.java.common.helpers.DisplayRotationHelper
import com.google.ar.core.examples.java.common.helpers.TrackingStateHelper
import com.google.ar.core.examples.java.common.samplerender.Framebuffer
import com.google.ar.core.examples.java.common.samplerender.GLError
import com.google.ar.core.examples.java.common.samplerender.Mesh
import com.google.ar.core.examples.java.common.samplerender.SampleRender
import com.google.ar.core.examples.java.common.samplerender.Shader
import com.google.ar.core.examples.java.common.samplerender.Texture
import com.google.ar.core.examples.java.common.samplerender.VertexBuffer
import com.google.ar.core.examples.java.common.samplerender.arcore.BackgroundRenderer
import com.google.ar.core.examples.java.common.samplerender.arcore.PlaneRenderer
import com.google.ar.core.examples.java.common.samplerender.arcore.SpecularCubemapFilter
import com.google.ar.core.exceptions.CameraNotAvailableException
import com.google.ar.core.exceptions.NotYetAvailableException
import java.io.IOException
import java.nio.ByteBuffer
import kotlin.random.Random


/** Renders the HelloAR application using our example Renderer. */
class ArCityRenderer(val activity: ArCityActivity) :
    SampleRender.Renderer, DefaultLifecycleObserver {
    companion object {
        val TAG = "ArCityRenderer"

        // See the definition of updateSphericalHarmonicsCoefficients for an explanation of these
        // constants.
        private val sphericalHarmonicFactors =
            floatArrayOf(
                0.282095f,
                -0.325735f,
                0.325735f,
                -0.325735f,
                0.273137f,
                -0.273137f,
                0.078848f,
                -0.273137f,
                0.136569f
            )

        private val Z_NEAR = 0.1f
        private val Z_FAR = 100f

        // Assumed distance from the device camera to the surface on which user will try to place
        // objects.
        // This value affects the apparent scale of objects while the tracking method of the
        // Instant Placement point is SCREENSPACE_WITH_APPROXIMATE_DISTANCE.
        // Values in the [0.2, 2.0] meter range are a good choice for most AR experiences. Use lower
        // values for AR experiences where users are expected to place objects on surfaces close to the
        // camera. Use larger values for experiences where the user will likely be standing and trying
        // to
        // place an object on the ground or floor in front of them.
        val APPROXIMATE_DISTANCE_METERS = 2.0f

        val CUBEMAP_RESOLUTION = 16
        val CUBEMAP_NUMBER_OF_IMPORTANCE_SAMPLES = 32

        // Custom Settings
        val TARGET_PLANE_AREA: Float = 10f
        val GRID_SIZE: Int = 20
        val SPACING: Float = 0.25f
        val MIN_SCALE_FACTOR: Float = 1f
        val MAX_SCALE_FACTOR: Float = 2.5f

        val MAX_AR_PAWN_ANCHOR_COUNT: Int = 5
        //
    }

    lateinit var render: SampleRender
    lateinit var planeRenderer: PlaneRenderer
    lateinit var backgroundRenderer: BackgroundRenderer
    lateinit var virtualSceneFramebuffer: Framebuffer
    var hasSetTextureNames = false

    // Point Cloud
    lateinit var pointCloudVertexBuffer: VertexBuffer
    lateinit var pointCloudMesh: Mesh
    lateinit var pointCloudShader: Shader

    // Keep track of the last point cloud rendered to avoid updating the VBO if point cloud
    // was not changed.  Do this using the timestamp since we can't compare PointCloud objects.
    var lastPointCloudTimestamp: Long = 0

    // Virtual object (ARCore pawn)
    lateinit var virtualObject: ArSceneObject

    // Cube object (ARCore cube)
    lateinit var cubeObjectMesh: Mesh
    lateinit var cubeObjectShader: Shader

    private val customShaders = mutableListOf<Shader>()
    private val generatedShaders = mutableListOf<Shader>()

    private val cubeObjects = mutableListOf<CubeObject>()

    private val wrappedAnchors = mutableListOf<WrappedAnchor>()
    private val cubeAnchors = mutableListOf<CubeAnchor>()

    // Environmental HDR
    lateinit var dfgTexture: Texture
    lateinit var cubemapFilter: SpecularCubemapFilter

    // Temporary matrix allocated here to reduce number of allocations for each frame.
    val modelMatrix = FloatArray(16)
    val viewMatrix = FloatArray(16)
    val projectionMatrix = FloatArray(16)
    val modelViewMatrix = FloatArray(16) // view x model

    val modelViewProjectionMatrix = FloatArray(16) // projection x view x model

    val sphericalHarmonicsCoefficients = FloatArray(9 * 3)
    val viewInverseMatrix = FloatArray(16)
    val worldLightDirection = floatArrayOf(0.0f, 0.0f, 0.0f, 0.0f)
    val viewLightDirection = FloatArray(4) // view x world light direction

    val session
        get() = activity.arCoreSessionHelper.session

    val displayRotationHelper = DisplayRotationHelper(activity)
    val trackingStateHelper = TrackingStateHelper(activity)

    val horizontalPlanes: MutableList<Plane> = mutableListOf()

    override fun onResume(owner: LifecycleOwner) {
        displayRotationHelper.onResume()
        hasSetTextureNames = false
    }

    override fun onPause(owner: LifecycleOwner) {
        displayRotationHelper.onPause()
    }

    override fun onSurfaceCreated(render: SampleRender) {
        // Prepare the rendering objects.
        // This involves reading shaders and 3D model files, so may throw an IOException.
        try {
            planeRenderer = PlaneRenderer(render)
            backgroundRenderer = BackgroundRenderer(render)
            virtualSceneFramebuffer = Framebuffer(render, /*width=*/ 1, /*height=*/ 1)

            cubemapFilter =
                SpecularCubemapFilter(
                    render,
                    CUBEMAP_RESOLUTION,
                    CUBEMAP_NUMBER_OF_IMPORTANCE_SAMPLES
                )
            // Load environmental lighting values lookup table
            dfgTexture =
                Texture(
                    render,
                    Texture.Target.TEXTURE_2D,
                    Texture.WrapMode.CLAMP_TO_EDGE,
                    /*useMipmaps=*/ false
                )
            // The dfg.raw file is a raw half-float texture with two channels.
            val dfgResolution = 64
            val dfgChannels = 2
            val halfFloatSize = 2

            val buffer: ByteBuffer =
                ByteBuffer.allocateDirect(dfgResolution * dfgResolution * dfgChannels * halfFloatSize)
            activity.assets.open("models/dfg.raw").use { it.read(buffer.array()) }

            // SampleRender abstraction leaks here.
            GLES30.glBindTexture(GLES30.GL_TEXTURE_2D, dfgTexture.textureId)
            GLError.maybeThrowGLException("Failed to bind DFG texture", "glBindTexture")
            GLES30.glTexImage2D(
                GLES30.GL_TEXTURE_2D,
                /*level=*/ 0,
                GLES30.GL_RG16F,
                /*width=*/ dfgResolution,
                /*height=*/ dfgResolution,
                /*border=*/ 0,
                GLES30.GL_RG,
                GLES30.GL_HALF_FLOAT,
                buffer
            )
            GLError.maybeThrowGLException("Failed to populate DFG texture", "glTexImage2D")

            // Point cloud
            pointCloudShader =
                Shader.createFromAssets(
                    render,
                    "shaders/point_cloud.vert",
                    "shaders/point_cloud.frag",
                    /*defines=*/ null
                )
                    .setVec4(
                        "u_Color",
                        floatArrayOf(31.0f / 255.0f, 188.0f / 255.0f, 210.0f / 255.0f, 1.0f)
                    )
                    .setFloat("u_PointSize", 5.0f)

            // four entries per vertex: X, Y, Z, confidence
            pointCloudVertexBuffer =
                VertexBuffer(render, /*numberOfEntriesPerVertex=*/ 4, /*entries=*/ null)
            val pointCloudVertexBuffers = arrayOf(pointCloudVertexBuffer)
            pointCloudMesh =
                Mesh(
                    render,
                    Mesh.PrimitiveMode.POINTS, /*indexBuffer=*/
                    null,
                    pointCloudVertexBuffers
                )

            // Virtual object to render (ARCore pawn)
            val virtualObjectAlbedoTexture =
                Texture.createFromAsset(
                    render,
                    "models/pawn_albedo.png",
                    Texture.WrapMode.CLAMP_TO_EDGE,
                    Texture.ColorFormat.SRGB
                )

            val virtualObjectAlbedoInstantPlacementTexture =
                Texture.createFromAsset(
                    render,
                    "models/pawn_albedo_instant_placement.png",
                    Texture.WrapMode.CLAMP_TO_EDGE,
                    Texture.ColorFormat.SRGB
                )

            val virtualObjectPbrTexture =
                Texture.createFromAsset(
                    render,
                    "models/pawn_roughness_metallic_ao.png",
                    Texture.WrapMode.CLAMP_TO_EDGE,
                    Texture.ColorFormat.LINEAR
                )

            val virtualObjectMesh = Mesh.createFromAsset(render, "models/pawn.obj")
            val virtualObjectShader =
                Shader.createFromAssets(
                    render,
                    "shaders/environmental_hdr.vert",
                    "shaders/environmental_hdr.frag",
                    mapOf("NUMBER_OF_MIPMAP_LEVELS" to cubemapFilter.numberOfMipmapLevels.toString())
                )
                    .setTexture("u_AlbedoTexture", virtualObjectAlbedoTexture)
                    .setTexture(
                        "u_RoughnessMetallicAmbientOcclusionTexture",
                        virtualObjectPbrTexture
                    )
                    .setTexture("u_Cubemap", cubemapFilter.filteredCubemapTexture)
                    .setTexture("u_DfgTexture", dfgTexture)

            virtualObject = ArSceneObject(virtualObjectMesh, virtualObjectShader)

            customShaders.add(virtualObjectShader)

            cubeObjectMesh = Mesh.createFromAsset(render, "models/cube.obj")

            for (i in 1..10) {
                val shader = generateShader(render)
                generatedShaders.add(shader)
                customShaders.add(shader)
            }

            cubeObjectShader = generateShader(render)
            customShaders.add(cubeObjectShader)


        } catch (e: IOException) {
            Log.e(TAG, "Failed to read a required asset file", e)
            showError("Failed to read a required asset file: $e")
        }
    }

    override fun onSurfaceChanged(render: SampleRender, width: Int, height: Int) {
        displayRotationHelper.onSurfaceChanged(width, height)
        virtualSceneFramebuffer.resize(width, height)
    }

    override fun onDrawFrame(render: SampleRender) {
        val session = session ?: return

        // Texture names should only be set once on a GL thread unless they change. This is done during
        // onDrawFrame rather than onSurfaceCreated since the session is not guaranteed to have been
        // initialized during the execution of onSurfaceCreated.
        if (!hasSetTextureNames) {
            session.setCameraTextureNames(intArrayOf(backgroundRenderer.cameraColorTexture.textureId))
            hasSetTextureNames = true
        }

        // -- Update per-frame state

        // Notify ARCore session that the view size changed so that the perspective matrix and
        // the video background can be properly adjusted.
        displayRotationHelper.updateSessionIfNeeded(session)

        // Obtain the current frame from ARSession. When the configuration is set to
        // UpdateMode.BLOCKING (it is by default), this will throttle the rendering to the
        // camera framerate.
        val frame =
            try {
                session.update()
            } catch (e: CameraNotAvailableException) {
                Log.e(TAG, "Camera not available during onDrawFrame", e)
                showError("Camera not available. Try restarting the app.")
                return
            }

        val camera = frame.camera

        // Update BackgroundRenderer state to match the depth settings.
        try {
            backgroundRenderer.setUseDepthVisualization(
                render,
                activity.depthSettings.depthColorVisualizationEnabled()
            )
            backgroundRenderer.setUseOcclusion(
                render,
                activity.depthSettings.useDepthForOcclusion()
            )
        } catch (e: IOException) {
            Log.e(TAG, "Failed to read a required asset file", e)
            showError("Failed to read a required asset file: $e")
            return
        }

        // BackgroundRenderer.updateDisplayGeometry must be called every frame to update the coordinates
        // used to draw the background camera image.
        backgroundRenderer.updateDisplayGeometry(frame)
        val shouldGetDepthImage =
            activity.depthSettings.useDepthForOcclusion() ||
                    activity.depthSettings.depthColorVisualizationEnabled()
        if (camera.trackingState == TrackingState.TRACKING && shouldGetDepthImage) {
            try {
                val depthImage = frame.acquireDepthImage16Bits()
                backgroundRenderer.updateCameraDepthTexture(depthImage)
                depthImage.close()
            } catch (e: NotYetAvailableException) {
                // This normally means that depth data is not available yet. This is normal so we will not
                // spam the logcat with this.
            }
        }

        if (cubeAnchors.isNotEmpty())
        {
            // Handle one tap per frame.
            handleTap(frame, camera)
        }

        // Keep the screen unlocked while tracking, but allow it to lock when tracking stops.
        trackingStateHelper.updateKeepScreenOnFlag(camera.trackingState)

        // Show a message based on whether tracking has failed, if planes are detected, and if the user
        // has placed any objects.
        val message: String? =
            when {
                cubeAnchors.isEmpty() -> "Tracking Horizontal Planes Surface Area | Target Surface Area ${TARGET_PLANE_AREA}"

                camera.trackingState == TrackingState.PAUSED &&
                        camera.trackingFailureReason == TrackingFailureReason.NONE ->
                    activity.getString(R.string.searching_planes)

                camera.trackingState == TrackingState.PAUSED ->
                    TrackingStateHelper.getTrackingFailureReasonString(camera)

                session.hasTrackingPlane() && wrappedAnchors.isEmpty() ->
                    activity.getString(R.string.waiting_taps)

                session.hasTrackingPlane() && wrappedAnchors.isNotEmpty() -> null
                else -> activity.getString(R.string.searching_planes)
            }
        if (message == null) {
            activity.view.snackbarHelper.hide(activity)
        } else {
            activity.view.snackbarHelper.showMessage(activity, message)
        }

        // -- Draw background
        if (frame.timestamp != 0L) {
            // Suppress rendering if the camera did not produce the first frame yet. This is to avoid
            // drawing possible leftover data from previous sessions if the texture is reused.
            backgroundRenderer.drawBackground(render)
        }

        // If not tracking, don't draw 3D objects.
        if (camera.trackingState == TrackingState.PAUSED) {
            return
        }

        // -- Draw non-occluded virtual objects (planes, point cloud)

        // Get projection matrix.
        camera.getProjectionMatrix(projectionMatrix, 0, Z_NEAR, Z_FAR)

        // Get camera matrix and draw.
        camera.getViewMatrix(viewMatrix, 0)

        // draw planes and point clouds before first tap only
        if (wrappedAnchors.isEmpty()) {
            frame.acquirePointCloud().use { pointCloud ->
                if (pointCloud.timestamp > lastPointCloudTimestamp) {
                    pointCloudVertexBuffer.set(pointCloud.points)
                    lastPointCloudTimestamp = pointCloud.timestamp
                }
                Matrix.multiplyMM(modelViewProjectionMatrix, 0, projectionMatrix, 0, viewMatrix, 0)
                pointCloudShader.setMat4("u_ModelViewProjection", modelViewProjectionMatrix)
                render.draw(pointCloudMesh, pointCloudShader)
            }


            // Visualize planes.
            planeRenderer.drawPlanes(
                render,
                session.getAllTrackables<Plane>(Plane::class.java),
                camera.displayOrientedPose,
                projectionMatrix
            )
        }

        if (cubeAnchors.isEmpty()) {

            val allPlanes: Collection<Plane> = session.getAllTrackables(Plane::class.java)

            // for ((index, plane) in allPlanes.withIndex()) { }

            for (plane in allPlanes) {
                if (plane.trackingState != TrackingState.TRACKING || plane.subsumedBy != null) {
                    continue
                }
                if (plane.type == Plane.Type.HORIZONTAL_UPWARD_FACING) {
                    if (!horizontalPlanes.contains(plane)) {
                        horizontalPlanes.add(plane)
                        // activity.view.snackbarHelper.showMessage(activity, "Plane Count ${horizontalPlanes.size} | extendX: ${plane.extentX} extendY: ${plane.extentZ} centerPose: ${plane.centerPose.translation}")
                    }
                }
            }

            // Removes planes already subsumed by other larger planes.
            horizontalPlanes.removeIf { it.subsumedBy != null && horizontalPlanes.contains(it.subsumedBy) }

            // Sorts planes by their surface area in descending order.
            horizontalPlanes.sortByDescending { it.extentX * it.extentZ }

            if (horizontalPlanes.isNotEmpty()) {
                val plane: Plane = horizontalPlanes[0] /*planes.first()*/

                val planeArea = plane.extentX * plane.extentZ

                // activity.view.snackbarHelper.showMessage(activity, "Plane Surface Area ${plane.extentX * plane.extentZ} | Z: ${plane.centerPose.translation[1]}")
                if (planeArea > TARGET_PLANE_AREA) {

                    // initialize city
                    val gridSize = GRID_SIZE
                    val spacing = SPACING
                    var centerPose = plane.centerPose

                    val extentX = plane.extentX
                    val extentZ = plane.extentZ

                    for (row in 0 until gridSize) {
                        for (col in 0 until gridSize) {

                            val tx = centerPose.tx() + (col - gridSize / 2) * spacing
                            val tz = centerPose.tz() + (row - gridSize / 2) * spacing

                            // world rotation
                            val newPose = Pose.makeTranslation(tx, centerPose.ty(), tz)

                            // alternative - local rotation
                            // val newPose = centerPose.compose(Pose.makeTranslation(tx, ty, tz))

                            if (plane.isPoseInPolygon(newPose)) {
                                val anchor = plane.createAnchor(newPose)
                                val randomScaleFactor = MIN_SCALE_FACTOR + Random.nextFloat() * (MAX_SCALE_FACTOR - MIN_SCALE_FACTOR)
                                val cubeObject =
                                    CubeObject(anchor, plane, randomScaleFactor, generatedShaders.random())
                                val cubeAnchor = CubeAnchor(anchor, plane, cubeObject)
                                cubeAnchors.add(cubeAnchor)
                            }
                        }
                    }
                }


                // val xAxis = centerPose.xAxis
                // val yAxis = centerPose.yAxis
                // val zAxis = centerPose.zAxis


            }
        }


        // -- Draw occluded virtual objects

        // Update lighting parameters in the shader
        updateLightEstimation(frame.lightEstimate, viewMatrix)

        // Visualize anchors created by touch.
        render.clear(virtualSceneFramebuffer, 0f, 0f, 0f, 0f)
        for ((anchor, trackable) in
        wrappedAnchors.filter { it.anchor.trackingState == TrackingState.TRACKING }) {
            // Get the current pose of an Anchor in world space. The Anchor pose is updated
            // during calls to session.update() as ARCore refines its estimate of the world.
            anchor.pose.toMatrix(modelMatrix, 0)

            // SCALE & TRANSLATE
            // Matrix.scaleM(modelMatrix, 0, 1f, 2f, 1f)
            // Matrix.translateM(modelMatrix, 0, 1f, 0f, 0f)

            // Calculate model/view/projection matrices
            Matrix.multiplyMM(modelViewMatrix, 0, viewMatrix, 0, modelMatrix, 0)
            Matrix.multiplyMM(modelViewProjectionMatrix, 0, projectionMatrix, 0, modelViewMatrix, 0)

            customShaders.forEach { shader ->
                shader.setMat4("u_ModelView", modelViewMatrix)
                shader.setMat4("u_ModelViewProjection", modelViewProjectionMatrix)
            }

            /*
            // Update shader properties and draw
            val virtualObjectTexture =
                if ((trackable as? InstantPlacementPoint)?.trackingMethod ==
                    InstantPlacementPoint.TrackingMethod.SCREENSPACE_WITH_APPROXIMATE_DISTANCE
                ) {
                    virtualObjectAlbedoInstantPlacementTexture
                } else {
                    virtualObjectAlbedoTexture
                }

            virtualObjectShader.setTexture("u_AlbedoTexture", virtualObjectTexture)
            */

            render.draw(virtualObject.mesh, virtualObject.shader, virtualSceneFramebuffer)
            // render.draw(cubeObjectMesh, generatedShaders[2], virtualSceneFramebuffer)
        }

        for ((anchor, trackable, cubeObject) in cubeAnchors.filter { it.anchor.trackingState == TrackingState.TRACKING }) {
            anchor.pose.toMatrix(modelMatrix, 0)

            Matrix.scaleM(modelMatrix, 0, 1f, cubeObject.scaleFactor, 1f)

            Matrix.multiplyMM(modelViewMatrix, 0, viewMatrix, 0, modelMatrix, 0)
            Matrix.multiplyMM(modelViewProjectionMatrix, 0, projectionMatrix, 0, modelViewMatrix, 0)

            customShaders.forEach { shader ->
                shader.setMat4("u_ModelView", modelViewMatrix)
                shader.setMat4("u_ModelViewProjection", modelViewProjectionMatrix)
            }

            render.draw(cubeObjectMesh, cubeObject.shader, virtualSceneFramebuffer)
        }


        // Compose the virtual scene with the background.
        backgroundRenderer.drawVirtualScene(render, virtualSceneFramebuffer, Z_NEAR, Z_FAR)
    }

    /** Checks if we detected at least one plane. */
    private fun Session.hasTrackingPlane() =
        getAllTrackables(Plane::class.java).any { it.trackingState == TrackingState.TRACKING }

    /** Update state based on the current frame's light estimation. */
    private fun updateLightEstimation(lightEstimate: LightEstimate, viewMatrix: FloatArray) {
        if (lightEstimate.state != LightEstimate.State.VALID) {
            customShaders.forEach { shader ->
                shader.setBool("u_LightEstimateIsValid", false)
            }
            return
        }
        customShaders.forEach { shader ->
            shader.setBool("u_LightEstimateIsValid", true)
        }

        Matrix.invertM(viewInverseMatrix, 0, viewMatrix, 0)
        customShaders.forEach { shader ->
            shader.setMat4("u_ViewInverse", viewInverseMatrix)
        }

        updateMainLight(
            lightEstimate.environmentalHdrMainLightDirection,
            lightEstimate.environmentalHdrMainLightIntensity,
            viewMatrix
        )
        updateSphericalHarmonicsCoefficients(lightEstimate.environmentalHdrAmbientSphericalHarmonics)
        cubemapFilter.update(lightEstimate.acquireEnvironmentalHdrCubeMap())
    }

    private fun updateMainLight(
        direction: FloatArray,
        intensity: FloatArray,
        viewMatrix: FloatArray
    ) {
        // We need the direction in a vec4 with 0.0 as the final component to transform it to view space
        worldLightDirection[0] = direction[0]
        worldLightDirection[1] = direction[1]
        worldLightDirection[2] = direction[2]
        Matrix.multiplyMV(viewLightDirection, 0, viewMatrix, 0, worldLightDirection, 0)

        customShaders.forEach { shader ->
            shader.setVec4("u_ViewLightDirection", viewLightDirection)
            shader.setVec3("u_LightIntensity", intensity)
        }
    }

    private fun updateSphericalHarmonicsCoefficients(coefficients: FloatArray) {
        // Pre-multiply the spherical harmonics coefficients before passing them to the shader. The
        // constants in sphericalHarmonicFactors were derived from three terms:
        //
        // 1. The normalized spherical harmonics basis functions (y_lm)
        //
        // 2. The lambertian diffuse BRDF factor (1/pi)
        //
        // 3. A <cos> convolution. This is done to so that the resulting function outputs the irradiance
        // of all incoming light over a hemisphere for a given surface normal, which is what the shader
        // (environmental_hdr.frag) expects.
        //
        // You can read more details about the math here:
        // https://google.github.io/filament/Filament.html#annex/sphericalharmonics
        require(coefficients.size == 9 * 3) {
            "The given coefficients array must be of length 27 (3 components per 9 coefficients"
        }

        // Apply each factor to every component of each coefficient
        for (i in 0 until 9 * 3) {
            sphericalHarmonicsCoefficients[i] = coefficients[i] * sphericalHarmonicFactors[i / 3]
        }

        customShaders.forEach { shader ->
            shader.setVec3Array("u_SphericalHarmonicsCoefficients", sphericalHarmonicsCoefficients)
        }
    }

    // Handle only one tap per frame, as taps are usually low frequency compared to frame rate.
    private fun handleTap(frame: Frame, camera: Camera) {
        if (camera.trackingState != TrackingState.TRACKING) return
        val tap = activity.view.tapHelper.poll() ?: return

        val hitResultList =
            if (activity.instantPlacementSettings.isInstantPlacementEnabled) {
                frame.hitTestInstantPlacement(tap.x, tap.y, APPROXIMATE_DISTANCE_METERS)
            } else {
                frame.hitTest(tap)
            }

        // Hits are sorted by depth. Consider only closest hit on a plane, Oriented Point, Depth Point,
        // or Instant Placement Point.
        val firstHitResult =
            hitResultList.firstOrNull { hit ->
                when (val trackable = hit.trackable!!) {
                    is Plane ->
                        trackable.type == Plane.Type.HORIZONTAL_UPWARD_FACING &&
                                trackable.isPoseInPolygon(hit.hitPose) &&
                                PlaneRenderer.calculateDistanceToPlane(hit.hitPose, camera.pose) > 0
                    // is Point -> trackable.orientationMode == Point.OrientationMode.ESTIMATED_SURFACE_NORMAL
                    // is InstantPlacementPoint -> true
                    // DepthPoints are only returned if Config.DepthMode is set to AUTOMATIC.
                    // is DepthPoint -> true

                    // important feature | CubeObject is derived from Trackable class and is calculated hit result
                    // is CubeObject -> false
                    else -> false
                }
            }


        if (firstHitResult != null) {
            // Cap the number of objects created. This avoids overloading both the
            // rendering system and ARCore.
            if (wrappedAnchors.size >= MAX_AR_PAWN_ANCHOR_COUNT) {
                wrappedAnchors[0].anchor.detach()
                wrappedAnchors.removeAt(0)
            }

            // Adding an Anchor tells ARCore that it should track this position in
            // space. This anchor is created on the Plane to place the 3D model
            // in the correct position relative both to the world and to the plane.
            wrappedAnchors.add(
                WrappedAnchor(
                    firstHitResult.createAnchor(),
                    firstHitResult.trackable
                )
            )

            // For devices that support the Depth API, shows a dialog to suggest enabling
            // depth-based occlusion. This dialog needs to be spawned on the UI thread.
            activity.runOnUiThread { activity.view.showOcclusionDialogIfNeeded() }
        }
    }

    private fun showError(errorMessage: String) =
        activity.view.snackbarHelper.showError(activity, errorMessage)


    private fun generateShader(render: SampleRender): Shader {
        val albedoTexture = Texture.createSolidColorTexture(
            render,
            ColorToInt.randomColor(),
            Texture.WrapMode.CLAMP_TO_EDGE,
            Texture.ColorFormat.SRGB
        )
        val albedoInstantPlacementTexture = Texture.createSolidColorTexture(
            render,
            ColorToInt.color(120, 194, 123),
            Texture.WrapMode.CLAMP_TO_EDGE,
            Texture.ColorFormat.SRGB
        )
        val pbrTexture = Texture.createSolidColorTexture(
            render,
            ColorToInt.color(228, 228, 228),
            Texture.WrapMode.CLAMP_TO_EDGE,
            Texture.ColorFormat.SRGB
        )

        val shader =
            Shader.createFromAssets(
                render,
                "shaders/environmental_hdr.vert",
                "shaders/environmental_hdr.frag",
                mapOf("NUMBER_OF_MIPMAP_LEVELS" to cubemapFilter.numberOfMipmapLevels.toString())
            )
                .setTexture("u_AlbedoTexture", albedoTexture)
                .setTexture(
                    "u_RoughnessMetallicAmbientOcclusionTexture",
                    pbrTexture
                )
                .setTexture("u_Cubemap", cubemapFilter.filteredCubemapTexture)
                .setTexture("u_DfgTexture", dfgTexture)
        return shader
    }
}

/**
 * Associates an Anchor with the trackable it was attached to. This is used to be able to check
 * whether or not an Anchor originally was attached to an {@link InstantPlacementPoint}.
 */
private data class WrappedAnchor(
    val anchor: Anchor,
    val trackable: Trackable
)

private data class CubeAnchor(
    val anchor: Anchor,
    val trackable: Trackable,
    val cubeObject: CubeObject
)

private data class CubeObject(
    val anchor: Anchor,
    val plane: Plane,
    val scaleFactor: Float,
    val shader: Shader,
)


/*
class CubeObject(
    val anchor: Anchor,
    val plane: Plane,
    val scaleFactor: Float,
    val color: Int
) : Trackable {
    lateinit var shader: Shader
    lateinit var mesh: Mesh

    override fun getTrackingState(): TrackingState {
        return TrackingState.TRACKING
    }

    override fun createAnchor(pose: Pose?): Anchor {
        return anchor
    }

    override fun getAnchors(): MutableCollection<Anchor> {
        return mutableListOf(anchor)
    }
}
*/

data class ArSceneObject(
    val mesh: Mesh,
    val shader: Shader
)

class ColorToInt {
    companion object {
        fun randomColor(): Int {
            val alpha = 0xFF
            val red = Random.nextInt(0, 256)
            val green = Random.nextInt(0, 256)
            val blue = Random.nextInt(0, 256)

            return (alpha shl 24) or (red shl 16) or (green shl 8) or blue
        }

        fun color(red: Int, green: Int, blue: Int): Int {
            val alpha = 0xFF
            return (alpha shl 24) or (red shl 16) or (green shl 8) or blue
        }
    }
}
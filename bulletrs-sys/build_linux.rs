use cc;

pub fn build_linux() {
    cc::Build::new()
        .include("bullet3/src")
        .define("BT_USE_DOUBLE_PRECISION", None)
        .define("LinearMath_EXPORTS", None)
        .define("NDEBUG", None)
        .define("USE_GRAPHICAL_BENCHMARK", None)
        .opt_level(3) // ignoring OPT_LEVEL from the crate
        .cpp(true)
        .warnings(false)

        .file("bullet3/src/LinearMath/btAlignedAllocator.cpp")
        .file("bullet3/src/LinearMath/btConvexHull.cpp")
        .file("bullet3/src/LinearMath/btConvexHullComputer.cpp")
        .file("bullet3/src/LinearMath/btGeometryUtil.cpp")
        .file("bullet3/src/LinearMath/btPolarDecomposition.cpp")
        .file("bullet3/src/LinearMath/btQuickprof.cpp")
        .file("bullet3/src/LinearMath/btSerializer.cpp")
        .file("bullet3/src/LinearMath/btSerializer64.cpp")
        .file("bullet3/src/LinearMath/btThreads.cpp")
        .file("bullet3/src/LinearMath/btVector3.cpp")
        .compile("LinearMath");

    cc::Build::new()
        .include("bullet3/src")
        .define("BT_USE_DOUBLE_PRECISION", None)
        .define("BulletCollision_EXPORTS", None)
        .define("NDEBUG", None)
        .define("USE_GRAPHICAL_BENCHMARK", None)
        .opt_level(3)
        .cpp(true)
        .warnings(false)

        .file("bullet3/src/BulletCollision/BroadphaseCollision/btAxisSweep3.cpp")
        .file("bullet3/src/BulletCollision/BroadphaseCollision/btBroadphaseProxy.cpp")
        .file("bullet3/src/BulletCollision/BroadphaseCollision/btCollisionAlgorithm.cpp")
        .file("bullet3/src/BulletCollision/BroadphaseCollision/btDbvt.cpp")
        .file("bullet3/src/BulletCollision/BroadphaseCollision/btDbvtBroadphase.cpp")
        .file("bullet3/src/BulletCollision/BroadphaseCollision/btDispatcher.cpp")
        .file("bullet3/src/BulletCollision/BroadphaseCollision/btOverlappingPairCache.cpp")
        .file("bullet3/src/BulletCollision/BroadphaseCollision/btQuantizedBvh.cpp")
        .file("bullet3/src/BulletCollision/BroadphaseCollision/btSimpleBroadphase.cpp")
        .file("bullet3/src/BulletCollision/CollisionDispatch/btActivatingCollisionAlgorithm.cpp")
        .file("bullet3/src/BulletCollision/CollisionDispatch/btBoxBoxCollisionAlgorithm.cpp")
        .file("bullet3/src/BulletCollision/CollisionDispatch/btBox2dBox2dCollisionAlgorithm.cpp")
        .file("bullet3/src/BulletCollision/CollisionDispatch/btBoxBoxDetector.cpp")
        .file("bullet3/src/BulletCollision/CollisionDispatch/btCollisionDispatcher.cpp")
        .file("bullet3/src/BulletCollision/CollisionDispatch/btCollisionDispatcherMt.cpp")
        .file("bullet3/src/BulletCollision/CollisionDispatch/btCollisionObject.cpp")
        .file("bullet3/src/BulletCollision/CollisionDispatch/btCollisionWorld.cpp")
        .file("bullet3/src/BulletCollision/CollisionDispatch/btCollisionWorldImporter.cpp")
        .file("bullet3/src/BulletCollision/CollisionDispatch/btCompoundCollisionAlgorithm.cpp")
        .file("bullet3/src/BulletCollision/CollisionDispatch/btCompoundCompoundCollisionAlgorithm.cpp")
        .file("bullet3/src/BulletCollision/CollisionDispatch/btConvexConcaveCollisionAlgorithm.cpp")
        .file("bullet3/src/BulletCollision/CollisionDispatch/btConvexConvexAlgorithm.cpp")
        .file("bullet3/src/BulletCollision/CollisionDispatch/btConvexPlaneCollisionAlgorithm.cpp")
        .file("bullet3/src/BulletCollision/CollisionDispatch/btConvex2dConvex2dAlgorithm.cpp")
        .file("bullet3/src/BulletCollision/CollisionDispatch/btDefaultCollisionConfiguration.cpp")
        .file("bullet3/src/BulletCollision/CollisionDispatch/btEmptyCollisionAlgorithm.cpp")
        .file("bullet3/src/BulletCollision/CollisionDispatch/btGhostObject.cpp")
        .file("bullet3/src/BulletCollision/CollisionDispatch/btHashedSimplePairCache.cpp")
        .file("bullet3/src/BulletCollision/CollisionDispatch/btInternalEdgeUtility.cpp")
        .file("bullet3/src/BulletCollision/CollisionDispatch/btManifoldResult.cpp")
        .file("bullet3/src/BulletCollision/CollisionDispatch/btSimulationIslandManager.cpp")
        .file("bullet3/src/BulletCollision/CollisionDispatch/btSphereBoxCollisionAlgorithm.cpp")
        .file("bullet3/src/BulletCollision/CollisionDispatch/btSphereSphereCollisionAlgorithm.cpp")
        .file("bullet3/src/BulletCollision/CollisionDispatch/btSphereTriangleCollisionAlgorithm.cpp")
        .file("bullet3/src/BulletCollision/CollisionDispatch/btUnionFind.cpp")
        .file("bullet3/src/BulletCollision/CollisionDispatch/SphereTriangleDetector.cpp")
        .file("bullet3/src/BulletCollision/CollisionShapes/btBoxShape.cpp")
        .file("bullet3/src/BulletCollision/CollisionShapes/btBox2dShape.cpp")
        .file("bullet3/src/BulletCollision/CollisionShapes/btBvhTriangleMeshShape.cpp")
        .file("bullet3/src/BulletCollision/CollisionShapes/btCapsuleShape.cpp")
        .file("bullet3/src/BulletCollision/CollisionShapes/btCollisionShape.cpp")
        .file("bullet3/src/BulletCollision/CollisionShapes/btCompoundShape.cpp")
        .file("bullet3/src/BulletCollision/CollisionShapes/btConcaveShape.cpp")
        .file("bullet3/src/BulletCollision/CollisionShapes/btConeShape.cpp")
        .file("bullet3/src/BulletCollision/CollisionShapes/btConvexHullShape.cpp")
        .file("bullet3/src/BulletCollision/CollisionShapes/btConvexInternalShape.cpp")
        .file("bullet3/src/BulletCollision/CollisionShapes/btConvexPointCloudShape.cpp")
        .file("bullet3/src/BulletCollision/CollisionShapes/btConvexPolyhedron.cpp")
        .file("bullet3/src/BulletCollision/CollisionShapes/btConvexShape.cpp")
        .file("bullet3/src/BulletCollision/CollisionShapes/btConvex2dShape.cpp")
        .file("bullet3/src/BulletCollision/CollisionShapes/btConvexTriangleMeshShape.cpp")
        .file("bullet3/src/BulletCollision/CollisionShapes/btCylinderShape.cpp")
        .file("bullet3/src/BulletCollision/CollisionShapes/btEmptyShape.cpp")
        .file("bullet3/src/BulletCollision/CollisionShapes/btHeightfieldTerrainShape.cpp")
        .file("bullet3/src/BulletCollision/CollisionShapes/btMinkowskiSumShape.cpp")
        .file("bullet3/src/BulletCollision/CollisionShapes/btMultimaterialTriangleMeshShape.cpp")
        .file("bullet3/src/BulletCollision/CollisionShapes/btMultiSphereShape.cpp")
        .file("bullet3/src/BulletCollision/CollisionShapes/btOptimizedBvh.cpp")
        .file("bullet3/src/BulletCollision/CollisionShapes/btPolyhedralConvexShape.cpp")
        .file("bullet3/src/BulletCollision/CollisionShapes/btScaledBvhTriangleMeshShape.cpp")
        .file("bullet3/src/BulletCollision/CollisionShapes/btShapeHull.cpp")
        .file("bullet3/src/BulletCollision/CollisionShapes/btSphereShape.cpp")
        .file("bullet3/src/BulletCollision/CollisionShapes/btStaticPlaneShape.cpp")
        .file("bullet3/src/BulletCollision/CollisionShapes/btStridingMeshInterface.cpp")
        .file("bullet3/src/BulletCollision/CollisionShapes/btTetrahedronShape.cpp")
        .file("bullet3/src/BulletCollision/CollisionShapes/btTriangleBuffer.cpp")
        .file("bullet3/src/BulletCollision/CollisionShapes/btTriangleCallback.cpp")
        .file("bullet3/src/BulletCollision/CollisionShapes/btTriangleIndexVertexArray.cpp")
        .file("bullet3/src/BulletCollision/CollisionShapes/btTriangleIndexVertexMaterialArray.cpp")
        .file("bullet3/src/BulletCollision/CollisionShapes/btTriangleMesh.cpp")
        .file("bullet3/src/BulletCollision/CollisionShapes/btTriangleMeshShape.cpp")
        .file("bullet3/src/BulletCollision/CollisionShapes/btUniformScalingShape.cpp")
        .file("bullet3/src/BulletCollision/Gimpact/btContactProcessing.cpp")
        .file("bullet3/src/BulletCollision/Gimpact/btGenericPoolAllocator.cpp")
        .file("bullet3/src/BulletCollision/Gimpact/btGImpactBvh.cpp")
        .file("bullet3/src/BulletCollision/Gimpact/btGImpactCollisionAlgorithm.cpp")
        .file("bullet3/src/BulletCollision/Gimpact/btGImpactQuantizedBvh.cpp")
        .file("bullet3/src/BulletCollision/Gimpact/btGImpactShape.cpp")
        .file("bullet3/src/BulletCollision/Gimpact/btTriangleShapeEx.cpp")
        .file("bullet3/src/BulletCollision/Gimpact/gim_box_set.cpp")
        .file("bullet3/src/BulletCollision/Gimpact/gim_contact.cpp")
        .file("bullet3/src/BulletCollision/Gimpact/gim_memory.cpp")
        .file("bullet3/src/BulletCollision/Gimpact/gim_tri_collision.cpp")
        .file("bullet3/src/BulletCollision/NarrowPhaseCollision/btContinuousConvexCollision.cpp")
        .file("bullet3/src/BulletCollision/NarrowPhaseCollision/btConvexCast.cpp")
        .file("bullet3/src/BulletCollision/NarrowPhaseCollision/btGjkConvexCast.cpp")
        .file("bullet3/src/BulletCollision/NarrowPhaseCollision/btGjkEpa2.cpp")
        .file("bullet3/src/BulletCollision/NarrowPhaseCollision/btGjkEpaPenetrationDepthSolver.cpp")
        .file("bullet3/src/BulletCollision/NarrowPhaseCollision/btGjkPairDetector.cpp")
        .file("bullet3/src/BulletCollision/NarrowPhaseCollision/btMinkowskiPenetrationDepthSolver.cpp")
        .file("bullet3/src/BulletCollision/NarrowPhaseCollision/btPersistentManifold.cpp")
        .file("bullet3/src/BulletCollision/NarrowPhaseCollision/btRaycastCallback.cpp")
        .file("bullet3/src/BulletCollision/NarrowPhaseCollision/btSubSimplexConvexCast.cpp")
        .file("bullet3/src/BulletCollision/NarrowPhaseCollision/btVoronoiSimplexSolver.cpp")
        .file("bullet3/src/BulletCollision/NarrowPhaseCollision/btPolyhedralContactClipping.cpp")

        .compile("BulletCollision");


    cc::Build::new()
        .include("bullet3/src")
        .define("BT_USE_DOUBLE_PRECISION", None)
        .define("BulletDynamics_EXPORTS", None)
        .define("NDEBUG", None)
        .define("USE_GRAPHICAL_BENCHMARK", None)
        .opt_level(3)
        .cpp(true)
        .warnings(false)

        .file("bullet3/src/BulletDynamics/Character/btKinematicCharacterController.cpp")
        .file("bullet3/src/BulletDynamics/ConstraintSolver/btConeTwistConstraint.cpp")
        .file("bullet3/src/BulletDynamics/ConstraintSolver/btContactConstraint.cpp")
        .file("bullet3/src/BulletDynamics/ConstraintSolver/btFixedConstraint.cpp")
        .file("bullet3/src/BulletDynamics/ConstraintSolver/btGearConstraint.cpp")
        .file("bullet3/src/BulletDynamics/ConstraintSolver/btGeneric6DofConstraint.cpp")
        .file("bullet3/src/BulletDynamics/ConstraintSolver/btGeneric6DofSpringConstraint.cpp")
        .file("bullet3/src/BulletDynamics/ConstraintSolver/btGeneric6DofSpring2Constraint.cpp")
        .file("bullet3/src/BulletDynamics/ConstraintSolver/btHinge2Constraint.cpp")
        .file("bullet3/src/BulletDynamics/ConstraintSolver/btHingeConstraint.cpp")
        .file("bullet3/src/BulletDynamics/ConstraintSolver/btPoint2PointConstraint.cpp")
        .file("bullet3/src/BulletDynamics/ConstraintSolver/btSequentialImpulseConstraintSolver.cpp")
        .file("bullet3/src/BulletDynamics/ConstraintSolver/btNNCGConstraintSolver.cpp")
        .file("bullet3/src/BulletDynamics/ConstraintSolver/btSliderConstraint.cpp")
        .file("bullet3/src/BulletDynamics/ConstraintSolver/btSolve2LinearConstraint.cpp")
        .file("bullet3/src/BulletDynamics/ConstraintSolver/btTypedConstraint.cpp")
        .file("bullet3/src/BulletDynamics/ConstraintSolver/btUniversalConstraint.cpp")
        .file("bullet3/src/BulletDynamics/Dynamics/btDiscreteDynamicsWorld.cpp")
        .file("bullet3/src/BulletDynamics/Dynamics/btDiscreteDynamicsWorldMt.cpp")
        .file("bullet3/src/BulletDynamics/Dynamics/btSimulationIslandManagerMt.cpp")
        .file("bullet3/src/BulletDynamics/Dynamics/btRigidBody.cpp")
        .file("bullet3/src/BulletDynamics/Dynamics/btSimpleDynamicsWorld.cpp")
        .file("bullet3/src/BulletDynamics/Vehicle/btRaycastVehicle.cpp")
        .file("bullet3/src/BulletDynamics/Vehicle/btWheelInfo.cpp")
        .file("bullet3/src/BulletDynamics/Featherstone/btMultiBody.cpp")
        .file("bullet3/src/BulletDynamics/Featherstone/btMultiBodyConstraintSolver.cpp")
        .file("bullet3/src/BulletDynamics/Featherstone/btMultiBodyDynamicsWorld.cpp")
        .file("bullet3/src/BulletDynamics/Featherstone/btMultiBodyJointLimitConstraint.cpp")
        .file("bullet3/src/BulletDynamics/Featherstone/btMultiBodyConstraint.cpp")
        .file("bullet3/src/BulletDynamics/Featherstone/btMultiBodyFixedConstraint.cpp")
        .file("bullet3/src/BulletDynamics/Featherstone/btMultiBodySliderConstraint.cpp")
        .file("bullet3/src/BulletDynamics/Featherstone/btMultiBodyJointMotor.cpp")
        .file("bullet3/src/BulletDynamics/Featherstone/btMultiBodyGearConstraint.cpp")
        .file("bullet3/src/BulletDynamics/Featherstone/btMultiBodyPoint2Point.cpp")
        .file("bullet3/src/BulletDynamics/MLCPSolvers/btDantzigLCP.cpp")
        .file("bullet3/src/BulletDynamics/MLCPSolvers/btMLCPSolver.cpp")
        .file("bullet3/src/BulletDynamics/MLCPSolvers/btLemkeAlgorithm.cpp")
        .compile("BulletDynamics");

    cc::Build::new()
        .include("bullet3/src")
        .define("Bullet3Common_EXPORTS", None)
        .define("NDEBUG", None)
        .define("USE_GRAPHICAL_BENCHMARK", None)
        .define("BT_USE_DOUBLE_PRECISION", None)
        .opt_level(3)
        .cpp(true)
        .warnings(false)

        .file("bullet3/src/Bullet3Common/b3AlignedAllocator.cpp")
        .file("bullet3/src/Bullet3Common/b3Vector3.cpp")
        .file("bullet3/src/Bullet3Common/b3Logging.cpp")
        .compile("Bullet3Common");

    cc::Build::new()
        .include("bullet3/src")
        .include("examples/OpenGLWindow/..")
        .include("bullet3/examples/OpenGLWindow/../ThirdPartyLibs")
        .include("bullet3/examples/OpenGLWindow/../../src")
        .include("bullet3/examples/ThirdPartyLibs/Glew")
        .include("bullet3/examples/ThirdPartyLibs/optionalX11")
        .define("BT_USE_DOUBLE_PRECISION", None)
        .define("DYNAMIC_LOAD_X11_FUNCTIONS", Some("1"))
        .define("GLEW_DYNAMIC_LOAD_ALL_GLX_FUNCTIONS", Some("1"))
        .define("GLEW_STATIC", None)
        .define("GLEW_INIT_OPENGL11_FUNCTIONS", Some("1"))
        .define("OpenGLWindow_EXPORTS", None)
        .define("NDEBUG", None)
        .define("USE_GRAPHICAL_BENCHMARK", None)
        .opt_level(3)
        .cpp(true)
        .warnings(false)

        .file("bullet3/examples/OpenGLWindow/X11OpenGLWindow.cpp")
        .file("bullet3/examples/ThirdPartyLibs/Glew/glew.c")
        .file("bullet3/examples/OpenGLWindow/EGLOpenGLWindow.cpp")
        .file("bullet3/examples/OpenGLWindow/GLFWOpenGLWindow.cpp")
        .file("bullet3/examples/OpenGLWindow/GLInstancingRenderer.cpp")
        .file("bullet3/examples/OpenGLWindow/GLPrimitiveRenderer.cpp")
        .file("bullet3/examples/OpenGLWindow/GLRenderToTexture.cpp")
        .file("bullet3/examples/OpenGLWindow/LoadShader.cpp")
        .file("bullet3/examples/OpenGLWindow/OpenSans.cpp")
        .file("bullet3/examples/OpenGLWindow/SimpleOpenGL2App.cpp")
        .file("bullet3/examples/OpenGLWindow/SimpleOpenGL2Renderer.cpp")
        .file("bullet3/examples/OpenGLWindow/SimpleOpenGL3App.cpp")
        .file("bullet3/examples/OpenGLWindow/TwFonts.cpp")
        .file("bullet3/examples/OpenGLWindow/fontstash.cpp")
        .file("bullet3/examples/OpenGLWindow/opengl_fontstashcallbacks.cpp")
        .compile("OpenGLWindow");

    cc::Build::new()
        .include("bullet3/src")
        .include("bullet3/examples/ExampleBrowser/")
        .include("bullet3/examples/ThirdPartyLibs")
        .include("bullet3/examples/ThirdPartyLibs/Glew")
        .define("BulletExampleBrowserLib_EXPORTS", None)
        .define("BT_USE_DOUBLE_PRECISION", None)
        .define("NDEBUG", None)
        .define("USE_GRAPHICAL_BENCHMARK", None)
        .define("GLEW_DYNAMIC_LOAD_ALL_GLX_FUNCTIONS", Some("1"))
        .define("GLEW_INIT_OPENGL11_FUNCTIONS", Some("1"))
        .define("GLEW_STATIC", None)
        .define("GWEN_COMPILE_STATIC", None)
        .define("_STATIC_CPPLIB", None)
        .opt_level(3)
        .cpp(true)
        .warnings(false)

        .file("bullet3/examples/ExampleBrowser/OpenGLExampleBrowser.cpp")
        .file("bullet3/examples/ExampleBrowser/OpenGLGuiHelper.cpp")
        .file("bullet3/examples/ExampleBrowser/GL_ShapeDrawer.cpp")
        .file("bullet3/examples/ExampleBrowser/CollisionShape2TriangleMesh.cpp")
        .file("bullet3/examples/Utils/b3Clock.cpp")
        .file("bullet3/examples/Utils/ChromeTraceUtil.cpp")
        .file("bullet3/examples/Utils/b3ResourcePath.cpp")
        .file("bullet3/examples/ExampleBrowser/GwenGUISupport/GraphingTexture.cpp")
        .file("bullet3/examples/ExampleBrowser/GwenGUISupport/GwenParameterInterface.cpp")
        .file("bullet3/examples/ExampleBrowser/GwenGUISupport/GwenProfileWindow.cpp")
        .file("bullet3/examples/ExampleBrowser/GwenGUISupport/GwenTextureWindow.cpp")
        .file("bullet3/examples/ExampleBrowser/GwenGUISupport/gwenUserInterface.cpp")
        .compile("BulletExampleBrowserLib");

    cc::Build::new()
        .include("bullet3/src")
        .include("bullet3/examples/ThirdPartyLibs")
        .include("bullet3/examples/ThirdPartyLibs/Glew")
        .include("bullet3/examples/ThirdPartyLibs/optionalX11")
        .define("DYNAMIC_LOAD_X11_FUNCTIONS", Some("1"))
        .define("GLEW_DYNAMIC_LOAD_ALL_GLX_FUNCTIONS", Some("1"))
        .define("GLEW_INIT_OPENGL11_FUNCTIONS", Some("1"))
        .define("GLEW_STATIC", None)
        .define("GWEN_COMPILE_STATIC", None)
        .define("_STATIC_CPPLIB", None)
        .define("gwen_EXPORTS", None)
        .define("NDEBUG", None)
        .define("USE_GRAPHICAL_BENCHMARK", None)
        .define("BT_USE_DOUBLE_PRECISION", None)
        .opt_level(3)
        .cpp(true)
        .warnings(false)

        .file("bullet3/examples/ThirdPartyLibs/Gwen/Anim.cpp")
        .file("bullet3/examples/ThirdPartyLibs/Gwen/BaseRender.cpp")
        .file("bullet3/examples/ThirdPartyLibs/Gwen/DragAndDrop.cpp")
        .file("bullet3/examples/ThirdPartyLibs/Gwen/Gwen.cpp")
        .file("bullet3/examples/ThirdPartyLibs/Gwen/Hook.cpp")
        .file("bullet3/examples/ThirdPartyLibs/Gwen/Skin.cpp")
        .file("bullet3/examples/ThirdPartyLibs/Gwen/ToolTip.cpp")
        .file("bullet3/examples/ThirdPartyLibs/Gwen/Utility.cpp")
        .file("bullet3/examples/ThirdPartyLibs/Gwen/events.cpp")
        .file("bullet3/examples/ThirdPartyLibs/Gwen/inputhandler.cpp")
        .file("bullet3/examples/ThirdPartyLibs/Gwen/Controls/Base.cpp")
        .file("bullet3/examples/ThirdPartyLibs/Gwen/Controls/Button.cpp")
        .file("bullet3/examples/ThirdPartyLibs/Gwen/Controls/Canvas.cpp")
        .file("bullet3/examples/ThirdPartyLibs/Gwen/Controls/CheckBox.cpp")
        .file("bullet3/examples/ThirdPartyLibs/Gwen/Controls/ColorControls.cpp")
        .file("bullet3/examples/ThirdPartyLibs/Gwen/Controls/ColorPicker.cpp")
        .file("bullet3/examples/ThirdPartyLibs/Gwen/Controls/ComboBox.cpp")
        .file("bullet3/examples/ThirdPartyLibs/Gwen/Controls/CrossSplitter.cpp")
        .file("bullet3/examples/ThirdPartyLibs/Gwen/Controls/DockBase.cpp")
        .file("bullet3/examples/ThirdPartyLibs/Gwen/Controls/DockedTabControl.cpp")
        .file("bullet3/examples/ThirdPartyLibs/Gwen/Controls/Dragger.cpp")
        .file("bullet3/examples/ThirdPartyLibs/Gwen/Controls/GroupBox.cpp")
        .file("bullet3/examples/ThirdPartyLibs/Gwen/Controls/HSVColorPicker.cpp")
        .file("bullet3/examples/ThirdPartyLibs/Gwen/Controls/HorizontalScrollBar.cpp")
        .file("bullet3/examples/ThirdPartyLibs/Gwen/Controls/HorizontalSlider.cpp")
        .file("bullet3/examples/ThirdPartyLibs/Gwen/Controls/ImagePanel.cpp")
        .file("bullet3/examples/ThirdPartyLibs/Gwen/Controls/Label.cpp")
        .file("bullet3/examples/ThirdPartyLibs/Gwen/Controls/LabelClickable.cpp")
        .file("bullet3/examples/ThirdPartyLibs/Gwen/Controls/ListBox.cpp")
        .file("bullet3/examples/ThirdPartyLibs/Gwen/Controls/Menu.cpp")
        .file("bullet3/examples/ThirdPartyLibs/Gwen/Controls/MenuItem.cpp")
        .file("bullet3/examples/ThirdPartyLibs/Gwen/Controls/MenuStrip.cpp")
        .file("bullet3/examples/ThirdPartyLibs/Gwen/Controls/NumericUpDown.cpp")
        .file("bullet3/examples/ThirdPartyLibs/Gwen/Controls/PanelListPanel.cpp")
        .file("bullet3/examples/ThirdPartyLibs/Gwen/Controls/ProgressBar.cpp")
        .file("bullet3/examples/ThirdPartyLibs/Gwen/Controls/Properties.cpp")
        .file("bullet3/examples/ThirdPartyLibs/Gwen/Controls/RadioButton.cpp")
        .file("bullet3/examples/ThirdPartyLibs/Gwen/Controls/RadioButtonController.cpp")
        .file("bullet3/examples/ThirdPartyLibs/Gwen/Controls/ResizableControl.cpp")
        .file("bullet3/examples/ThirdPartyLibs/Gwen/Controls/Resizer.cpp")
        .file("bullet3/examples/ThirdPartyLibs/Gwen/Controls/RichLabel.cpp")
        .file("bullet3/examples/ThirdPartyLibs/Gwen/Controls/ScrollBar.cpp")
        .file("bullet3/examples/ThirdPartyLibs/Gwen/Controls/ScrollBarBar.cpp")
        .file("bullet3/examples/ThirdPartyLibs/Gwen/Controls/ScrollBarButton.cpp")
        .file("bullet3/examples/ThirdPartyLibs/Gwen/Controls/ScrollControl.cpp")
        .file("bullet3/examples/ThirdPartyLibs/Gwen/Controls/Slider.cpp")
        .file("bullet3/examples/ThirdPartyLibs/Gwen/Controls/SplitterBar.cpp")
        .file("bullet3/examples/ThirdPartyLibs/Gwen/Controls/TabButton.cpp")
        .file("bullet3/examples/ThirdPartyLibs/Gwen/Controls/TabControl.cpp")
        .file("bullet3/examples/ThirdPartyLibs/Gwen/Controls/TabStrip.cpp")
        .file("bullet3/examples/ThirdPartyLibs/Gwen/Controls/Text.cpp")
        .file("bullet3/examples/ThirdPartyLibs/Gwen/Controls/TextBox.cpp")
        .file("bullet3/examples/ThirdPartyLibs/Gwen/Controls/TextBoxNumeric.cpp")
        .file("bullet3/examples/ThirdPartyLibs/Gwen/Controls/TreeControl.cpp")
        .file("bullet3/examples/ThirdPartyLibs/Gwen/Controls/TreeNode.cpp")
        .file("bullet3/examples/ThirdPartyLibs/Gwen/Controls/VerticalScrollBar.cpp")
        .file("bullet3/examples/ThirdPartyLibs/Gwen/Controls/VerticalSlider.cpp")
        .file("bullet3/examples/ThirdPartyLibs/Gwen/Controls/WindowControl.cpp")
        .file("bullet3/examples/ThirdPartyLibs/Gwen/Controls/Dialog/FileOpen.cpp")
        .file("bullet3/examples/ThirdPartyLibs/Gwen/Controls/Dialog/FileSave.cpp")
        .file("bullet3/examples/ThirdPartyLibs/Gwen/Controls/Dialog/Query.cpp")
        .file("bullet3/examples/ThirdPartyLibs/Gwen/Platforms/Null.cpp")
        .file("bullet3/examples/ThirdPartyLibs/Gwen/Platforms/Windows.cpp")
        .file("bullet3/examples/ThirdPartyLibs/Gwen/Renderers/OpenGL_DebugFont.cpp")
        .compile("gwen");

    cc::Build::new()
        .include("bullet3/src")
        .define("BussIK_EXPORTS", None)
        .define("NDEBUG", None)
        .define("USE_GRAPHICAL_BENCHMARK", None)
        .define("BT_USE_DOUBLE_PRECISION", None)
        .opt_level(3)
        .cpp(true)
        .warnings(false)

        .file("bullet3/examples/ThirdPartyLibs/BussIK/Jacobian.cpp")
        .file("bullet3/examples/ThirdPartyLibs/BussIK/LinearR2.cpp")
        .file("bullet3/examples/ThirdPartyLibs/BussIK/LinearR3.cpp")
        .file("bullet3/examples/ThirdPartyLibs/BussIK/LinearR4.cpp")
        .file("bullet3/examples/ThirdPartyLibs/BussIK/MatrixRmn.cpp")
        .file("bullet3/examples/ThirdPartyLibs/BussIK/Misc.cpp")
        .file("bullet3/examples/ThirdPartyLibs/BussIK/Node.cpp")
        .file("bullet3/examples/ThirdPartyLibs/BussIK/Tree.cpp")
        .file("bullet3/examples/ThirdPartyLibs/BussIK/VectorRn.cpp")
        .compile("BussIK");

    cc::Build::new()
        .include("bullet3/src")
        .define("BulletInverseDynamics_EXPORTS", None)
        .define("BT_USE_DOUBLE_PRECISION", None)
        .define("NDEBUG", None)
        .define("USE_GRAPHICAL_BENCHMARK", None)
        .opt_level(3)
        .cpp(true)
        .warnings(false)

        .file("bullet3/src/BulletInverseDynamics/IDMath.cpp")
        .file("bullet3/src/BulletInverseDynamics/MultiBodyTree.cpp")
        .file("bullet3/src/BulletInverseDynamics/details/MultiBodyTreeInitCache.cpp")
        .file("bullet3/src/BulletInverseDynamics/details/MultiBodyTreeImpl.cpp")
        .compile("BulletInverseDynamics");

    cc::Build::new()
        .include("bullet3/src")
        .define("BulletInverseDynamicsUtils_EXPORTS", None)
        .define("BT_USE_DOUBLE_PRECISION", None)
        .define("NDEBUG", None)
        .define("USE_GRAPHICAL_BENCHMARK", None)
        .opt_level(3)
        .cpp(true)
        .warnings(false)

        .file("bullet3/Extras/InverseDynamics/CloneTreeCreator.cpp")
        .file("bullet3/Extras/InverseDynamics/CoilCreator.cpp")
        .file("bullet3/Extras/InverseDynamics/MultiBodyTreeCreator.cpp")
        .file("bullet3/Extras/InverseDynamics/btMultiBodyTreeCreator.cpp")
        .file("bullet3/Extras/InverseDynamics/DillCreator.cpp")
        .file("bullet3/Extras/InverseDynamics/MultiBodyTreeDebugGraph.cpp")
        .file("bullet3/Extras/InverseDynamics/invdyn_bullet_comparison.cpp")
        .file("bullet3/Extras/InverseDynamics/IDRandomUtil.cpp")
        .file("bullet3/Extras/InverseDynamics/RandomTreeCreator.cpp")
        .file("bullet3/Extras/InverseDynamics/SimpleTreeCreator.cpp")
        .file("bullet3/Extras/InverseDynamics/MultiBodyNameMap.cpp")
        .file("bullet3/Extras/InverseDynamics/User2InternalIndex.cpp")
        .compile("BulletInverseDynamicsUtils");

    cc::Build::new()
        .include("bullet3/src")
        .define("BulletFileLoader_EXPORTS", None)
        .define("BT_USE_DOUBLE_PRECISION", None)
        .define("NDEBUG", None)
        .define("USE_GRAPHICAL_BENCHMARK", None)
        .opt_level(3)
        .cpp(true)
        .warnings(false)

        .file("bullet3/Extras/Serialize/BulletFileLoader/bChunk.cpp")
        .file("bullet3/Extras/Serialize/BulletFileLoader/bDNA.cpp")
        .file("bullet3/Extras/Serialize/BulletFileLoader/bFile.cpp")
        .file("bullet3/Extras/Serialize/BulletFileLoader/btBulletFile.cpp")
        .compile("BulletFileLoaderr");

    cc::Build::new()
        .include("bullet3/src")
        .define("BulletWorldImporter_EXPORTS", None)
        .define("BT_USE_DOUBLE_PRECISION", None)
        .define("NDEBUG", None)
        .define("USE_GRAPHICAL_BENCHMARK", None)
        .opt_level(3)
        .cpp(true)
        .warnings(false)

        .file("bullet3/Extras/Serialize/BulletWorldImporter/btBulletWorldImporter.cpp")
        .file("bullet3/Extras/Serialize/BulletWorldImporter/btWorldImporter.cpp")
        .compile("BulletWorldImporter");

    cc::Build::new()
        .include("bullet3/src")
        .include("bullet3/examples/ThirdPartyLibs")
        .include("bullet3/examples/ThirdPartyLibs/enet/include")
        .include("bullet3/examples/ThirdPartyLibs/clsocket/src")
        .define("BT_ENABLE_CLSOCKET", None)
        .define("BT_ENABLE_ENET", None)
        .define("HAS_SOCKLEN_T", None)
        .define("_LINUX", None)
        .define("BT_USE_DOUBLE_PRECISION", None)
        .define("libpybullet_EXPORTS", None)
        .define("NDEBUG", None)
        .define("USE_GRAPHICAL_BENCHMARK", None)
        .opt_level(3)
        .cpp(true)
        .warnings(false)

        .file("bullet3/examples/SharedMemory/IKTrajectoryHelper.cpp")
        .file("bullet3/examples/ExampleBrowser/InProcessExampleBrowser.cpp")
        .file("bullet3/examples/SharedMemory/TinyRendererVisualShapeConverter.cpp")
        .file("bullet3/examples/OpenGLWindow/SimpleCamera.cpp")
        .file("bullet3/examples/TinyRenderer/geometry.cpp")
        .file("bullet3/examples/TinyRenderer/model.cpp")
        .file("bullet3/examples/TinyRenderer/tgaimage.cpp")
        .file("bullet3/examples/TinyRenderer/our_gl.cpp")
        .file("bullet3/examples/TinyRenderer/TinyRenderer.cpp")
        .file("bullet3/examples/SharedMemory/InProcessMemory.cpp")
        .file("bullet3/examples/SharedMemory/PhysicsClient.cpp")
        .file("bullet3/examples/SharedMemory/PhysicsServer.cpp")
        .file("bullet3/examples/SharedMemory/SharedMemoryInProcessPhysicsC_API.cpp")
        .file("bullet3/examples/SharedMemory/PhysicsServerSharedMemory.cpp")
        .file("bullet3/examples/SharedMemory/PhysicsDirect.cpp")
        .file("bullet3/examples/SharedMemory/PhysicsDirectC_API.cpp")
        .file("bullet3/examples/SharedMemory/PhysicsServerCommandProcessor.cpp")
        .file("bullet3/examples/SharedMemory/b3PluginManager.cpp")
        .file("bullet3/examples/SharedMemory/PhysicsClientSharedMemory.cpp")
        .file("bullet3/examples/SharedMemory/PhysicsClientSharedMemory_C_API.cpp")
        .file("bullet3/examples/SharedMemory/PhysicsClientC_API.cpp")
        .file("bullet3/examples/SharedMemory/Win32SharedMemory.cpp")
        .file("bullet3/examples/SharedMemory/PosixSharedMemory.cpp")
        .file("bullet3/examples/Utils/b3ResourcePath.cpp")
        .file("bullet3/examples/Utils/RobotLoggingUtil.cpp")
        .file("bullet3/examples/ThirdPartyLibs/tinyxml/tinystr.cpp")
        .file("bullet3/examples/ThirdPartyLibs/tinyxml/tinyxml.cpp")
        .file("bullet3/examples/ThirdPartyLibs/tinyxml/tinyxmlerror.cpp")
        .file("bullet3/examples/ThirdPartyLibs/tinyxml/tinyxmlparser.cpp")
        .file("bullet3/examples/ThirdPartyLibs/Wavefront/tiny_obj_loader.cpp")
        .file("bullet3/examples/ThirdPartyLibs/stb_image/stb_image.cpp")
        .file("bullet3/examples/Importers/ImportColladaDemo/LoadMeshFromCollada.cpp")
        .file("bullet3/examples/Importers/ImportObjDemo/LoadMeshFromObj.cpp")
        .file("bullet3/examples/Importers/ImportObjDemo/Wavefront2GLInstanceGraphicsShape.cpp")
        .file("bullet3/examples/Importers/ImportMJCFDemo/BulletMJCFImporter.cpp")
        .file("bullet3/examples/Importers/ImportURDFDemo/BulletUrdfImporter.cpp")
        .file("bullet3/examples/Importers/ImportURDFDemo/MyMultiBodyCreator.cpp")
        .file("bullet3/examples/Importers/ImportURDFDemo/URDF2Bullet.cpp")
        .file("bullet3/examples/Importers/ImportURDFDemo/UrdfParser.cpp")
        .file("bullet3/examples/Importers/ImportURDFDemo/urdfStringSplit.cpp")
        .file("bullet3/examples/Importers/ImportMeshUtility/b3ImportMeshUtility.cpp")
        .file("bullet3/examples/MultiThreading/b3PosixThreadSupport.cpp")
        .file("bullet3/examples/MultiThreading/b3Win32ThreadSupport.cpp")
        .file("bullet3/examples/MultiThreading/b3ThreadSupportInterface.cpp")
        .file("bullet3/examples/SharedMemory/PhysicsClientUDP.cpp")
        .file("bullet3/examples/SharedMemory/PhysicsClientUDP_C_API.cpp")
        .file("bullet3/examples/ThirdPartyLibs/enet/win32.c")
        .file("bullet3/examples/ThirdPartyLibs/enet/unix.c")
        .file("bullet3/examples/ThirdPartyLibs/enet/callbacks.c")
        .file("bullet3/examples/ThirdPartyLibs/enet/compress.c")
        .file("bullet3/examples/ThirdPartyLibs/enet/host.c")
        .file("bullet3/examples/ThirdPartyLibs/enet/list.c")
        .file("bullet3/examples/ThirdPartyLibs/enet/packet.c")
        .file("bullet3/examples/ThirdPartyLibs/enet/peer.c")
        .file("bullet3/examples/ThirdPartyLibs/enet/protocol.c")
        .file("bullet3/examples/SharedMemory/PhysicsClientTCP.cpp")
        .file("bullet3/examples/SharedMemory/PhysicsClientTCP_C_API.cpp")
        .file("bullet3/examples/ThirdPartyLibs/clsocket/src/SimpleSocket.cpp")
        .file("bullet3/examples/ThirdPartyLibs/clsocket/src/ActiveSocket.cpp")
        .file("bullet3/examples/ThirdPartyLibs/clsocket/src/PassiveSocket.cpp")
        .compile("pybullet");
}
 

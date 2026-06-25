"""Isaac Sim timeline hook for the culvert full 3D fusion demo.

Run this inside Isaac Sim's Script Editor, or pass it with Isaac's python
startup tooling. Press Play to generate/import a fresh culvert USD and create
ROS 2 camera + point-cloud publishers. Press Stop to remove the generated
culvert reference and delete the temporary USD.
"""

from __future__ import annotations

import importlib.util
from pathlib import Path
import random
import sys

import carb
import omni.graph.core as og
import omni.timeline
import omni.usd
from pxr import Gf, Sdf, UsdGeom, UsdPhysics


SCRIPT_PATH = Path(
    r"C:\Users\AVARADAR\Downloads\isaac-culvert-sim\scripts\generate_culvert_usd.py"
)
GENERATED_USD = Path(
    r"C:\Users\AVARADAR\Downloads\isaac-culvert-sim\outputs\live\culvert_live.usd"
)
CULVERT_PRIM_PATH = "/World/GeneratedCulvert"
CAMERA_PRIM_PATH = "/World/FusionRig/Camera"
GRAPH_PATH = "/World/FusionRig/ROS2Graph"
GROUND_PRIM_PATH = "/World/SimulationGround"
PHYSICS_SCENE_PATH = "/World/PhysicsScene"

IMAGE_TOPIC = "/ros2_image"
POINTCLOUD_TOPIC = "/lidar/raw"
CAMERA_FRAME_ID = "camera_optical_frame"
POINTCLOUD_FRAME_ID = "camera_optical_frame"
WIDTH = 1280
HEIGHT = 720
HORIZONTAL_APERTURE_MM = 20.955
FOCAL_LENGTH_MM = 13.608
FPS = 10

_subscription = None
_loaded = False


def _load_generator():
    if not SCRIPT_PATH.is_file():
        raise FileNotFoundError(f"Missing culvert generator: {SCRIPT_PATH}")

    spec = importlib.util.spec_from_file_location("generate_culvert_usd", SCRIPT_PATH)
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def _ensure_xform(stage, path):
    prim = stage.GetPrimAtPath(path)
    if prim.IsValid():
        return prim
    return UsdGeom.Xform.Define(stage, path).GetPrim()


def _set_transform(prim, translation, rotation_xyz, scale=(1.0, 1.0, 1.0)):
    xform = UsdGeom.Xformable(prim)
    xform.ClearXformOpOrder()
    xform.AddTranslateOp().Set(Gf.Vec3d(*translation))
    xform.AddRotateXYZOp().Set(Gf.Vec3f(*rotation_xyz))
    xform.AddScaleOp().Set(Gf.Vec3f(*scale))


def _remove_prim(stage, path):
    prim = stage.GetPrimAtPath(path)
    if prim.IsValid():
        stage.RemovePrim(path)


def _create_or_update_camera(stage):
    camera = UsdGeom.Camera.Define(stage, CAMERA_PRIM_PATH)
    _set_transform(
        camera.GetPrim(),
        translation=(0.0, -10.5, 0.15),
        rotation_xyz=(90.0, 0.0, 0.0),
    )
    camera.CreateHorizontalApertureAttr(HORIZONTAL_APERTURE_MM)
    camera.CreateFocalLengthAttr(FOCAL_LENGTH_MM)
    camera.CreateFocusDistanceAttr(10.5)
    camera.CreateClippingRangeAttr(Gf.Vec2f(0.05, 1000.0))
    return camera.GetPrim()


def _create_or_update_ground(stage):
    UsdPhysics.Scene.Define(stage, PHYSICS_SCENE_PATH)
    ground = UsdGeom.Cube.Define(stage, GROUND_PRIM_PATH)
    ground.CreateSizeAttr(1.0)
    _set_transform(
        ground.GetPrim(),
        translation=(0.0, 0.0, -0.05),
        rotation_xyz=(0.0, 0.0, 0.0),
        scale=(100.0, 100.0, 0.10),
    )
    UsdPhysics.CollisionAPI.Apply(ground.GetPrim())
    ground.GetPrim().CreateAttribute("culvert:purpose", Sdf.ValueTypeNames.String).Set("static_fall_catcher")
    return ground.GetPrim()


def _connect(graph, source, target):
    controller = og.Controller()
    try:
        controller.connect(source, target)
    except Exception as exc:
        carb.log_warn(f"Could not connect {source} -> {target}: {exc}")


def _set_attr(graph, attr, value):
    controller = og.Controller()
    try:
        controller.attribute(f"{graph}{attr}").set(value)
    except Exception as exc:
        carb.log_warn(f"Could not set {graph}{attr}: {exc}")


def _create_ros_graph(stage):
    _remove_prim(stage, GRAPH_PATH)
    og.Controller.edit(
        {"graph_path": GRAPH_PATH, "evaluator_name": "execution"},
        {
            og.Controller.Keys.CREATE_NODES: [
                ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                ("ReadSimTime", "omni.isaac.core_nodes.IsaacReadSimulationTime"),
                ("CreateRenderProduct", "omni.isaac.core_nodes.IsaacCreateRenderProduct"),
                ("CameraHelper", "omni.isaac.ros2_bridge.ROS2CameraHelper"),
                ("PointCloudHelper", "omni.isaac.ros2_bridge.ROS2CameraHelper"),
            ],
        },
    )

    graph_path = GRAPH_PATH
    _set_attr(graph_path, "/CreateRenderProduct.inputs:cameraPrim", [Sdf.Path(CAMERA_PRIM_PATH)])
    _set_attr(graph_path, "/CreateRenderProduct.inputs:width", WIDTH)
    _set_attr(graph_path, "/CreateRenderProduct.inputs:height", HEIGHT)
    _set_attr(graph_path, "/CameraHelper.inputs:topicName", IMAGE_TOPIC)
    _set_attr(graph_path, "/CameraHelper.inputs:frameId", CAMERA_FRAME_ID)
    _set_attr(graph_path, "/CameraHelper.inputs:type", "rgb")
    _set_attr(graph_path, "/PointCloudHelper.inputs:topicName", POINTCLOUD_TOPIC)
    _set_attr(graph_path, "/PointCloudHelper.inputs:frameId", POINTCLOUD_FRAME_ID)
    _set_attr(graph_path, "/PointCloudHelper.inputs:type", "depth_pcl")

    _connect(graph_path, "/OnPlaybackTick.outputs:tick", f"{graph_path}/CreateRenderProduct.inputs:execIn")
    _connect(graph_path, "/CreateRenderProduct.outputs:execOut", f"{graph_path}/CameraHelper.inputs:execIn")
    _connect(graph_path, "/CreateRenderProduct.outputs:renderProductPath", f"{graph_path}/CameraHelper.inputs:renderProductPath")
    _connect(graph_path, "/ReadSimTime.outputs:simulationTime", f"{graph_path}/CameraHelper.inputs:timeStamp")
    _connect(graph_path, "/CreateRenderProduct.outputs:execOut", f"{graph_path}/PointCloudHelper.inputs:execIn")
    _connect(graph_path, "/CreateRenderProduct.outputs:renderProductPath", f"{graph_path}/PointCloudHelper.inputs:renderProductPath")
    _connect(graph_path, "/ReadSimTime.outputs:simulationTime", f"{graph_path}/PointCloudHelper.inputs:timeStamp")


def load_scene():
    global _loaded
    if _loaded:
        return

    stage = omni.usd.get_context().get_stage()
    if stage is None:
        carb.log_error("No open USD stage")
        return

    generator = _load_generator()
    GENERATED_USD.parent.mkdir(parents=True, exist_ok=True)
    seed = random.randint(1, 2_147_483_647)
    generator.generate_scene(GENERATED_USD, seed)

    _remove_prim(stage, CULVERT_PRIM_PATH)
    culvert = UsdGeom.Xform.Define(stage, CULVERT_PRIM_PATH)
    culvert.GetPrim().GetReferences().AddReference(str(GENERATED_USD), "/World")
    _set_transform(culvert.GetPrim(), translation=(0.0, 0.0, 0.0), rotation_xyz=(0.0, 0.0, 0.0))

    _ensure_xform(stage, "/World/FusionRig")
    _create_or_update_ground(stage)
    _create_or_update_camera(stage)
    _create_ros_graph(stage)

    _loaded = True
    carb.log_info(f"Loaded generated culvert for fusion: {GENERATED_USD} seed={seed}")


def unload_scene():
    global _loaded
    stage = omni.usd.get_context().get_stage()
    if stage is not None:
        _remove_prim(stage, CULVERT_PRIM_PATH)
        _remove_prim(stage, GRAPH_PATH)

    try:
        GENERATED_USD.unlink(missing_ok=True)
    except OSError as exc:
        carb.log_warn(f"Could not delete {GENERATED_USD}: {exc}")

    _loaded = False
    carb.log_info("Removed generated culvert fusion scene")


def _on_timeline_event(event):
    event_type = int(event.type)
    timeline_events = omni.timeline.TimelineEventType
    if event_type == int(timeline_events.PLAY):
        load_scene()
    elif event_type == int(timeline_events.STOP):
        unload_scene()


def install():
    global _subscription
    timeline = omni.timeline.get_timeline_interface()
    stream = timeline.get_timeline_event_stream()
    _subscription = stream.create_subscription_to_pop(_on_timeline_event)
    carb.log_info("Installed culvert full-fusion Play/Stop hook")


install()

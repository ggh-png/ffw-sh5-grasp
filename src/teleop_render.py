"""Rendering and viewport interaction helpers for teleop_app.TeleopApp.

This module owns the GLFW/ImGui/MuJoCo render plumbing that is visually important but
orthogonal to teleop state, IK, grasping, and base driving.  It deliberately works on the
`app` object passed from teleop_app instead of importing TeleopApp, mirroring teleop_ui.py's
duck-typed boundary and avoiding a circular import.
"""

import ctypes
import time

import glfw

# Must precede glfw.init() -- see teleop_app's module docstring.
glfw.init_hint(glfw.PLATFORM, glfw.PLATFORM_X11)

from imgui_bundle import imgui
from imgui_bundle import imguizmo
import mujoco
import numpy as np


def set_camera_preset(cam, preset):
    if preset == 0:  # overview
        cam.lookat[:] = [0.3, 0.0, 1.0]
        cam.distance = 2.2
        cam.azimuth = 120
        cam.elevation = -20
    else:  # right-hand close-up
        cam.lookat[:] = [0.5055, 0.0, 0.85]
        cam.distance = 0.5
        cam.azimuth = 90
        cam.elevation = -15


def setup_render(app, window_w, window_h):
    if not glfw.init():
        raise RuntimeError("glfw.init() failed")
    window = glfw.create_window(window_w, window_h, "FFW-SH5 Teleop", None, None)
    if not window:
        glfw.terminate()
        raise RuntimeError("glfw.create_window() failed")
    glfw.make_context_current(window)
    glfw.swap_interval(0)
    app.window = window
    app.window_h = window_h

    imgui.create_context()
    io = imgui.get_io()
    io.config_flags |= imgui.ConfigFlags_.viewports_enable
    window_address = ctypes.cast(window, ctypes.c_void_p).value
    if not imgui.backends.glfw_init_for_opengl(window_address, True):
        imgui.destroy_context()
        glfw.destroy_window(window)
        glfw.terminate()
        raise RuntimeError("ImGui GLFW backend initialization failed")
    if not imgui.backends.opengl3_init("#version 130"):
        imgui.backends.glfw_shutdown()
        imgui.destroy_context()
        glfw.destroy_window(window)
        glfw.terminate()
        raise RuntimeError("ImGui OpenGL3 backend initialization failed")
    app.imgui_multi_viewport = True

    app.scene = mujoco.MjvScene(app.model, maxgeom=10000)
    app.cam = mujoco.MjvCamera()
    mujoco.mjv_defaultCamera(app.cam)
    set_camera_preset(app.cam, 0)
    app.opt = mujoco.MjvOption()
    mujoco.mjv_defaultOption(app.opt)
    app.pert = mujoco.MjvPerturb()
    app.context = mujoco.MjrContext(app.model, mujoco.mjtFontScale.mjFONTSCALE_150)


def begin_frame(app):
    glfw.poll_events()
    imgui.backends.opengl3_new_frame()
    imgui.backends.glfw_new_frame()
    imgui.new_frame()
    return imgui.get_io()


def shutdown(app):
    imgui.destroy_platform_windows()
    imgui.backends.opengl3_shutdown()
    imgui.backends.glfw_shutdown()
    imgui.destroy_context()
    glfw.destroy_window(app.window)
    glfw.terminate()


def handle_camera_mouse(app, io):
    cur_mouse = list(glfw.get_cursor_pos(app.window))
    dx, dy = cur_mouse[0] - app.last_mouse[0], cur_mouse[1] - app.last_mouse[1]
    app.last_mouse = cur_mouse
    if io.want_capture_mouse or app.gizmo_mouse_active:
        return

    left = glfw.get_mouse_button(app.window, glfw.MOUSE_BUTTON_LEFT) == glfw.PRESS
    right = glfw.get_mouse_button(app.window, glfw.MOUSE_BUTTON_RIGHT) == glfw.PRESS
    middle = glfw.get_mouse_button(app.window, glfw.MOUSE_BUTTON_MIDDLE) == glfw.PRESS
    if left or right or middle:
        _, win_h = glfw.get_window_size(app.window)
        mod_shift = (glfw.get_key(app.window, glfw.KEY_LEFT_SHIFT) == glfw.PRESS
                     or glfw.get_key(app.window, glfw.KEY_RIGHT_SHIFT) == glfw.PRESS)
        if right:
            action = mujoco.mjtMouse.mjMOUSE_MOVE_H if mod_shift else mujoco.mjtMouse.mjMOUSE_MOVE_V
        elif left:
            action = mujoco.mjtMouse.mjMOUSE_ROTATE_H if mod_shift else mujoco.mjtMouse.mjMOUSE_ROTATE_V
        else:
            action = mujoco.mjtMouse.mjMOUSE_ZOOM
        mujoco.mjv_moveCamera(app.model, action, dx / win_h, dy / win_h, app.scene, app.cam)
    if io.mouse_wheel != 0:
        mujoco.mjv_moveCamera(app.model, mujoco.mjtMouse.mjMOUSE_ZOOM, 0.0,
                              -0.05 * io.mouse_wheel, app.scene, app.cam)


def pose_to_imguizmo_matrix(app, world_pos, world_quat):
    mat = np.eye(4)
    mat[:3, :3] = app._quat_to_mat(world_quat)
    mat[:3, 3] = world_pos
    return imguizmo.im_guizmo.Matrix16(mat.astype(float).reshape(16, order="F"))


def imguizmo_matrix_to_pose(app, matrix):
    mat = np.array(matrix.values, dtype=float).reshape((4, 4), order="F")
    world_pos = mat[:3, 3].copy()
    world_quat = app._mat_to_quat(mat[:3, :3])
    return world_pos, world_quat


def _imguizmo_camera_matrices(app, viewport):
    glcam = app.scene.camera[0]
    forward = np.array(glcam.forward, dtype=float)
    forward /= max(np.linalg.norm(forward), 1e-9)
    up = np.array(glcam.up, dtype=float)
    up /= max(np.linalg.norm(up), 1e-9)
    right = np.cross(forward, up)
    right /= max(np.linalg.norm(right), 1e-9)
    up = np.cross(right, forward)
    pos = np.array(glcam.pos, dtype=float)

    view = np.eye(4)
    view[0, :3] = right
    view[1, :3] = up
    view[2, :3] = -forward
    view[0, 3] = -np.dot(right, pos)
    view[1, 3] = -np.dot(up, pos)
    view[2, 3] = np.dot(forward, pos)

    near = float(glcam.frustum_near)
    far = float(glcam.frustum_far)
    top = float(glcam.frustum_top)
    bottom = float(glcam.frustum_bottom)
    aspect = viewport.width / max(1.0, float(viewport.height))
    right_f = top * aspect
    left_f = bottom * aspect
    proj = np.zeros((4, 4))
    proj[0, 0] = 2.0 * near / (right_f - left_f)
    proj[0, 2] = (right_f + left_f) / (right_f - left_f)
    proj[1, 1] = 2.0 * near / (top - bottom)
    proj[1, 2] = (top + bottom) / (top - bottom)
    proj[2, 2] = -(far + near) / (far - near)
    proj[2, 3] = -(2.0 * far * near) / (far - near)
    proj[3, 2] = -1.0

    return (imguizmo.im_guizmo.Matrix16(view.reshape(16, order="F")),
            imguizmo.im_guizmo.Matrix16(proj.reshape(16, order="F")))


def draw_transform_gizmo(app, viewport):
    """Draw ImGuizmo over the active target in the MuJoCo main viewport.

    ``MjrRect`` is expressed in framebuffer-local pixels, while ImGui multi-viewport
    draw lists use desktop-space logical coordinates.  Using ``viewport.left/bottom``
    here therefore moves the gizmo away from the rendered target as soon as the main
    GLFW window is not at desktop origin.  The camera aspect is still taken from the
    MuJoCo framebuffer; drawing is anchored to ImGui's main viewport rectangle.
    """
    target = app._active_gizmo_target()
    world_pos, world_quat = app._gizmo_target_world_pose(target)
    object_matrix = pose_to_imguizmo_matrix(app, world_pos, world_quat)
    view_matrix, proj_matrix = _imguizmo_camera_matrices(app, viewport)
    main_viewport = imgui.get_main_viewport()

    gizmo = imguizmo.im_guizmo
    gizmo.begin_frame()
    gizmo.set_drawlist(imgui.get_foreground_draw_list(main_viewport))
    gizmo.set_rect(
        float(main_viewport.pos.x), float(main_viewport.pos.y),
        float(main_viewport.size.x), float(main_viewport.size.y))
    gizmo.set_orthographic(False)
    gizmo.set_gizmo_size_clip_space(0.18)
    changed_translate = gizmo.manipulate(
        view_matrix, proj_matrix, gizmo.OPERATION.translate, gizmo.MODE.world,
        object_matrix)
    changed_rotate = gizmo.manipulate(
        view_matrix, proj_matrix, gizmo.OPERATION.rotate, gizmo.MODE.local,
        object_matrix)
    app.gizmo_mouse_active = bool(gizmo.is_using_any() or gizmo.is_over())
    if changed_translate or changed_rotate:
        new_pos, new_quat = imguizmo_matrix_to_pose(app, object_matrix)
        app._set_gizmo_target_world_pose(target, new_pos, new_quat)


def collision_visualization_data(app):
    """Return the exact active collision pairs that the CBF currently monitors."""
    if not getattr(app, "collision_viz", False):
        return ()
    solver = getattr(app, "whole_body_solver", None)
    if solver is None:
        return ()
    return solver.collision_distances(app.data)


def _collision_color(distance, safe_distance):
    if distance <= 0.0:
        return np.array([1.0, 0.02, 0.02, 1.0], dtype=np.float32)
    if distance < safe_distance:
        return np.array([1.0, 0.18, 0.02, 1.0], dtype=np.float32)
    return np.array([1.0, 0.78, 0.05, 1.0], dtype=np.float32)


def _append_visual_geom(scene, geom_type, size, pos, mat, rgba):
    if scene.ngeom >= scene.maxgeom:
        return None
    geom = scene.geoms[scene.ngeom]
    mujoco.mjv_initGeom(
        geom, geom_type, np.asarray(size, dtype=float), np.asarray(pos, dtype=float),
        np.asarray(mat, dtype=float).reshape(9), np.asarray(rgba, dtype=np.float32))
    scene.ngeom += 1
    return geom


def _append_collision_overlay(app, constraints):
    """Tint collision meshes, then draw closest points and their separation segment."""
    for index in range(app.scene.ngeom):
        geom = app.scene.geoms[index]
        if (int(geom.objtype) == int(mujoco.mjtObj.mjOBJ_GEOM)
                and 0 <= int(geom.objid) < app.model.ngeom
                and int(app.model.geom_group[int(geom.objid)]) == 3):
            geom.rgba[:] = [0.05, 0.75, 1.0, 0.28]
            geom.transparent = 1

    identity = np.eye(3)
    safe_distance = app.whole_body_solver.collision_safe_distance
    for constraint in constraints:
        color = _collision_color(constraint.distance, safe_distance)
        for point in (constraint.point_a, constraint.point_b):
            _append_visual_geom(
                app.scene, mujoco.mjtGeom.mjGEOM_SPHERE,
                [0.007, 0.007, 0.007], point, identity, color)
        line = _append_visual_geom(
            app.scene, mujoco.mjtGeom.mjGEOM_LINE,
            [0.0, 0.0, 0.0], np.zeros(3), identity, color)
        if line is not None:
            mujoco.mjv_connector(
                line, mujoco.mjtGeom.mjGEOM_LINE, 4.0,
                constraint.point_a, constraint.point_b)


def render_scene(app):
    app._sync_ik_mocaps_from_targets()
    app.opt.flags[mujoco.mjtVisFlag.mjVIS_CONTACTPOINT] = app.contact_viz
    app.opt.flags[mujoco.mjtVisFlag.mjVIS_CONTACTFORCE] = app.contact_viz
    # Model collision geometry lives in group 3 and is hidden by MuJoCo's default option.
    # The CBF overlay uses the same toggle but remains independent of contact-force viz.
    app.opt.geomgroup[3] = bool(getattr(app, "collision_viz", False))
    fb_w, fb_h = glfw.get_framebuffer_size(app.window)
    viewport = mujoco.MjrRect(0, 0, fb_w, fb_h)
    mujoco.mjv_updateScene(app.model, app.data, app.opt, app.pert, app.cam,
                           mujoco.mjtCatBit.mjCAT_ALL, app.scene)
    collision_data = collision_visualization_data(app)
    if app.collision_viz:
        _append_collision_overlay(app, collision_data)
    mujoco.mjr_render(viewport, app.scene, app.context)
    draw_transform_gizmo(app, viewport)

    imgui.render()
    imgui.backends.opengl3_render_draw_data(imgui.get_draw_data())
    if imgui.get_io().config_flags & imgui.ConfigFlags_.viewports_enable:
        main_context = glfw.get_current_context()
        imgui.update_platform_windows()
        imgui.render_platform_windows_default()
        glfw.make_context_current(main_context)
    glfw.swap_buffers(app.window)


def end_frame(app, t0):
    elapsed = time.perf_counter() - t0
    app.freq_ema = 0.9 * app.freq_ema + 0.1 * (1.0 / max(elapsed, 1e-6))
    sleep_time = app.frame_dt - elapsed
    if sleep_time > 0:
        time.sleep(sleep_time)

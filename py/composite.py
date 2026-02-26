'''
Docstring for blender_scripts.composite
这个文件用来渲染外部模型
'''

import bpy
import os
import math
import random
import numpy as np


OUTPUT_DIR = r"F:\BlenderDataset" 

# --- 渲染参数 ---
RES_X = 2592
RES_Y = 2048
NUM_SAMPLES = 5 

# --- 物理参数 (单位: 米) ---
BASELINE = 0.12            
D_HOUSING = 20.2426 / 1000 
D_GLASS = 5.0 / 1000       
GLASS_NORMAL = np.array([-3.02972e-03, -7.03209e-03, 9.99971e-01]) # deprecated

# --- 折射率 ---
IOR_GLASS = 1.50
IOR_WATER = 1.333

# --- 相机内参 ---
FX = 2595.09879
CX = 1275.04847
CY = 1010.90121
SENSOR_WIDTH_MM = 14.1 

# --- 物体生成范围 ---，Deprecated
OBJ_DIST_MIN = 0.6  
OBJ_DIST_MAX = 1.0  

# 外部模型设置：注意外部模型的坐标轴配置：
# Z：模型的Z轴正方向是相机前方
# X: 画面上的竖直方向
# Y: 画面上的水平方向
MODEL_PATH = r"F:\Models\pressure_vessel1.PLY"
MODEL_LOCATION = (0.0, 0.0, 7.0)  # 注意，这里的Z轴正方向是相机前方
UNIT_SCALE = 0.001  # 单位缩放比例，这里需要根据模型导出的大小进行修改

MODEL_SCALE = (1.0 * UNIT_SCALE, 1.0 * UNIT_SCALE, 1.0 * UNIT_SCALE) # 模型缩放比例

MODEL_ROTATION = (0.0, 0.0, 0.0)  # 模型旋转
ROTATE_SCHEME = "uniform" # "random" or "uniform"，random表示随机旋转，uniform表示均匀旋转
AUTO_CENTER_GEOMETRY = False  # 是否自动将几何中心移到原点？
MODEL_ROTATION_OFFSET = (0.0, 0.0, 0.0)  # 如果模型不是以前方为正面，需要额外的旋转修正
MODEL_POSTITION_OFFSET = (0.0, 0.0, 0.0)  # 模型位置偏移，用于调整模型位置


def setup_render_settings():
    scene = bpy.context.scene
    scene.render.engine = 'CYCLES'
    scene.render.resolution_x = RES_X
    scene.render.resolution_y = RES_Y
    
    # GPU 设置
    prefs = bpy.context.preferences.addons['cycles'].preferences
    try:
        cprefs = prefs
        backend = 'CUDA'
        if any(d.type == 'OPTIX' for d in cprefs.devices):
            backend = 'OPTIX'
        cprefs.compute_device_type = backend
        for device in cprefs.devices:
            if device.type == backend: device.use = True
    except: pass
    scene.cycles.device = 'GPU'
    
    # 光程设置
    scene.cycles.max_bounces = 32
    scene.cycles.transmission_bounces = 32
    scene.cycles.transparent_max_bounces = 32
    scene.cycles.samples = 64 

def clean_scene():
    if bpy.context.object and bpy.context.object.mode != 'OBJECT':
        bpy.ops.object.mode_set(mode='OBJECT')
    bpy.ops.object.select_all(action='SELECT')
    bpy.ops.object.delete()

def create_materials():
    # 1. 玻璃
    mat_glass = bpy.data.materials.new(name="OpticalGlass")
    mat_glass.use_nodes = True
    nodes = mat_glass.node_tree.nodes
    nodes.clear()
    bsdf = nodes.new('ShaderNodeBsdfGlass')
    bsdf.inputs['IOR'].default_value = IOR_GLASS
    bsdf.inputs['Roughness'].default_value = 0.0
    out = nodes.new('ShaderNodeOutputMaterial')
    mat_glass.node_tree.links.new(bsdf.outputs['BSDF'], out.inputs['Surface'])
    
    # 2. 水体
    mat_water = bpy.data.materials.new(name="ClearWater")
    mat_water.use_nodes = True
    nodes = mat_water.node_tree.nodes
    nodes.clear()
    refract = nodes.new('ShaderNodeBsdfRefraction')
    refract.inputs['IOR'].default_value = IOR_WATER
    refract.inputs['Roughness'].default_value = 0.0
    refract.inputs['Color'].default_value = (1.0, 1.0, 1.0, 1.0) 
    out = nodes.new('ShaderNodeOutputMaterial')
    mat_water.node_tree.links.new(refract.outputs['BSDF'], out.inputs['Surface'])
    
    return mat_glass, mat_water

def setup_camera_intrinsics(cam_obj):
    cam = cam_obj.data
    cam.sensor_fit = 'HORIZONTAL'
    cam.sensor_width = SENSOR_WIDTH_MM
    f_mm = (FX / RES_X) * cam.sensor_width
    cam.lens = f_mm
    cam.shift_x = -(CX - RES_X / 2.0) / RES_X
    cam.shift_y = (CY - RES_Y / 2.0) / RES_X 
    cam.dof.use_dof = True
    cam.dof.focus_distance = 0.8 
    cam.dof.aperture_fstop = 8.0 
    cam.clip_start = 0.001 
    cam.clip_end = 100.0
    return cam

def build_physical_environment(mat_glass, mat_water):
    rig = bpy.data.objects.new("Rig", None)
    bpy.context.collection.objects.link(rig)
    
    rot_correction = (math.radians(180), 0, 0)

    cam_l = bpy.data.objects.new("Cam_L", bpy.data.cameras.new("Cam_L"))
    setup_camera_intrinsics(cam_l)
    cam_l.location = (0, 0, 0)
    cam_l.rotation_euler = rot_correction 
    cam_l.parent = rig
    bpy.context.collection.objects.link(cam_l)
    
    cam_r = bpy.data.objects.new("Cam_R", bpy.data.cameras.new("Cam_R"))
    setup_camera_intrinsics(cam_r)
    cam_r.location = (BASELINE, 0, 0)
    cam_r.rotation_euler = rot_correction
    cam_r.parent = rig
    bpy.context.collection.objects.link(cam_r)
    
    bpy.ops.mesh.primitive_cube_add(size=1)
    glass = bpy.context.active_object
    glass.data.materials.append(mat_glass)
    glass.parent = rig
    glass.scale = (5.0, 5.0, D_GLASS)
    glass.location = (0, 0, D_HOUSING + D_GLASS/2.0)
    # glass.rotation_euler = (np.arcsin(GLASS_NORMAL[1]), -np.arcsin(GLASS_NORMAL[0]), 0)
    glass.rotation_euler = (0, 0, 0) # 强制玻璃与光心垂直

    bpy.ops.mesh.primitive_cube_add(size=1)
    tank = bpy.context.active_object
    tank.data.materials.append(mat_water)
    tank.parent = rig
    tank.scale = (10.0, 10.0, 5.0)
    tank.location = (0, 0, D_HOUSING + D_GLASS + 2.5) 
    
    return rig, cam_l, cam_r

def load_external_model(rig):
    """
    导入外部模型到场景中
    """
    # 1. 清理上一轮导入的模型
    for obj in bpy.data.objects:
        if "ImportedModel" in obj.name:
            bpy.data.objects.remove(obj, do_unlink=True)
            
    if not os.path.exists(MODEL_PATH):
        print(f"Warning: Model file not found at {MODEL_PATH}")
        return

    # 根据后缀名选择导入器
    ext = os.path.splitext(MODEL_PATH)[1].lower()
    print(f"Loading model: {MODEL_PATH} ({ext})")
    try:
        if ext == '.obj':
            if hasattr(bpy.ops.wm, 'obj_import'):
                bpy.ops.wm.obj_import(filepath=MODEL_PATH)
            else:
                bpy.ops.import_scene.obj(filepath=MODEL_PATH)

        elif ext == '.fbx':
            try: bpy.ops.preferences.addon_enable(module="io_scene_fbx")
            except: pass

            bpy.ops.import_scene.fbx(filepath=MODEL_PATH)
        
        elif ext == '.ply':
            if hasattr(bpy.ops.wm, 'ply_import'):
                bpy.ops.wm.ply_import(filepath=MODEL_PATH)
            elif hasattr(bpy.ops.import_mesh, 'ply'):
                bpy.ops.import_mesh.ply(filepath=MODEL_PATH)
            else:
                raise Exception("Cannot find PLY importer operator.")
            
        elif ext in ['.glb', '.gltf']:
            try: bpy.ops.preferences.addon_enable(module="io_scene_gltf")
            except: pass

            bpy.ops.import_scene.gltf(filepath=MODEL_PATH)
        else:
            print(f"Unsupported file format: {ext}")
            return
        
    except Exception as e:
        print(f"Import Failed: {e}")
        return

    # 获取导入的物体并设置位置
    # 导入后选中的通常就是刚导入的物体
    # 如果导入的是一组物体，我们将它们全部归入一个父级或者统一移动
    selected_objects = bpy.context.selected_objects
    
    if not selected_objects:
        return
    
    # 确保我们操作的是物体模式
    bpy.ops.object.select_all(action='DESELECT')
    for obj in selected_objects:
        obj.select_set(True)
    bpy.context.view_layer.objects.active = selected_objects[0]

    if AUTO_CENTER_GEOMETRY:
        bpy.ops.object.origin_set(type='ORIGIN_GEOMETRY', center='BOUNDS')  # 几何中心位置

    # 创建一个父级 Empty 来管理导入的模型 (方便整体移动)
    model_anchor = bpy.data.objects.new("ImportedModel_Anchor", None)
    bpy.context.collection.objects.link(model_anchor)

    final_position = (MODEL_LOCATION[0] + MODEL_POSTITION_OFFSET[0],
                      MODEL_LOCATION[1] + MODEL_POSTITION_OFFSET[1],
                      MODEL_LOCATION[2] + MODEL_POSTITION_OFFSET[2])
    model_anchor.location = final_position

    model_anchor.scale = MODEL_SCALE

    model_anchor.rotation_euler = (
        MODEL_ROTATION[0] + MODEL_ROTATION_OFFSET[0],
        MODEL_ROTATION[1] + MODEL_ROTATION_OFFSET[1],
        MODEL_ROTATION[2] + MODEL_ROTATION_OFFSET[2],
    )
    
    # 将导入的所有物体挂载到 anchor 下
    for obj in selected_objects:
        obj.parent = model_anchor
        if AUTO_CENTER_GEOMETRY:
            obj.location = (0, 0, 0)
        # 如果模型自带材质，这里就不强制覆盖了，保留模型原貌
        # 如果需要强制覆盖，可以在这里添加 obj.data.materials.append(...)

def generate_objects(rig):
    for obj in bpy.data.objects:
        if "TargetObj" in obj.name:
            bpy.data.objects.remove(obj, do_unlink=True)
            
    for i in range(15): 
        bpy.ops.mesh.primitive_ico_sphere_add(radius=random.uniform(0.05, 0.1)) 
        obj = bpy.context.active_object
        obj.name = f"TargetObj_{i}"
        
        z = random.uniform(OBJ_DIST_MIN, OBJ_DIST_MAX)
        limit_x = z * 0.35 
        limit_y = z * 0.25
        x = random.uniform(-limit_x, limit_x)
        y = random.uniform(-limit_y, limit_y)
        
        obj.location = (x, y, z)
        
        mat = bpy.data.materials.new(name=f"Mat_{i}")
        mat.use_nodes = True
        nodes = mat.node_tree.nodes
        nodes.clear()
        
        bsdf = nodes.new('ShaderNodeBsdfPrincipled')
        bsdf.inputs['Base Color'].default_value = (random.random(), random.random(), random.random(), 1)
        bsdf.inputs['Roughness'].default_value = 0.5 
        
        out = nodes.new('ShaderNodeOutputMaterial')
        mat.node_tree.links.new(bsdf.outputs['BSDF'], out.inputs['Surface'])
        obj.data.materials.append(mat)

def init_compositor():
    """
    合成器配置
    使用两个独立的 OutputFile 节点，彻底隔离 PNG(RGB) 和 EXR(Depth) 的设置
    """
    scene = bpy.context.scene
    scene.use_nodes = True
    bpy.context.view_layer.use_pass_z = True
    tree = scene.node_tree
    for n in tree.nodes: tree.nodes.remove(n)
    
    rl = tree.nodes.new('CompositorNodeRLayers')
    
    # === 节点 1: 专门负责 RGB 图像 (PNG) ===
    out_rgb = tree.nodes.new('CompositorNodeOutputFile')
    out_rgb.base_path = OUTPUT_DIR
    out_rgb.format.file_format = 'PNG'
    # 强制 RGB，确保输出是 3 通道
    out_rgb.format.color_mode = 'RGB'  
    out_rgb.format.color_depth = '8'
    
    # 链接 Render Layers 的 Image 到 输出节点
    tree.links.new(rl.outputs[0], out_rgb.inputs[0])
    
    # === 节点 2: 专门负责 Depth 图像 (EXR) ===
    out_depth = tree.nodes.new('CompositorNodeOutputFile')
    out_depth.base_path = OUTPUT_DIR
    out_depth.format.file_format = 'OPEN_EXR'
    out_depth.format.color_depth = '32'
    
    # 链接 Render Layers 的 Depth 到 输出节点
    depth_sock = rl.outputs.get('Depth') or rl.outputs.get('Z')
    if depth_sock: 
        tree.links.new(depth_sock, out_depth.inputs[0])
    
    return out_rgb, out_depth

def set_path(node_rgb, node_depth, side, idx):
    folder = os.path.join(OUTPUT_DIR, side)
    if not os.path.exists(folder): os.makedirs(folder)
    
    # 设置 RGB 节点路径
    node_rgb.base_path = folder
    node_rgb.file_slots[0].path = f"{idx:04d}_rgb"
    
    # 设置 Depth 节点路径
    node_depth.base_path = folder
    node_depth.file_slots[0].path = f"{idx:04d}_depth"

def main():
    if not os.path.exists(OUTPUT_DIR): os.makedirs(OUTPUT_DIR)
    
    setup_render_settings()
    clean_scene()
    
    mat_glass, mat_water = create_materials()
    rig, cam_l, cam_r = build_physical_environment(mat_glass, mat_water)
    
    # === 环境 ===
    bpy.context.scene.world.node_tree.nodes["Background"].inputs[0].default_value = (0.1, 0.1, 0.1, 1)

    # === 灯光 ===
    bpy.ops.object.light_add(type='SUN', location=(0, -2, 2))
    sun = bpy.context.active_object
    sun.rotation_euler = (0.3, 0.1, 0) 
    sun.data.energy = 10.0 
    sun.data.angle = 0.5
    
    sun.visible_camera = False       # 摄像机不可见
    sun.visible_glossy = False       # 玻璃反射不可见
    sun.visible_transmission = False # 玻璃折射不可见
    
    node_rgb, node_depth = init_compositor()
    
    print(f"Start Rendering...")
    
    load_external_model(rig)
    model_anchor = bpy.data.objects.get("ImportedModel_Anchor")
    if not model_anchor:
        print("Error: Model not loaded correctly")
        return

    for i in range(NUM_SAMPLES):  
        base_rot_x = model_anchor.rotation_euler[0]
        base_rot_y = model_anchor.rotation_euler[1]
        base_rot_z = model_anchor.rotation_euler[2]

        if ROTATE_SCHEME == "uniform":
            random_angle = (1 / NUM_SAMPLES) * 2 * math.pi
        elif ROTATE_SCHEME == "random":
            random_angle = random.uniform(0, 2 * math.pi)
        else:
            random_angle = 0
            
        # 绕Z轴旋转
        model_anchor.rotation_euler = (base_rot_x + random_angle, base_rot_y, base_rot_z)
        bpy.context.view_layer.update()
        
        bpy.context.scene.camera = cam_l
        set_path(node_rgb, node_depth, "left", i)
        print(f"Frame {i} Left...")
        bpy.ops.render.render(write_still=False)
        
        bpy.context.scene.camera = cam_r
        set_path(node_rgb, node_depth, "right", i)
        print(f"Frame {i} Right...")
        bpy.ops.render.render(write_still=False)
        
    print("Done!")

if __name__ == "__main__":
    main()
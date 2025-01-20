# -*- coding: utf-8 -*-

'''
Copyright <2025> <CaveMapper>
Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:
The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
'''

import bpy
from bpy.props import (
    IntProperty,
    FloatProperty,
    FloatVectorProperty,
    EnumProperty,
    BoolProperty,
    StringProperty
)
import open3d as o3d
import bmesh
import numpy as np
import copy
import time
import math
import glob
import os
import mathutils
from mathutils import Color
from mathutils import Vector
from mathutils import Matrix
import pkg_resources
import subprocess
import sys
import textwrap
import zipfile
import shutil
import re

global ini_verts
global end_verts
global source_obj
global selected_verts_id
global canAdjustEffect
global min_distance_list

ini_verts = []
end_verts = []
len_to_center = []
max_len_to_center_selection = 0.0
source_obj = None
selected_verts_id = []
canAdjustEffect = False

#color_convertor
def setObjColor(i):
    h_div = 12.0
    c = Color()
    c.hsv = (i % h_div)/h_div,0.8, 1.0
    
    return (c.r,c.g,c.b,1)

#######################################################################
#Transform interpolation functions
#######################################################################
def tfm_to_quat(tfm):
    r11 = tfm[0][0]
    r12 = tfm[0][1]
    r13 = tfm[0][2]
    r21 = tfm[1][0]
    r22 = tfm[1][1]
    r23 = tfm[1][2]
    r31 = tfm[2][0]
    r32 = tfm[2][1]
    r33 = tfm[2][2]

    q0 = (r11+r22+r33+1.0)/4.0
    q1 = (r11-r22-r33+1.0)/4.0
    q2 = (-r11+r22-r33+1.0)/4.0
    q3 = (-r11-r22+r33+1.0)/4.0

    if q0<0:q0=0
    if q1<0:q1=0
    if q2<0:q2=0
    if q3<0:q3=0

    q0=np.sqrt(q0)
    q1=np.sqrt(q1)
    q2=np.sqrt(q2)
    q3=np.sqrt(q3)

    if(q0>q1) and (q0>q2) and (q0>q3):
        q0 *= +1.0
        q1 *= np.sign(r32 - r23)
        q2 *= np.sign(r13 - r31)
        q3 *= np.sign(r21 - r12)
    elif(q1 >= q0) and (q1 >= q2) and (q1 >= q3):
        q0 *= np.sign(r32 - r23)
        q1 *= +1.0
        q2 *= np.sign(r21 + r12)
        q3 *= np.sign(r13 + r31)
    elif(q2 >= q0) and (q2 >= q1) and (q2 >= q3):
        q0 *= np.sign(r13 - r31)
        q1 *= np.sign(r21 + r12)
        q2 *= +1.0
        q3 *= np.sign(r32 + r23)
    elif(q3 >= q0) and (q3 >= q1) and (q3 >= q2):
        q0 *= np.sign(r21 - r12)
        q1 *= np.sign(r31 + r13)
        q2 *= np.sign(r32 + r23)
        q3 *= +1.0
    else:
        print("coding error\n")
    
    a = np.array([q0, q1, q2, q3])
    r = np.linalg.norm(a)
    q0 /= r
    q1 /= r
    q2 /= r
    q3 /= r

    return np.array([q0, q1, q2, q3])
    
def quat_to_bmesh_rot(q):
    w=q[0]
    x=q[1]
    y=q[2]
    z=q[3]

    r11=1-2*y**2-2*z**2
    r12=2*x*y+2*w*z
    r13=2*x*z-2*w*y
    r21=2*x*y-2*w*z
    r22=1-2*x**2-2*z**2
    r23=2*y*z+2*w*x
    r31=2*x*z+2*w*y
    r32=2*y*z-2*w*x
    r33=1-2*x**2-2*y**2

    #I do not know why blender matrix 4x4 needs to swap row a columns for rotation matrix, but it works.
    rot = mathutils.Matrix.Identity(4)
    rot[0][0] = r11
    rot[0][1] = r21
    rot[0][2] = r31
    rot[0][3] = 0
    rot[1][0] = r12
    rot[1][1] = r22
    rot[1][2] = r32
    rot[1][3] = 0
    rot[2][0] = r13
    rot[2][1] = r23
    rot[2][2] = r33
    rot[2][3] = 0
    rot[3][0] = 0
    rot[3][1] = 0
    rot[3][2] = 0
    rot[3][3] = 1

    return rot

def quat_ip(q1,q2,t):
    theta = math.acos(np.dot(q1,q2))
    if theta != 0:        
        k1 = math.sin((1-t)*theta)/math.sin(theta)
        k2 = math.sin(t*theta)/math.sin(theta)
        return (k1*q1)+(k2*q2)
    else:
        return q1

def tfm_ip(tfm1,tfm2,t):
    q1 = tfm_to_quat(tfm1)
    q2 = tfm_to_quat(tfm2)
    q = quat_ip(q1,q2,t)
    tfm_ip = quat_to_bmesh_rot(q)

    x = tfm1[0][3] + t*(tfm2[0][3] - tfm1[0][3])
    y = tfm1[1][3] + t*(tfm2[1][3] - tfm1[1][3])
    z = tfm1[2][3] + t*(tfm2[2][3] - tfm1[2][3])
    tfm_ip[0][3] = x
    tfm_ip[1][3] = y
    tfm_ip[2][3] = z

    return tfm_ip


#######################################################################
#Interface functions between blender and open3d
#######################################################################
def select_objs(selection_obj_name_list,active_obj_name):
    bpy.ops.object.mode_set(mode='OBJECT', toggle=False)
    for obj in bpy.context.scene.objects:
        obj.select_set(False)
    for obj_name in selection_obj_name_list:
        obj = bpy.data.objects.get(obj_name)
        obj.select_set(True)
    
    if active_obj_name != None:
        active_obj = bpy.data.objects.get(active_obj_name)
        active_obj.select_set(True)
        bpy.context.view_layer.objects.active = active_obj
        
def select_verts(source_obj,selected_verts_id):
    bpy.ops.object.mode_set(mode='OBJECT', toggle=False)
    for obj in bpy.context.scene.objects:
        obj.select_set(False)
    source_obj.select_set(True)
    bpy.context.view_layer.objects.active = source_obj
    bpy.ops.object.mode_set(mode='EDIT', toggle=False)
    
    mesh = source_obj.data
    bm = bmesh.from_edit_mesh(mesh)
    for v in bm.verts:
        if v.index in [v_id for v_id in selected_verts_id]:
            v.select = True
        else:
            v.select = False

    bmesh.update_edit_mesh(mesh)
    

def bmesh_to_pcd(arg_objectname, vert_select):
    selectob = bpy.data.objects.get(arg_objectname)
    if selectob == None:
        return False
    for ob in bpy.context.scene.objects:
        ob.select_set(False)
    selectob.select_set(True)

    if vert_select == False:
        bpy.context.view_layer.objects.active = selectob
        bpy.ops.object.mode_set(mode='EDIT', toggle=False)    
        bpy.ops.mesh.select_all(action='SELECT')
        bpy.ops.mesh.remove_doubles(threshold=0.0001)
        
    meshdata = bmesh.from_edit_mesh(selectob.data)
    meshdata.select_mode = {'VERT'}
    
    np_tfm = np.array(selectob.matrix_world)
    np_points = np.ones((len([v for v in meshdata.verts if v.select]),3))
    np_normals = np.ones((len([v for v in meshdata.verts if v.select]),3))
    
    meshdata.verts.ensure_lookup_table()
    for i, v in enumerate([v for v in meshdata.verts if v.select]):
        np_points[i] = v.co
        np_normals[i] = v.normal
        
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(np_points)
    pcd.normals = o3d.utility.Vector3dVector(np_normals)
    
    return pcd, np_tfm

def np_tfm_2_matrix4x4_tfm(np_tfm):
    matrix4x4_tfm = mathutils.Matrix.Identity(4)
    for i in range(4):
        for j in range(4):
            matrix4x4_tfm[i][j] = np_tfm[i][j]
    print(np_tfm)
    print(matrix4x4_tfm)
    return matrix4x4_tfm

def transform_anime(obj,end_tfm,anime_time,fps):
    wait_time = (1/fps)
    f_num = int(anime_time * fps)

    ini_tfm = copy.copy(obj.matrix_world)
    for f in range(f_num):
        tfm_temp = tfm_ip(ini_tfm,end_tfm,(f+1)/f_num)
        obj.matrix_world = tfm_temp
        bpy.ops.wm.redraw_timer(type='DRAW_WIN_SWAP', iterations=1)
        time.sleep(wait_time)

def transform_bmesh(arg_objectname, np_tfm):
    bpy.ops.object.mode_set(mode='OBJECT', toggle=False)
    selectob = bpy.data.objects.get(arg_objectname)
    if selectob == None:
        return False
    for ob in bpy.context.scene.objects:
        ob.select_set(False)
    selectob.select_set(True)

    bpy.context.view_layer.objects.active = selectob
    bpy.ops.object.mode_set(mode='OBJECT', toggle=False)
    matrix4x4_tfm = np_tfm_2_matrix4x4_tfm(np_tfm)

    transform_anime(selectob,matrix4x4_tfm,1,10)
    #selectob.matrix_world = matrix4x4_tfm

def make_blenderBmesh_from_open3dTriangleMesh(o3d_triangle_mesh,mesh_name):
    verts = o3d_triangle_mesh.vertices
    faces = o3d_triangle_mesh.triangles
    
    msh = bpy.data.meshes.new(mesh_name) #Meshデータの宣言
    msh.from_pydata(verts, [], faces) # 頂点座標と各面の頂点の情報でメッシュを作成
    obj = bpy.data.objects.new(mesh_name, msh) # メッシュデータでオブジェクトを作成
    bpy.context.scene.collection.objects.link(obj) # シーンにオブジェクトを配置
    
    bpy.ops.object.mode_set(mode='OBJECT', toggle=False)
    for ob in bpy.context.scene.objects:
        ob.select_set(False)
    obj.select_set(True)
    
    return obj
    
###########################################################################
#Open3d functions
###########################################################################

def preprocess_point_cloud(pcd, voxel_size, FPFH_coef):
    print(":: Downsample with a voxel size %.3f." % voxel_size)
    pcd_down = pcd.voxel_down_sample(voxel_size)

    radius_feature = voxel_size * FPFH_coef
    print(":: Compute FPFH feature with search radius %.3f." % radius_feature)
    pcd_fpfh = o3d.pipelines.registration.compute_fpfh_feature(
        pcd_down,
        o3d.geometry.KDTreeSearchParamHybrid(radius=radius_feature, max_nn=100))
        
    return pcd_down, pcd_fpfh


def downsize_point_cloud_with_normal(pcd, voxel_size):
    print(":: Downsample with a voxel size %.3f." % voxel_size)
    pcd_down = pcd.voxel_down_sample(voxel_size)
    
    return pcd_down
    

def prepare_dataset(voxel_size, Aligment_FPFH_coef,source,source_tfm, target,target_tfm):
    print(":: Load two point clouds and disturb initial pose.")
    
    source.transform(source_tfm)
    target.transform(target_tfm)
    
    #draw_registration_result(source, target, np.identity(4))

    source_down, source_fpfh = preprocess_point_cloud(source, voxel_size, Aligment_FPFH_coef)
    target_down, target_fpfh = preprocess_point_cloud(target, voxel_size, Aligment_FPFH_coef)
    return source, target, source_down, target_down, source_fpfh, target_fpfh


def execute_global_registration(source_down, target_down, source_fpfh,
                                target_fpfh, voxel_size,GR_coef):
    distance_threshold = voxel_size * GR_coef
    #print(":: RANSAC registration on downsampled point clouds.")
    #print("   Since the downsampling voxel size is %.3f," % voxel_size)
    #print("   we use a liberal distance threshold %.3f." % distance_threshold)
    result = o3d.pipelines.registration.registration_ransac_based_on_feature_matching(
        source_down, target_down, source_fpfh, target_fpfh, True,
        distance_threshold,
        o3d.pipelines.registration.TransformationEstimationPointToPoint(False),
        3, [
            o3d.pipelines.registration.CorrespondenceCheckerBasedOnEdgeLength(
                0.9),
            o3d.pipelines.registration.CorrespondenceCheckerBasedOnDistance(
                distance_threshold)
        ], o3d.pipelines.registration.RANSACConvergenceCriteria(100000, 0.999))
    return result


def execute_fast_global_registration(source_down, target_down, source_fpfh,
                                     target_fpfh, voxel_size, GR_coef):
    distance_threshold = voxel_size * GR_coef
    print(":: Apply fast global registration with distance threshold %.3f" \
            % distance_threshold)
    result = o3d.pipelines.registration.registration_fgr_based_on_feature_matching(
        source_down, target_down, source_fpfh, target_fpfh,
        o3d.pipelines.registration.FastGlobalRegistrationOption(
            maximum_correspondence_distance=distance_threshold))
    return result


def refine_registration(source, target, source_fpfh, target_fpfh, voxel_size,ICP_coef):
    distance_threshold = voxel_size * ICP_coef   
    trans_init = [[1.0, 0.0, 0.0, 0.0], [0.0, 1.0, 0.0, 0.0],
                             [0.0, 0.0, 1.0, 0.0], [0.0, 0.0, 0.0, 1.0]]
    
    print(":: Point-to-plane ICP registration is applied on original point")
    print("   clouds to refine the alignment. This time we use a strict")
    print("   distance threshold %.3f." % distance_threshold)
    result = o3d.pipelines.registration.registration_icp(
        source, target, distance_threshold,trans_init,
        o3d.pipelines.registration.TransformationEstimationPointToPlane())
    return result


################################################################################
#Process Functions
################################################################################
def Process_GR(source_name,target_name,voxel_size,GR_coef, Aligment_FPFH_coef):
    source,source_tfm = bmesh_to_pcd(source_name,False)
    target,target_tfm = bmesh_to_pcd(target_name,False)

    source, target, source_down, target_down, source_fpfh, target_fpfh = prepare_dataset(
        voxel_size, Aligment_FPFH_coef, source,source_tfm, target,target_tfm)
    result_ransac = execute_global_registration(source_down, target_down, source_fpfh, target_fpfh, voxel_size,GR_coef)
    #result_ransac = execute_fast_global_registration(source_down, target_down, source_fpfh, target_fpfh, voxel_size,GR_coef)
    
    transform_bmesh(source_name,result_ransac.transformation @ source_tfm)
    
    select_objs([target_name],source_name)

def Process_ICP(source_name,target_name,voxel_size,ICP_coef, Aligment_FPFH_coef):
    source,source_tfm = bmesh_to_pcd(source_name,False)
    target,target_tfm = bmesh_to_pcd(target_name,False)

    source, target, source_down, target_down, source_fpfh, target_fpfh = prepare_dataset(
        voxel_size, Aligment_FPFH_coef,source,source_tfm, target,target_tfm)
    result_icp = refine_registration(source_down, target_down, source_fpfh, target_fpfh,
                                     voxel_size,ICP_coef)
    #print(result_icp.transformation)
    transform_bmesh(source_name,result_icp.transformation @ source_tfm)
    
    select_objs([target_name],source_name)

def find_overlap_center(source, target, threshold):
    # KD-Treeを作成して近傍点を検索
    target_tree = o3d.geometry.KDTreeFlann(target)
    
    # オーバーラップ部分の点を格納するリスト
    overlap_points = []
    
    # ソース点群の各点について、ターゲット点群内で近い点を検索
    for point in source.points:
        [k, idx, _] = target_tree.search_knn_vector_3d(point, 1)
        if k > 0:
            distance = np.linalg.norm(np.asarray(point) - np.asarray(target.points)[idx[0]])
            if distance < threshold:
                overlap_points.append(point)
    
    # オーバーラップ部分がない場合はNoneを返す
    if len(overlap_points) == 0:
        return None
    
    # オーバーラップ部分の点群を作成
    overlap_point_cloud = o3d.geometry.PointCloud()
    overlap_point_cloud.points = o3d.utility.Vector3dVector(np.array(overlap_points))
    
    # オーバーラップ部分の中心座標を計算
    overlap_center = np.mean(np.asarray(overlap_point_cloud.points), axis=0)
    
    return overlap_center
    
def Process_Hrz_fix(source_name,target_name,voxel_size,ICP_coef, Aligment_FPFH_coef):
    source,source_tfm = bmesh_to_pcd(source_name,False)
    target,target_tfm = bmesh_to_pcd(target_name,False)
    
    source, target, source_down, target_down, source_fpfh, target_fpfh = prepare_dataset(
        voxel_size, Aligment_FPFH_coef, source,source_tfm, target,target_tfm)
    
    overlap_center = find_overlap_center(source_down, target_down, voxel_size)
    
    return overlap_center    

def Process_Partial_ICP(source_name,target_name,voxel_size,ICP_coef, Aligment_FPFH_coef):
    global ini_verts
    global end_verts
    global source_obj
    global selected_verts_id
    global canAdjustEffect
    global min_distance_list
    
    ini_verts = []
    end_verts = []
    
    obj = bpy.data.objects.get(source_name)
    mesh = obj.data
    bm = bmesh.from_edit_mesh(mesh)
    source_obj = obj
    
    ini_verts = [v.co.copy() for v in bm.verts]
    selected_verts = [v for v in bm.verts if v.select]
    selected_verts_id = [v.index for v in bm.verts if v.select]    
    
    min_distance_list = get_min_connected_distance(obj, selected_verts_id)
    
    source,source_tfm = bmesh_to_pcd(source_name,True)
    target,target_tfm = bmesh_to_pcd(target_name,False)

    source, target, source_down, target_down, source_fpfh, target_fpfh = prepare_dataset(
        voxel_size, Aligment_FPFH_coef,source,source_tfm, target,target_tfm)
    result_icp = refine_registration(source_down, target_down, source_fpfh, target_fpfh,
                                     voxel_size,ICP_coef)
                                    
    matrix4x4_tfm = np_tfm_2_matrix4x4_tfm(np.linalg.inv(source_tfm) @ result_icp.transformation @ source_tfm)
    
    end_verts = [(matrix4x4_tfm @ co) for co in ini_verts]
    
    select_verts(source_obj,selected_verts_id)
    
    canAdjustEffect = True
    bm.free()

def Process_apply_propotional_edit():
    global ini_verts
    global end_verts
    global source_obj
    global selected_verts_id
    global canAdjustEffect
    global min_distance_list
    
    select_verts(source_obj,selected_verts_id)

    el = bpy.context.scene.effective_length
    
    effected_verts_id = []
    
    mesh = source_obj.data
    bm = bmesh.from_edit_mesh(mesh) 
    
    for i, v in enumerate(bm.verts):
        c = 1 - (min_distance_list[i] / el)
        if c < 0:
            c = 0
        else:
            effected_verts_id.append(i)
        v.co = ini_verts[i] + (end_verts[i] - ini_verts[i]) * c

    select_verts(source_obj,effected_verts_id)
    bmesh.update_edit_mesh(mesh)
    bm.free()

def Process_remesh(mesh_list,voxel_size):
    pcd_list= []
    for m in mesh_list:
        pcd, pcd_tfm = bmesh_to_pcd(m.name,False)
        pcd.transform(pcd_tfm)
        pcd_list.append(pcd)
    print(pcd_list)
    
    all_points = []
    all_normals = []
    for pcd in pcd_list:
        all_points.append(np.asarray(pcd.points))
        all_normals.append(np.asarray(pcd.normals))

    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(np.vstack(all_points))
    pcd.normals = o3d.utility.Vector3dVector(np.vstack(all_normals))
    
    pcd_down  = downsize_point_cloud_with_normal(pcd, voxel_size)  
            
    o3d_triangle_mesh, densities = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(pcd_down, depth=9)
    
    return(o3d_triangle_mesh)        



def Process_decimate(mesh_list,decimate_ref_length):
    trs = bpy.app.translations.pgettext
    if bpy.context.active_object == None:
        active_obj_name = None
    else:    
        active_obj_name = bpy.context.active_object.name
        
    for m in mesh_list:
        selectob = bpy.data.objects.get(m.name)
        if selectob == None:
            return False
        for ob in bpy.context.scene.objects:
            ob.select_set(False)
        selectob.select_set(True)
        bpy.context.view_layer.objects.active = selectob
        
        bpy.ops.object.mode_set(mode='EDIT', toggle=False)
        meshdata = bmesh.from_edit_mesh(m.data)
        meshdata.select_mode = {'EDGE'}
        
        l_sum = 0
        meshdata.edges.ensure_lookup_table()
        for i in range (len(meshdata.edges)):
            a = meshdata.edges[i].verts[0].co
            b = meshdata.edges[i].verts[1].co
            l=np.linalg.norm(b-a)
            l_sum += l
        l_ave = l_sum / len(meshdata.edges)
        decimate_ratio = l_ave / decimate_ref_length
        
        print(m.name)
        print(decimate_ratio)
        
        if decimate_ratio < 0.8:                
            bpy.ops.object.mode_set(mode='OBJECT', toggle=False)
            bpy.ops.object.modifier_add(type='DECIMATE')

            bpy.context.object.modifiers[trs("Decimate")].decimate_type = 'COLLAPSE'
            bpy.context.object.modifiers[trs("Decimate")].ratio = decimate_ratio
            bpy.ops.object.modifier_apply(modifier=trs("Decimate"))

    
    mesh_name_list = [m.name for m in mesh_list]
    select_objs(mesh_name_list,active_obj_name)    

def Process_texture_reduction(image_reduction_ratio):
    for image in bpy.data.images:
        if image.name in ['texture','z_shade','mask','scale','Render Result']:
            pass
        else:
            x = int(image.size[0] / float(image_reduction_ratio))
            y = int(image.size[1] / float(image_reduction_ratio))
            image.scale(x,y)

################################################################################
#Blender functions
################################################################################
#to treat incomplete object when import files
def merge_empty_objects(obj):
    if obj.type == 'EMPTY':

        empty_name = obj.name
        
        mesh_objects = [child for child in obj.children if child.type == 'MESH']
        for ob in bpy.context.scene.objects:
            ob.select_set(False)# すべての選択を解除
        for mesh in mesh_objects:
            mesh.select_set(True)
        bpy.context.view_layer.objects.active = mesh_objects[0]
        bpy.ops.object.join()
        
        bpy.data.objects.remove(obj)
        
        print(empty_name)
        mesh_objects = bpy.context.active_object      
        mesh_objects.name=empty_name
        
        return mesh_objects
    else:
        return obj

#for fix horizontal
def fix_horizontal_by_overlap_centor(source_name,overlap_center_in_blender):
    # 新しいEmptyオブジェクトを作成    
    empty_obj = bpy.data.objects.new("Temp_Empty", None)
    bpy.context.collection.objects.link(empty_obj)
    empty_obj2 = bpy.data.objects.new("Temp_Empty2", None)
    bpy.context.collection.objects.link(empty_obj2)

    # 指定した座標位置に移動
    empty_obj.location = overlap_center_in_blender
    empty_obj2.location = overlap_center_in_blender

    # 親オブジェクトを取得
    parent_obj = bpy.data.objects.get(source_name)
    
    for ob in bpy.context.scene.objects:
        ob.select_set(False)# すべての選択を解除
    empty_obj.select_set(True)
    parent_obj.select_set(True)
    bpy.context.view_layer.objects.active = parent_obj
    bpy.ops.object.parent_set(type='OBJECT', keep_transform=True)
    
    for ob in bpy.context.scene.objects:
        ob.select_set(False)# すべての選択を解除
    parent_obj.select_set(True)
    bpy.context.view_layer.objects.active = parent_obj
    bpy.context.object.rotation_mode = 'XYZ'
    bpy.context.object.rotation_euler[0] = 0
    bpy.context.object.rotation_euler[1] = 0
    
    bpy.context.view_layer.update()
        
    location_delta = empty_obj.matrix_world.translation - empty_obj2.matrix_world.translation
    parent_obj.location = parent_obj.location - location_delta
    
    bpy.data.objects.remove(empty_obj, do_unlink=True)
    bpy.data.objects.remove(empty_obj2, do_unlink=True)
    
    bpy.ops.object.mode_set(mode='OBJECT', toggle=False)

#for propotional falloff editing
def get_boundary_verts(mesh_object, selected_verts_id):
    selected_verts_id_set = set(selected_verts_id)
    boundary_verts_id = []
    mesh = mesh_object.data
    for edge in mesh.edges:        
        bool0 = edge.vertices[0] in selected_verts_id_set
        bool1 = edge.vertices[1] in selected_verts_id_set
        if bool0 != bool1:
            if bool0:
                vvv = edge.vertices[0]
            else:
                vvv = edge.vertices[1]
            
            if vvv not in boundary_verts_id:
                boundary_verts_id.append(vvv)
    return boundary_verts_id

def get_edge_length(mesh,edge_index):
    i0 = mesh.edges[edge_index].vertices[0]
    i1 = mesh.edges[edge_index].vertices[1]
    v0 = mesh.vertices[i0].co
    v1 = mesh.vertices[i1].co
    distance = (v0 - v1).length
    return distance

def get_vertConection(mesh_object):
    mesh = mesh_object.data
    aaa = np.zeros((len(mesh.edges)*2, 3))
    for edge in mesh.edges:
        aaa[edge.index,0] = edge.vertices[0]
        aaa[edge.index,1] = edge.vertices[1]
        aaa[edge.index,2] = get_edge_length(mesh,edge.index)
        aaa[len(mesh.edges) + edge.index,0] =edge.vertices[1]
        aaa[len(mesh.edges) + edge.index,1] =edge.vertices[0]
        aaa[len(mesh.edges) + edge.index,2] =aaa[edge.index,2]
    bbb = aaa[np.argsort(aaa[:, 0])]
    
    vertConection = []    
    temp =[]
    i = 0
    for b in bbb:
        if i == int(b[0]):
            temp.append([int(b[1]),b[2]])
        else:
            vertConection.append(temp)
            temp =[]
            i = int(b[0])
            temp.append([int(b[1]),b[2]])
    vertConection.append(temp)

    return vertConection

def get_min_connected_distance(mesh_object, selected_verts_id):
    
    mesh = mesh_object.data
    
    min_dist = [9999.9] * len(mesh.vertices)
    is_active = ['yet'] * len(mesh.vertices)
    conferm_verts = [] * len(mesh.vertices)
    
    boundary_verts_id = get_boundary_verts(mesh_object, selected_verts_id)
    vertConection = get_vertConection(mesh_object)
    
    active_vert_id = copy.copy(boundary_verts_id)
    
    for i in selected_verts_id:
        min_dist[i] = 0
        is_active[i] = 'done'
    
    for i in boundary_verts_id:
        is_active[i] = 'active'
    
    #main itaration
    while (len(active_vert_id) >0):
        active_verts_dist = [9999.9] * len(active_vert_id)
        for i in range(len(active_vert_id)):
            active_verts_dist[i] = min_dist[active_vert_id[i]]
        v_id = active_vert_id[active_verts_dist.index(min(active_verts_dist))]
        
        for n_vert in vertConection[v_id]:
            if is_active[n_vert[0]] !='done':
                if is_active[n_vert[0]] =='yet':
                    is_active[n_vert[0]] ='active'
                    active_vert_id.append(n_vert[0])
                d = min_dist[v_id] + n_vert[1]
                if d < min_dist[n_vert[0]]:
                    min_dist[n_vert[0]] = d
        is_active[v_id] = 'done'
        print(v_id)
        active_vert_id.remove(v_id)
    
    return min_dist
    
#for after process of union    
def delete_small_poly_islands(obj):
    for ob in bpy.context.scene.objects:
        ob.select_set(False)
    obj.select_set(True)
    
    bpy.context.view_layer.objects.active = obj
    bpy.ops.object.mode_set(mode='EDIT', toggle=False)
    mesh = obj.data
    bm = bmesh.from_edit_mesh(mesh)
    
    for vert in bm.verts:
        i = vert.index
        bpy.ops.mesh.select_all(action='DESELECT')
        bpy.ops.mesh.select_linked_pick(deselect=False, delimit=set(), object_index=0, index=i)
        selected_verts_num = len([v for v in bm.verts if v.select])
            
        if selected_verts_num > (len(bm.verts) - selected_verts_num):
            break
    
    bpy.ops.mesh.select_all(action='INVERT')
    bpy.ops.mesh.delete(type='VERT')
    
    bpy.ops.object.mode_set(mode='OBJECT', toggle=False)

def apply_smooth(obj):
    for ob in bpy.context.scene.objects:
        ob.select_set(False)
    obj.select_set(True)
    
    bpy.ops.object.mode_set(mode='OBJECT', toggle=False)
    mod = obj.modifiers.new(name="Smooth", type='SMOOTH')
    mod.factor = 0.5
    mod.iterations = 1
    bpy.ops.object.modifier_apply(modifier="Smooth")
    
    bpy.ops.object.shade_smooth()

def make_object_collection(collection_name):
    for c in bpy.data.collections:
        if c.name == collection_name:
            return c
    newCollection = bpy.data.collections.new(collection_name)

    bpy.context.scene.collection.children.link(newCollection)
    return newCollection

def delete_collection(collection_name):
    for c in bpy.data.collections:
        if c.name == collection_name:
            bpy.ops.object.select_all(action='DESELECT')
            for remove_obj in c.all_objects:                
                remove_obj.select_set(True)
            bpy.ops.object.delete() # Delete object
            
            bpy.data.collections.remove(c)

def get_objs_in_collection(collection_name):
    for c in bpy.data.collections:
        if c.name == collection_name:
            return c.all_objects
        
        
### bake texture to union
def max_blending(temp_image, comp_image):
    """
    temp_imageとcomp_imageを「比較明」で重ねて処理し、結果をcomp_imageに上書きする関数。
    
    Parameters:
    temp_image (bpy.types.Image): 比較する一時的なテクスチャイメージ。
    comp_image (bpy.types.Image): 上書きされる比較対象のテクスチャイメージ。
    """
    
    """
    # temp_imageとcomp_imageのサイズが同じであることを確認
    if temp_image.size != comp_image.size:
        print("テクスチャイメージのサイズが一致しません。")
        return
    """
    
    width, height = temp_image.size
    
    # ピクセルデータを取得
    temp_pixels = np.array(temp_image.pixels[:])
    comp_pixels = np.array(comp_image.pixels[:])
    
    # ピクセルデータの長さ確認
    pixel_length = width * height * 4  # RGBA なので4チャンネル
    
    if len(temp_pixels) != pixel_length or len(comp_pixels) != pixel_length:
        print("ピクセルデータの長さが一致しません。")
        return
    
    # 比較明処理を行う
    result_pixels = np.maximum(temp_pixels, comp_pixels)
    
    # 結果をcomp_imageに上書き
    comp_image.pixels = result_pixels.tolist()
    comp_image.update()


def smart_uv_project_and_setup_material(obj, texture_size):
    """
    対象オブジェクトをUV展開し、新しいマテリアルとテクスチャイメージを設定する関数。    
    Parameters:
    obj (bpy.types.Object): UV展開とマテリアル設定を行うポリゴンメッシュオブジェクト。
    texture_size (int): 新規作成するテクスチャイメージの幅と高さ。
    """
    # 1. 対象オブジェクトをアクティブにし、編集モードに切り替え
    bpy.context.view_layer.objects.active = obj
    for ob in bpy.context.scene.objects:
        ob.select_set(False)
    obj.select_set(True)
    bpy.context.view_layer.objects.active = obj
    
    bpy.ops.object.mode_set(mode='EDIT')
    bpy.ops.mesh.select_all(action='SELECT')
    
    # Smart UV Projectを実行
    bpy.ops.uv.smart_project(
        angle_limit=0.5236, #use rad, not deg.
        margin_method = 'ADD',
        island_margin = bpy.context.scene.island_margin,
        area_weight=0.0,
        correct_aspect=True,
        scale_to_bounds=False
    )
    
    # オブジェクトモードに戻す
    bpy.ops.object.mode_set(mode='OBJECT')
    
    # 2. 同名のマテリアルが存在する場合は削除し、新しいマテリアルを作成
    if "caveMaterial" in bpy.data.materials:
        bpy.data.materials.remove(bpy.data.materials["caveMaterial"])
    new_material = bpy.data.materials.new(name="caveMaterial")
    
    # 既存のマテリアルを削除し、新しいマテリアルを適用
    obj.data.materials.clear()
    obj.data.materials.append(new_material)
    
    # 3. 同名のテクスチャイメージが存在する場合は削除し、新しいテクスチャイメージを作成
    if "temp texture" in bpy.data.images:
        bpy.data.images.remove(bpy.data.images["temp texture"])
    temp_texture_image = bpy.data.images.new(name="temp texture", width=texture_size, height=texture_size, alpha=True)    
    # テクスチャイメージを (0, 0, 0, 0) で初期化
    pixels = [0.0, 0.0, 0.0, 0.0] * (texture_size * texture_size) 
    temp_texture_image.pixels = pixels 
    temp_texture_image.update()
    
    # 3. 同名のテクスチャイメージが存在する場合は削除し、新しいテクスチャイメージを作成
    if "comp texture" in bpy.data.images:
        bpy.data.images.remove(bpy.data.images["comp texture"])
    comp_texture_image = bpy.data.images.new(name="comp texture", width=texture_size, height=texture_size, alpha=True)    
    # テクスチャイメージを (0, 0, 0, 0) で初期化
    pixels = [0.0, 0.0, 0.0, 0.0] * (texture_size * texture_size) 
    comp_texture_image.pixels = pixels 
    comp_texture_image.update()

    # マテリアルにノードを追加してテクスチャイメージを設定
    if not new_material.use_nodes:
        new_material.use_nodes = True
    nodes = new_material.node_tree.nodes
    links = new_material.node_tree.links
    
    temp_texture_node = nodes.new(type='ShaderNodeTexImage')
    temp_texture_node.image = temp_texture_image
    
    comp_texture_node = nodes.new(type='ShaderNodeTexImage')
    comp_texture_node.image = comp_texture_image
    
    # 既存のPrincipled BSDFノードを取得
    principled_bsdf_node = None
    for node in nodes:
        if node.type == 'BSDF_PRINCIPLED':
            principled_bsdf_node = node
            break
    
    # comp_texture_nodeのColor出力をPrincipled BSDFノードのBase Color入力に接続
    if principled_bsdf_node is not None:        
        links.new(comp_texture_node.outputs['Color'], principled_bsdf_node.inputs['Base Color'])

def bake_to_existing_texture(source_obj, target_obj):
    # ターゲットオブジェクトをアクティブに
    bpy.ops.object.mode_set(mode='OBJECT')
    for ob in bpy.context.scene.objects:
        ob.select_set(False)
    source_obj.select_set(True)
    target_obj.select_set(True)
    bpy.context.view_layer.objects.active = target_obj
    
    # 既存の「temp texture」テクスチャイメージノードを取得
    temp_texture_node = None
    for mat in target_obj.data.materials:
        if mat.use_nodes:
            for node in mat.node_tree.nodes:
                if node.type == 'TEX_IMAGE' and node.image.name == 'temp texture':
                    temp_texture_node = node
                    break
        if temp_texture_node:
            break

    if not temp_texture_node:
        print("ターゲットオブジェクトに 'temp texture' テクスチャイメージが見つかりませんでした。")
        return

    # テクスチャノードをアクティブに設定
    bpy.context.view_layer.objects.active = target_obj
    bpy.context.view_layer.objects.active.active_material_index = 0
    target_obj.active_material.node_tree.nodes.active = temp_texture_node

    # ベイク設定を行う
    bake_common_setting()
    bpy.ops.object.bake(
        type='DIFFUSE',
        pass_filter={'COLOR'},
        margin = 2,
        use_selected_to_active = True,
        use_cage = False,
        cage_extrusion = bpy.context.scene.union_bake_extrusion,
        max_ray_distance = bpy.context.scene.union_bake_max_ray_distance,
        use_clear = True,
        target='IMAGE_TEXTURES'
    )
    

def overwrite_alpha_zero_pixels(comp_image, custom_color):
    """
    アルファ値がゼロのピクセルをカスタムカラーで上書きする関数。

    Parameters:
    comp_image (bpy.types.Image): 上書きするテクスチャイメージ。
    custom_color (tuple): 使用するカスタムカラー(R, G, B, A)。
    """
    # テクスチャイメージのサイズを取得
    width, height = comp_image.size
    
    # ピクセルデータを取得
    pixels = np.array(comp_image.pixels[:])
    
    # 各ピクセルをチェックし、アルファ値がゼロの場合にカスタムカラーで上書き
    for i in range(0, len(pixels), 4):
        if pixels[i+3] == 0.0:  # アルファ値がゼロ
            pixels[i:i+4] = custom_color
    
    # 更新されたピクセルデータをテクスチャイメージに設定
    comp_image.pixels = pixels.tolist()
    comp_image.update()    

def bake_texture_to_union(texture_size):
    selected_objects = bpy.context.selected_objects
    target_obj = bpy.context.view_layer.objects.active

    if len(selected_objects) >= 2 and target_obj is not None:
        
        source_obj_list = [obj for obj in selected_objects if obj != target_obj]
        smart_uv_project_and_setup_material(target_obj, texture_size)

        for source_obj in source_obj_list:
            bake_to_existing_texture(source_obj, target_obj)
            
            temp_image = bpy.data.images['temp texture']
            comp_image = bpy.data.images['comp texture']
            max_blending(temp_image, comp_image)
        
        overwrite_alpha_zero_pixels(comp_image, bpy.context.scene.bake_base_color)  
        
        for material in target_obj.data.materials:
            if material and material.use_nodes:
                nodes = material.node_tree.nodes
                # ノードツリー内のすべてのノードを走査
                for node in nodes:
                    if node.type == 'TEX_IMAGE' and node.image and node.image.name == "temp texture":
                        # ノードを削除
                        nodes.remove(node)
        if "temp texture" in bpy.data.images:
            bpy.data.images.remove(bpy.data.images["temp texture"])

#closs section
def cave_model_pre_prs(obj):
    trs = bpy.app.translations.pgettext
    #set definition of collections
    import_obj_collection = make_object_collection('Import Models')
    texture_obj_collection = make_object_collection('Texture Bake Models')
    mask_obj_collection = make_object_collection('Mask Bake Models')
    
    #set select and active the obj
    for ob in bpy.context.scene.objects:
        ob.select_set(False)
    
    obj.select_set(True)
    bpy.context.view_layer.objects.active = obj
    
    #add solidify modifier
    bpy.ops.object.mode_set(mode='OBJECT', toggle=False)
    #bpy.ops.object.modifier_add(type='SOLIDIFY')
    #bpy.context.object.modifiers[trs("Solidify")].thickness = 0.0001

        
    #dupulicate texture bake model
    bpy.ops.object.duplicate_move(OBJECT_OT_duplicate=None, TRANSFORM_OT_translate=None)
    bpy.ops.object.modifier_add(type='SOLIDIFY')
    bpy.context.object.modifiers[trs("Solidify")].thickness = 0.0001
    texture_obj = bpy.context.selected_objects[0]
    import_obj_collection.objects.unlink(texture_obj)
    texture_obj_collection.objects.link(texture_obj)
    
    #dupulicate mask bake model 
    bpy.ops.object.duplicate_move(OBJECT_OT_duplicate=None, TRANSFORM_OT_translate=None)
    mask_obj = bpy.context.selected_objects[0]
    texture_obj_collection.objects.unlink(mask_obj)
    mask_obj_collection.objects.link(mask_obj)
    
    
    #set material ID offset
    bpy.context.view_layer.objects.active = mask_obj
    bpy.context.object.modifiers[trs("Solidify")].material_offset = 1
    #set material 
    for num in range(len(mask_obj.material_slots))[::-1]:
        targetmat=mask_obj.material_slots[num].material
        mask_obj.active_material_index=num
        bpy.ops.object.material_slot_remove()
    for material in bpy.data.materials:
        if material.name == 'inner':
            bpy.context.object.data.materials.append(material)
    for material in bpy.data.materials:
        if material.name == 'outer':
            bpy.context.object.data.materials.append(material)
    
def cs_model_pre_prs(obj):
    scene = bpy.context.scene
    image_size = scene.image_size
    scale_length = scene.scale_length 
    
    create_images(image_size)
    create_material_CrossSection()
    generate_scale(scale_length,0.002)
    bpy.context.view_layer.objects.active = obj
    # set material
    for num in range(len(obj.material_slots))[::-1]:
        targetmat=obj.material_slots[num].material
        obj.active_material_index=num
        bpy.ops.object.material_slot_remove()
    for material in bpy.data.materials:
        if material.name == 'CrossSection':
            bpy.context.object.data.materials.append(material)
    
    #Join scale_plane obj to cs_model
    obj.select_set(True)
    bpy.data.objects['scale_plane'].select_set(True)
    bpy.ops.object.join()

    #UV unwrap
    bpy.ops.object.mode_set(mode='EDIT', toggle=False)
    bpy.ops.mesh.select_all(action='SELECT')
    bpy.context.scene.tool_settings.use_uv_select_sync = True
    bpy.ops.uv.unwrap(method='ANGLE_BASED', margin=0.001)
    bpy.ops.object.mode_set(mode='OBJECT', toggle=False)

    #separate scale_plane
    bpy.ops.object.mode_set(mode='EDIT', toggle=False)
    meshdata = bmesh.from_edit_mesh(obj.data)
    meshdata.select_mode = {'FACE'}
    bpy.ops.mesh.select_all(action='DESELECT')
    meshdata.faces.ensure_lookup_table()
    meshdata.faces[len(meshdata.faces)-1].select_set(True)
    bpy.ops.mesh.separate(type='SELECTED')

    #rename and replace collection
    for ob in bpy.context.selected_objects :
        if ob != bpy.context.active_object:
            ob.name = 'scale_plane'
            for collection in bpy.data.collections:
                checklink = collection.objects.get(ob.name)
                if checklink != None:
                    collection.objects.unlink(ob)
            destinationcollection = bpy.data.collections.get('Scale')
            destinationcollection.objects.link(ob)
    bpy.ops.object.mode_set(mode='OBJECT', toggle=False)


def generate_scale(L,t):
    #initalize collection
    scale_collection = make_object_collection('Scale')

    vlayer = bpy.context.view_layer
    vlayer.layer_collection.children['Scale'].hide_viewport = False

    for c in bpy.data.collections:
        if c.name == scale_collection.name:
            bpy.ops.object.select_all(action='DESELECT')
            for remove_obj in c.all_objects:
                remove_obj.hide_set(False)
                remove_obj.select_set(True)
            bpy.ops.object.delete() # Delete object


    #generate_sclae_line
    r = 0.5 * t * L
    line_obj_name = 'scale_line'
    curvedata = bpy.data.curves.new(line_obj_name, type='CURVE')    
    curvedata.dimensions = '3D'
    
    polyline = curvedata.splines.new('BEZIER')
    polyline.bezier_points.add(1) 

    polyline.bezier_points[0].co = 0,r,0
    polyline.bezier_points[0].handle_left = 0,r,0
    polyline.bezier_points[0].handle_right = 0,r,0

    polyline.bezier_points[1].co = L,r,0
    polyline.bezier_points[1].handle_left = L,r,0
    polyline.bezier_points[1].handle_right = L,r,0
    
    for i in range(11):
        polyline = curvedata.splines.new('BEZIER')
        polyline.bezier_points.add(1) 

        if i in [0,5,10]:
            polyline.bezier_points[0].co = L *i* 0.1,L * 0.05,0
            polyline.bezier_points[0].handle_left = L *i* 0.1,L * 0.05,0
            polyline.bezier_points[0].handle_right = L *i* 0.1,L * 0.05,0
        else:
            polyline.bezier_points[0].co = L *i* 0.1,L * 0.03,0
            polyline.bezier_points[0].handle_left = L *i* 0.1,L * 0.03,0
            polyline.bezier_points[0].handle_right = L *i* 0.1,L * 0.03,0

        polyline.bezier_points[1].co = L *i* 0.1,0,0
        polyline.bezier_points[1].handle_left = L *i* 0.1,0,0
        polyline.bezier_points[1].handle_right = L *i* 0.1,0,0

    bezier_obj = bpy.data.objects.new(line_obj_name, curvedata) 
    bezier_obj.data.bevel_depth = r
    bezier_obj.data.bevel_resolution = 0
    bezier_obj.name = line_obj_name
    bezier_obj.data.name = line_obj_name
    
    #generate_scale_text
    text_obj_name = 'scale_text'
    fontCurve1 = bpy.data.curves.new(type="FONT",name=text_obj_name)
    text_obj = bpy.data.objects.new(text_obj_name,fontCurve1)
    text_obj.data.body = str(L) + 'm'
    text_obj.data.align_x = 'RIGHT'
    text_obj.data.align_y = 'TOP'
    text_obj.data.size = L *0.1
    text_obj.location[0] = L*1.09
    text_obj.name = text_obj_name
    text_obj.data.name = text_obj_name
    
    #generate_scale_plane
    plane_obj_name = 'scale_plane'
    l = 0.1*L
    verts = [(-l,l,l),(-l,-l,l),(L+l,-l,l),(L+l,l,l)]
    faces = [(0,1,2,3)]
    
    msh = bpy.data.meshes.new(plane_obj_name) 
    msh.from_pydata(verts, [], faces) 
    plane_obj = bpy.data.objects.new(plane_obj_name, msh) 
    plane_obj.name = plane_obj_name
    plane_obj.data.name = plane_obj_name
    
    #link objects to collection
    scale_collection.objects.link(bezier_obj)
    scale_collection.objects.link(text_obj)
    scale_collection.objects.link(plane_obj)
    
    #set material
    create_material_inner_outer('scale', 1.0)
    
    for obj in [bezier_obj,text_obj]:
        bpy.context.view_layer.objects.active = obj
        for num in range(len(obj.material_slots))[::-1]:
            targetmat=obj.material_slots[num].material
            obj.active_material_index=num
            bpy.ops.object.material_slot_remove()
        for material in bpy.data.materials:
            if material.name == 'scale':
                bpy.context.object.data.materials.append(material)
    
    for obj in [plane_obj]:
        bpy.context.view_layer.objects.active = obj
        for num in range(len(obj.material_slots))[::-1]:
            targetmat=obj.material_slots[num].material
            obj.active_material_index=num
            bpy.ops.object.material_slot_remove()
        for material in bpy.data.materials:
            if material.name == 'CrossSection':
                bpy.context.object.data.materials.append(material)
            
################################################################################
#generate materials, images and composite

def clear_materials_and_images():
    #delete all materials
    for material in bpy.data.materials:
        bpy.data.materials.remove(material)
    #delete all texrure images
    for image in bpy.data.images:
        bpy.data.images.remove(image)

def create_images(p):
    for image in bpy.data.images:
        if image.name in ['texture','z_shade','mask','scale']:
            bpy.data.images.remove(image)
        
    bake_texture = bpy.data.images.new(name = 'texture', width=p, height=p)
    bake_z_shade = bpy.data.images.new(name = 'z_shade', width=p, height=p)
    bake_mask = bpy.data.images.new(name = 'mask', width=p, height=p)
    bake_scale = bpy.data.images.new(name = 'scale', width=p, height=p)

def resize_images(p):
    for image in bpy.data.images:
        if image.name in ['texture','z_shade','mask','scale']:
            image.scale(p,p)

def create_material_CrossSection():
    for material in bpy.data.materials:
        if material.name == 'CrossSection':
            bpy.data.materials.remove(material)
    
    for image in bpy.data.images:
        if image.name == 'texture':
            bake_texture = image
        if image.name == 'z_shade':
            bake_z_shade = image
        if image.name == 'mask':
            bake_mask = image
        if image.name == 'scale':
            bake_scale = image
    
    material_cs = bpy.data.materials.new('CrossSection')
    material_cs.use_nodes = True  
    nodes = material_cs.node_tree.nodes
    links = material_cs.node_tree.links
    
    #delete pre-set nodes
    for n in nodes:
        nodes.remove(n)
    
    # add_shader_node
    bsdf_node = nodes.new(type="ShaderNodeBsdfDiffuse")
    bsdf_node.label ='Diffuse BSDF'
    bsdf_node.location = (0, 0)

    # add_output_node
    output_node = nodes.new(type="ShaderNodeOutputMaterial")
    output_node.label = 'Material Output'
    output_node.location = (200, 0)
    
    # add_texture_node
    image_texture_node = nodes.new(type="ShaderNodeTexImage")
    image_texture_node.location = (-300, 0)
    image_texture_node.label = 'Texture Image'
    image_texture_node.image = bake_texture
    
    # add_z_shade_node
    image_z_shade_node = nodes.new(type="ShaderNodeTexImage")
    image_z_shade_node.location = (-300, -300)
    image_z_shade_node.label = 'Z Shade Image'
    image_z_shade_node.image = bake_z_shade
    
    # add_mask_node
    image_mask_node = nodes.new(type="ShaderNodeTexImage")
    image_mask_node.location = (-300, -600)
    image_mask_node.label = 'Mask Image'
    image_mask_node.image = bake_mask
    
    # add_scale_node
    image_scale_node = nodes.new(type="ShaderNodeTexImage")
    image_scale_node.location = (-300, -900)
    image_scale_node.label = 'Scale Image'
    image_scale_node.image = bake_scale
    
    #conect_links
    links.new(bsdf_node.outputs[0], output_node.inputs[0])
    
def get_material_by_name(name):
    for material in bpy.data.materials:
        if material.name == name:
            return material
        
def get_material_node_by_label(material,label):
    nodes = material.node_tree.nodes
    for n in nodes:
        if n.label == label:
            return n       
    
def create_material_inner_outer(name,v):
    for material in bpy.data.materials:
        if material.name == name:
            bpy.data.materials.remove(material)
    
    material_cs = bpy.data.materials.new(name)
    material_cs.use_nodes = True  
    nodes = material_cs.node_tree.nodes
    links = material_cs.node_tree.links
    
    #delete pre-set nodes
    for n in nodes:
        nodes.remove(n)
    
    # add_shader_node
    bsdf_node = nodes.new(type="ShaderNodeBsdfDiffuse")
    bsdf_node.location = (0, 0)
    bsdf_node.inputs[0].default_value = (v,v,v,1)

    # add_output_node
    output_node = nodes.new(type="ShaderNodeOutputMaterial")
    output_node.location = (200, 0)
    
    #conect_links
    links.new(bsdf_node.outputs[0], output_node.inputs[0])

def make_nodes(node_tree, node_set):
    node_list = []
    for n_ in node_set:
        if "bl_idname" in n_:
            # ノードの作成
            node = node_tree.nodes.new( n_.pop("bl_idname") )
            # アトリビュートの設定
            for attr_name in n_.keys():
                if  hasattr(node, attr_name):
                    if attr_name == "color_ramp":
                        for i_,var_ in enumerate(n_[attr_name]):
                            if i_ > 1:
                                node.color_ramp.elements.new(0.100)
                        for i_,var_ in enumerate(n_[attr_name]):
                            node.color_ramp.elements[i_].position = var_[0]
                            node.color_ramp.elements[i_].color = (var_[1],var_[2],var_[3],var_[4])
                    elif attr_name == "inputs":
                        for i_,var_ in enumerate(n_[attr_name]):
                            if var_: node.inputs[i_].default_value = var_
                    elif attr_name in ['image', 'object' , 'operation',  'uv_map']:
                        try:setattr(node, attr_name, n_[attr_name])
                        except:print('%s:%s was not set' % (node.name, attr_name))
                    else: setattr(node, attr_name, n_[attr_name])
            node_list.append(node)
        if "node_linkes" in n_:
            for L in n_["node_linkes"]:
                node_tree.links.new( node_list[ L[0] ].outputs[ L[1] ], node_list[ L[2] ].inputs[ L[3] ] )

def get_node_set():
    node_set = [
    {'bl_idname': 'CompositorNodeMixRGB', 'name': 'Mix', 'label': '', 'height': 100.0, 'width': 140.0, 'location': Vector((425.327880859375, 467.68853759765625)), 'blend_type': 'MULTIPLY', 'inputs': [1.0, [1.0, 1.0, 1.0, 1.0], [1.0, 1.0, 1.0, 1.0]]},
    {'bl_idname': 'CompositorNodeDilateErode', 'name': 'Dilate/Erode', 'label': '', 'height': 100.0, 'width': 140.0, 'location': Vector((167.22454833984375, 525.585693359375)), 'inputs': [0.0]},
    {'bl_idname': 'CompositorNodeInvert', 'name': 'Invert.001', 'label': '', 'height': 100.0, 'width': 140.0, 'location': Vector((640.4141845703125, 451.358642578125)), 'invert_alpha': True, 'invert_rgb': False, 'inputs': [1.0, [1.0, 1.0, 1.0, 1.0]]},
    {'bl_idname': 'CompositorNodeInvert', 'name': 'Invert', 'label': '', 'height': 100.0, 'width': 140.0, 'location': Vector((107.641357421875, 347.4407958984375)), 'invert_alpha': False, 'invert_rgb': True, 'inputs': [1.0, [1.0, 1.0, 1.0, 1.0]]},
    {'bl_idname': 'CompositorNodeAlphaOver', 'name': 'Alpha Over', 'label': '', 'height': 100.0, 'width': 140.0, 'location': Vector((885.1673583984375, 473.626708984375)), 'inputs': [1.0, [1.0, 1.0, 1.0, 1.0], [0.0, 0.0, 0.0, 1.0]]},
    {'bl_idname': 'CompositorNodeMixRGB', 'name': 'Mix.002', 'label': '', 'height': 100.0, 'width': 140.0, 'location': Vector((746.146240234375, 1032.6480712890625)), 'blend_type': 'LIGHTEN', 'inputs': [1.0, [1.0, 1.0, 1.0, 1.0], [1.0, 1.0, 1.0, 1.0]]},
    {'bl_idname': 'CompositorNodeMixRGB', 'name': 'Mix.001', 'label': '', 'height': 100.0, 'width': 140.0, 'location': Vector((734.073974609375, 803.1356201171875)), 'blend_type': 'LIGHTEN', 'inputs': [1.0, [1.0, 1.0, 1.0, 1.0], [1.0, 1.0, 1.0, 1.0]]},
    {'bl_idname': 'CompositorNodeMixRGB', 'name': 'Mix.003', 'label': '', 'height': 100.0, 'width': 140.0, 'location': Vector((299.1533203125, -987.6131591796875)), 'blend_type': 'MULTIPLY', 'inputs': [1.0, [1.0, 1.0, 1.0, 1.0], [1.0, 1.0, 1.0, 1.0]]},
    {'bl_idname': 'CompositorNodeInvert', 'name': 'Invert.002', 'label': '', 'height': 100.0, 'width': 140.0, 'location': Vector((514.2392578125, -1003.9429931640625)), 'invert_alpha': True, 'invert_rgb': False, 'inputs': [1.0, [1.0, 1.0, 1.0, 1.0]]},
    {'bl_idname': 'CompositorNodeInvert', 'name': 'Invert.003', 'label': '', 'height': 100.0, 'width': 140.0, 'location': Vector((24.76513671875, -1127.119873046875)), 'invert_alpha': False, 'invert_rgb': True, 'inputs': [1.0, [1.0, 1.0, 1.0, 1.0]]},
    {'bl_idname': 'CompositorNodeDilateErode', 'name': 'Dilate/Erode.001', 'label': '', 'height': 100.0, 'width': 140.0, 'location': Vector((28.018310546875, -928.629150390625)), 'inputs': [0.0]},
    {'bl_idname': 'CompositorNodeAlphaOver', 'name': 'Alpha Over.003', 'label': '', 'height': 100.0, 'width': 140.0, 'location': Vector((758.99267578125, -981.6749267578125)), 'inputs': [1.0, [1.0, 1.0, 1.0, 1.0], [0.0, 0.0, 0.0, 1.0]]},
    {'bl_idname': 'CompositorNodeMixRGB', 'name': 'Mix.004', 'label': '', 'height': 100.0, 'width': 140.0, 'location': Vector((750.5654296875, -735.1914672851562)), 'blend_type': 'MULTIPLY', 'inputs': [1.0, [1.0, 1.0, 1.0, 1.0], [1.0, 1.0, 1.0, 1.0]]},
    {'bl_idname': 'CompositorNodeMixRGB', 'name': 'Mix.006', 'label': '', 'height': 100.0, 'width': 140.0, 'location': Vector((1069.5902099609375, -616.5360717773438)), 'blend_type': 'LIGHTEN', 'inputs': [1.0, [1.0, 1.0, 1.0, 1.0], [1.0, 1.0, 1.0, 1.0]]},
    {'bl_idname': 'CompositorNodeMixRGB', 'name': 'Mix.007', 'label': '', 'height': 100.0, 'width': 140.0, 'location': Vector((1077.8934326171875, -286.9171142578125)), 'blend_type': 'LIGHTEN', 'inputs': [1.0, [1.0, 1.0, 1.0, 1.0], [1.0, 1.0, 1.0, 1.0]]},
    {'bl_idname': 'CompositorNodeAlphaOver', 'name': 'Alpha Over.004', 'label': '', 'height': 100.0, 'width': 140.0, 'location': Vector((1217.3046875, -960.2227783203125)), 'inputs': [1.0, [1.0, 1.0, 1.0, 1.0], [1.0, 1.0, 1.0, 1.0]]},
    {'bl_idname': 'CompositorNodeMixRGB', 'name': 'Mix.005', 'label': '', 'height': 100.0, 'width': 140.0, 'location': Vector((747.3984375, -408.047119140625)), 'blend_type': 'MULTIPLY', 'inputs': [1.0, [1.0, 1.0, 1.0, 1.0], [1.0, 1.0, 1.0, 1.0]]},
    {'bl_idname': 'CompositorNodeMixRGB', 'name': 'Mix.011', 'label': '', 'height': 100.0, 'width': 140.0, 'location': Vector((1801.52734375, -766.0072021484375)), 'blend_type': 'MULTIPLY', 'inputs': [1.0, [1.0, 1.0, 1.0, 1.0], [1.0, 1.0, 1.0, 1.0]]},
    {'bl_idname': 'CompositorNodeAlphaOver', 'name': 'Alpha Over.005', 'label': '', 'height': 100.0, 'width': 140.0, 'location': Vector((1553.46826171875, -787.7064819335938)), 'inputs': [1.0, [1.0, 1.0, 1.0, 1.0], [1.0, 1.0, 1.0, 1.0]]},
    {'bl_idname': 'CompositorNodeMixRGB', 'name': 'Mix.010', 'label': '', 'height': 100.0, 'width': 140.0, 'location': Vector((1795.378173828125, -214.82598876953125)), 'blend_type': 'MULTIPLY', 'inputs': [1.0, [1.0, 1.0, 1.0, 1.0], [1.0, 1.0, 1.0, 1.0]]},
    {'bl_idname': 'CompositorNodeAlphaOver', 'name': 'Alpha Over.006', 'label': '', 'height': 100.0, 'width': 140.0, 'location': Vector((1555.802734375, -302.8038330078125)), 'inputs': [1.0, [1.0, 1.0, 1.0, 1.0], [1.0, 1.0, 1.0, 1.0]]},
    {'bl_idname': 'CompositorNodeAlphaOver', 'name': 'Alpha Over.001', 'label': '', 'height': 100.0, 'width': 140.0, 'location': Vector((1551.1943359375, 595.0882568359375)), 'inputs': [1.0, [1.0, 1.0, 1.0, 1.0], [1.0, 1.0, 1.0, 1.0]]},
    {'bl_idname': 'CompositorNodeMixRGB', 'name': 'Mix.009', 'label': '', 'height': 100.0, 'width': 140.0, 'location': Vector((1796.907470703125, 633.22216796875)), 'blend_type': 'MULTIPLY', 'inputs': [1.0, [1.0, 1.0, 1.0, 1.0], [1.0, 1.0, 1.0, 1.0]]},
    {'bl_idname': 'CompositorNodeAntiAliasing', 'name': 'Anti-Aliasing.001', 'label': '', 'height': 100.0, 'width': 170.0, 'location': Vector((2049.77392578125, 606.2823486328125)), 'inputs': [[1.0, 1.0, 1.0, 1.0]]},
    {'bl_idname': 'CompositorNodeValToRGB', 'name': 'ColorRamp', 'label': '', 'height': 100.0, 'width': 240.0, 'location': Vector((-295.327392578125, -111.88818359375)), 'inputs': [0.5], 'color_ramp': [[0.05000000819563866, 1.0, 1.0, 1.0, 1.0], [0.09090911597013474, 0.0, 0.0, 0.0, 1.0]]},
    {'bl_idname': 'CompositorNodeValToRGB', 'name': 'ColorRamp.002', 'label': '', 'height': 100.0, 'width': 240.0, 'location': Vector((-761.8375244140625, -163.9312744140625)), 'inputs': [0.5], 'color_ramp': [[0.0, 0.0, 0.0, 0.0, 1.0], [0.5, 0.15250837802886963, 0.15250837802886963, 0.15250837802886963, 1.0], [1.0, 1.0, 1.0, 1.0, 1.0]]},
    {'bl_idname': 'CompositorNodeValToRGB', 'name': 'ColorRamp.004', 'label': '', 'height': 100.0, 'width': 240.0, 'location': Vector((-276.9508056640625, -607.1824951171875)), 'inputs': [0.5], 'color_ramp': [[0.0, 0.0, 0.0, 0.0, 1.0], [0.5, 0.05380385369062424, 0.05380385369062424, 0.05380385369062424, 1.0], [1.0, 1.0, 1.0, 1.0, 1.0]]},
    {'bl_idname': 'CompositorNodeValToRGB', 'name': 'ColorRamp.003', 'label': '', 'height': 100.0, 'width': 240.0, 'location': Vector((-297.7109375, -1045.5362548828125)), 'inputs': [0.5], 'color_ramp': [[0.0, 0.0, 0.0, 0.0, 1.0], [0.11818180233240128, 1.0, 1.0, 1.0, 1.0]]},
    {'bl_idname': 'CompositorNodeInvert', 'name': 'Invert.004', 'label': '', 'height': 100.0, 'width': 140.0, 'location': Vector((280.585205078125, 1397.4453125)), 'invert_alpha': True, 'invert_rgb': False, 'inputs': [1.0, [1.0, 1.0, 1.0, 1.0]]},
    {'bl_idname': 'CompositorNodeInvert', 'name': 'Invert.005', 'label': '', 'height': 100.0, 'width': 140.0, 'location': Vector((576.0184326171875, 1435.311767578125)), 'invert_alpha': False, 'invert_rgb': True, 'inputs': [1.0, [1.0, 1.0, 1.0, 1.0]]},
    {'bl_idname': 'CompositorNodeValToRGB', 'name': 'ColorRamp.001', 'label': '', 'height': 100.0, 'width': 240.0, 'location': Vector((836.418701171875, 1438.7021484375)), 'inputs': [0.5], 'color_ramp': [[0.0, 0.0, 0.0, 0.0, 1.0], [1.0, 1.0, 1.0, 1.0, 1.0]]},
    {'bl_idname': 'CompositorNodeValToRGB', 'name': 'ColorRamp.006', 'label': '', 'height': 100.0, 'width': 240.0, 'location': Vector((-282.6741943359375, 450.1886901855469)), 'inputs': [0.5], 'color_ramp': [[0.6909090280532837, 0.0, 0.0, 0.0, 1.0], [0.7681819796562195, 1.0, 1.0, 1.0, 1.0]]},
    {'bl_idname': 'CompositorNodeImage', 'name': 'Image.004', 'label': 'UVtemp', 'height': 100.0, 'width': 230.49871826171875, 'location': Vector((1998.00830078125, 292.44464111328125)), 'inputs': []},
    {'bl_idname': 'CompositorNodeAlphaOver', 'name': 'Alpha Over.008', 'label': '', 'height': 100.0, 'width': 140.0, 'location': Vector((2587.6103515625, 586.7081298828125)), 'inputs': [1.0, [1.0, 1.0, 1.0, 1.0], [1.0, 1.0, 1.0, 1.0]]},
    {'bl_idname': 'CompositorNodeAntiAliasing', 'name': 'Anti-Aliasing.002', 'label': '', 'height': 100.0, 'width': 170.0, 'location': Vector((2038.6953125, -325.9761962890625)), 'inputs': [[1.0, 1.0, 1.0, 1.0]]},
    {'bl_idname': 'CompositorNodeAlphaOver', 'name': 'Alpha Over.009', 'label': '', 'height': 100.0, 'width': 140.0, 'location': Vector((2535.4306640625, -262.5286865234375)), 'inputs': [1.0, [1.0, 1.0, 1.0, 1.0], [1.0, 1.0, 1.0, 1.0]]},
    {'bl_idname': 'CompositorNodeAntiAliasing', 'name': 'Anti-Aliasing.003', 'label': '', 'height': 100.0, 'width': 170.0, 'location': Vector((2031.30859375, -776.5870971679688)), 'inputs': [[1.0, 1.0, 1.0, 1.0]]},
    {'bl_idname': 'CompositorNodeAlphaOver', 'name': 'Alpha Over.010', 'label': '', 'height': 100.0, 'width': 140.0, 'location': Vector((2532.1953125, -770.23681640625)), 'inputs': [1.0, [1.0, 1.0, 1.0, 1.0], [1.0, 1.0, 1.0, 1.0]]},
    {'bl_idname': 'CompositorNodeAlphaOver', 'name': 'Alpha Over.007', 'label': '', 'height': 100.0, 'width': 140.0, 'location': Vector((2583.333984375, 979.9429931640625)), 'inputs': [1.0, [1.0, 1.0, 1.0, 1.0], [1.0, 1.0, 1.0, 1.0]]},
    {'bl_idname': 'CompositorNodeMixRGB', 'name': 'Mix.008', 'label': '', 'height': 100.0, 'width': 140.0, 'location': Vector((1847.867919921875, 1001.1924438476562)), 'blend_type': 'MULTIPLY', 'inputs': [1.0, [1.0, 1.0, 1.0, 1.0], [1.0, 1.0, 1.0, 1.0]]},
    {'bl_idname': 'CompositorNodeAlphaOver', 'name': 'Alpha Over.002', 'label': '', 'height': 100.0, 'width': 140.0, 'location': Vector((1553.754638671875, 983.2327880859375)), 'inputs': [1.0, [1.0, 1.0, 1.0, 1.0], [1.0, 1.0, 1.0, 1.0]]},
    {'bl_idname': 'CompositorNodeValToRGB', 'name': 'ColorRamp.005', 'label': '', 'height': 100.0, 'width': 240.0, 'location': Vector((-267.647216796875, 969.2713623046875)), 'inputs': [0.5], 'color_ramp': [[0.7636361122131348, 1.0, 1.0, 1.0, 1.0], [0.8227274417877197, 0.0, 0.0, 0.0, 1.0]]},
    {'bl_idname': 'CompositorNodeViewer', 'name': 'Viewer', 'label': '', 'height': 100.0, 'width': 140.0, 'location': Vector((2346.13525390625, 1300.00146484375)), 'inputs': [[0.0, 0.0, 0.0, 1.0], 1.0, 1.0]},
    {'bl_idname': 'CompositorNodeAntiAliasing', 'name': 'Anti-Aliasing', 'label': '', 'height': 100.0, 'width': 170.0, 'location': Vector((2048.990234375, 996.0372924804688)), 'inputs': [[1.0, 1.0, 1.0, 1.0]]},
    {'bl_idname': 'CompositorNodeOutputFile', 'name': 'File Output.003', 'label': 'inner_z_shade', 'height': 100.0, 'width': 140.0, 'location': Vector((2341.876708984375, 1141.97216796875)), 'inputs': [[0.0, 0.0, 0.0, 1.0]]},
    {'bl_idname': 'CompositorNodeOutputFile', 'name': 'File Output.004', 'label': 'inner_z_shade_frame', 'height': 100.0, 'width': 140.0, 'location': Vector((2912.87841796875, 1132.581298828125)), 'inputs': [[0.0, 0.0, 0.0, 1.0]]},
    {'bl_idname': 'CompositorNodeOutputFile', 'name': 'File Output.006', 'label': 'inner_texture_frame', 'height': 100.0, 'width': 140.0, 'location': Vector((2918.531494140625, 756.4423828125)), 'inputs': [[0.0, 0.0, 0.0, 1.0]]},
    {'bl_idname': 'CompositorNodeOutputFile', 'name': 'File Output.005', 'label': 'inner_texture', 'height': 100.0, 'width': 140.0, 'location': Vector((2336.30126953125, 745.205810546875)), 'inputs': [[0.0, 0.0, 0.0, 1.0]]},
    {'bl_idname': 'CompositorNodeOutputFile', 'name': 'File Output.007', 'label': 'outer_texture', 'height': 100.0, 'width': 140.0, 'location': Vector((2323.439697265625, -177.67156982421875)), 'inputs': [[0.0, 0.0, 0.0, 1.0]]},
    {'bl_idname': 'CompositorNodeOutputFile', 'name': 'File Output.008', 'label': 'outer_texture_frame', 'height': 100.0, 'width': 140.0, 'location': Vector((2879.9306640625, -171.0936279296875)), 'inputs': [[0.0, 0.0, 0.0, 1.0]]},
    {'bl_idname': 'CompositorNodeOutputFile', 'name': 'File Output.010', 'label': 'outer_z_shade_frame', 'height': 100.0, 'width': 140.0, 'location': Vector((2872.30126953125, -654.2391357421875)), 'inputs': [[0.0, 0.0, 0.0, 1.0]]},
    {'bl_idname': 'CompositorNodeOutputFile', 'name': 'File Output.009', 'label': 'outer_z_shade', 'height': 100.0, 'width': 140.0, 'location': Vector((2331.146728515625, -656.4317626953125)), 'inputs': [[0.0, 0.0, 0.0, 1.0]]},
    {'bl_idname': 'CompositorNodeOutputFile', 'name': 'File Output.002', 'label': 'raw_z_shade', 'height': 100.0, 'width': 140.0, 'location': Vector((-1112.75634765625, -113.32546997070312)), 'inputs': [[0.0, 0.0, 0.0, 1.0]]},
    {'bl_idname': 'CompositorNodeOutputFile', 'name': 'File Output.001', 'label': 'raw_mask', 'height': 100.0, 'width': 140.0, 'location': Vector((-1111.2076416015625, 391.925537109375)), 'inputs': [[0.0, 0.0, 0.0, 1.0]]},
    {'bl_idname': 'CompositorNodeOutputFile', 'name': 'File Output', 'label': 'raw_texture', 'height': 100.0, 'width': 140.0, 'location': Vector((-1105.351318359375, 702.8851318359375)), 'inputs': [[0.0, 0.0, 0.0, 1.0]]},
    {'bl_idname': 'CompositorNodeImage', 'name': 'Image.003', 'label': 'scale', 'height': 100.0, 'width': 141.29229736328125, 'location': Vector((-1357.7130126953125, 1029.682861328125)), 'inputs': []},
    {'bl_idname': 'CompositorNodeImage', 'name': 'Image', 'label': 'texture', 'height': 100.0, 'width': 138.39630126953125, 'location': Vector((-1348.66015625, 556.95947265625)), 'inputs': []},
    {'bl_idname': 'CompositorNodeImage', 'name': 'Image.001', 'label': 'mask', 'height': 100.0, 'width': 159.24365234375, 'location': Vector((-1358.7181396484375, 172.14007568359375)), 'inputs': []},
    {'bl_idname': 'CompositorNodeImage', 'name': 'Image.002', 'label': 'z_shade', 'height': 100.0, 'width': 140.0, 'location': Vector((-1343.8524169921875, -232.18417358398438)), 'inputs': []},
    {'node_linkes': [[1, 0, 0, 1], [3, 0, 0, 2], [0, 0, 2, 1], [2, 0, 4, 0], [2, 0, 4, 1], [4, 0, 21, 2], [56, 0, 6, 2], [6, 0, 21, 1], [4, 0, 40, 2], [5, 0, 40, 1], [10, 0, 7, 1], [9, 0, 7, 2], [7, 0, 8, 1], [8, 0, 11, 0], [8, 0, 11, 1], [11, 0, 15, 2], [4, 0, 15, 1], [15, 0, 18, 2], [13, 0, 18, 1], [12, 0, 13, 2], [56, 0, 16, 2], [16, 0, 14, 2], [15, 0, 20, 2], [14, 0, 20, 1], [55, 0, 28, 1], [28, 0, 29, 1], [40, 0, 39, 2], [39, 0, 43, 0], [19, 0, 34, 0], [17, 0, 36, 0], [18, 0, 17, 2], [20, 0, 19, 2], [21, 0, 22, 2], [22, 0, 23, 0], [57, 0, 24, 0], [24, 0, 14, 1], [24, 0, 13, 1], [29, 0, 30, 0], [30, 0, 22, 1], [30, 0, 39, 1], [30, 0, 19, 1], [30, 0, 17, 1], [58, 0, 25, 0], [25, 0, 5, 2], [25, 0, 12, 2], [57, 0, 27, 0], [27, 0, 10, 0], [27, 0, 9, 1], [57, 0, 26, 0], [26, 0, 16, 1], [26, 0, 12, 1], [32, 0, 38, 2], [43, 0, 38, 1], [34, 0, 35, 1], [36, 0, 37, 1], [23, 0, 33, 1], [32, 0, 33, 2], [32, 0, 35, 2], [32, 0, 37, 2], [56, 0, 54, 0], [41, 0, 5, 1], [41, 0, 6, 1], [57, 0, 41, 0], [31, 0, 3, 1], [31, 0, 1, 0], [57, 0, 31, 0], [57, 0, 53, 0], [58, 0, 52, 0], [38, 0, 45, 0], [23, 0, 47, 0], [33, 0, 46, 0], [34, 0, 48, 0], [35, 0, 49, 0], [36, 0, 51, 0], [37, 0, 50, 0], [43, 0, 44, 0], [43, 0, 42, 0]]},
    ]
    return node_set

def create_composite():
    scene = bpy.context.scene
    folder_path = scene.folder_path
    cs_obj_name = scene.cs_obj_name
    image_size = scene.image_size
    outline_tickness = scene.outline_tickness
    
    bpy.context.scene.use_nodes = True
    nodes = bpy.context.scene.node_tree.nodes
    
    #export UV template
    selectob = bpy.data.objects.get(cs_obj_name)
    if selectob == None:
        return False
    for ob in bpy.context.scene.objects:
        ob.select_set(False)
    selectob.select_set(True)
    bpy.context.view_layer.objects.active = selectob
    bpy.ops.object.mode_set(mode='EDIT', toggle=False)
    bpy.ops.uv.select_all(action='SELECT')
    bpy.ops.uv.export_layout(filepath=folder_path + r'/' +cs_obj_name + '_UVtemp.png', size=(image_size, image_size),opacity=0.0)
    bpy.ops.object.mode_set(mode='OBJECT', toggle=False)
    
    #delete pre-set nodes
    for n in nodes:
        nodes.remove(n)
        
    #create nodes
    node_tree = bpy.context.scene.node_tree
    node_set = get_node_set()
    make_nodes(node_tree, node_set)
    
    for n in nodes:
        if n.bl_idname == 'CompositorNodeImage':
            if n.label == 'UVtemp':
                n.image = bpy.data.images.load(folder_path + r'/' + cs_obj_name + '_UVtemp.png') 
            else:
                for image in bpy.data.images:
                    if image.name == n.label:
                        n.image = image
            n.update()
        elif n.bl_idname == 'CompositorNodeOutputFile':
            n.base_path = folder_path
            n.file_slots[0].path = cs_obj_name + '_' + n.label
            n.file_slots[0].save_as_render = False
        elif n.bl_idname == 'CompositorNodeDilateErode':
            n.distance = outline_tickness

def bake_common_setting():
    bpy.context.scene.render.engine = 'CYCLES'
    try:
        bpy.context.scene.cycles.device = 'GPU'
    except:
        bpy.context.scene.cycles.device = 'CPU'

def bake_texture(cs_obj,collection, image_name, bake_type):
    #set select and active the obj
    obj_list = get_objs_in_collection(collection)
    
    for ob in bpy.context.scene.objects:
        ob.select_set(False)
    for obj in obj_list:
        obj.select_set(True)
    cs_obj.select_set(True)
    bpy.context.view_layer.objects.active = cs_obj
    
    #link image to shader
    cs_material = get_material_by_name('CrossSection')
    source_node = get_material_node_by_label(cs_material, image_name)
    bsdf_node = get_material_node_by_label(cs_material, 'Diffuse BSDF')
    output_node = get_material_node_by_label(cs_material, 'Material Output')
    
    links = cs_material.node_tree.links
    for l in links:
        links.remove(l)
    links.new(source_node.outputs[0], bsdf_node.inputs[0])
    links.new(bsdf_node.outputs[0], output_node.inputs[0])
    
    source_node.select
    cs_material.node_tree.nodes.active = source_node
    
    #BAKE
    if bake_type == 'color':
        bpy.ops.object.bake(type='DIFFUSE', pass_filter={'COLOR'},margin = 0,use_selected_to_active = True,target='IMAGE_TEXTURES')
    elif bake_type == 'z_shade':
        bpy.ops.object.bake(type='NORMAL', normal_r='POS_Z', normal_g='POS_Z', normal_b='POS_Z' ,margin = 0,use_selected_to_active = True,target='IMAGE_TEXTURES')

def bake_process(cs_obj):
    bake_common_setting()
    bake_texture(cs_obj, 'Texture Bake Models','Texture Image','color')
    bake_texture(cs_obj, 'Mask Bake Models','Mask Image','color')
    bake_texture(cs_obj, 'Mask Bake Models','Z Shade Image','z_shade')
    bake_texture(bpy.data.objects['scale_plane'], 'Scale','Scale Image','color')

################################################################################
#OS file control functions
################################################################################
def unzip_files(folder_path):
    if not os.path.exists(folder_path):
        os.makedirs(folder_path)
    
    zip_files = [f for f in os.listdir(folder_path) if f.endswith('.zip')]
    count = 1
    
    for zip_file in zip_files:
        zip_file_path = os.path.join(folder_path, zip_file)
        with zipfile.ZipFile(zip_file_path, 'r') as zip_ref:
            extract_folder = os.path.join(folder_path, os.path.splitext(zip_file)[0])
            zip_ref.extractall(extract_folder)
            
            # 解凍後のファイルに対して元のZIPファイル名を付ける
            for file_name in os.listdir(extract_folder):
                src_file_path = os.path.join(extract_folder, file_name)
                if os.path.isfile(src_file_path):
                    file_extension = os.path.splitext(file_name)[1]
                    new_file_name = f"{os.path.splitext(zip_file)[0]}{file_extension}"
                    dst_file_path = os.path.join(folder_path, new_file_name)
                    os.rename(src_file_path, dst_file_path)
            
            # 解凍後のフォルダを削除
            shutil.rmtree(extract_folder)
            
        # フォルダ内のglbファイル名を順に調べて連番を付ける
        for file_name in os.listdir(folder_path):
            if file_name.endswith('.glb'):
                base_name, file_extension = os.path.splitext(file_name)
                match = re.search(r" \d+$", base_name)
                if not match:
                    new_file_name = f"{base_name} 01{file_extension}"
                    os.rename(os.path.join(folder_path, file_name), os.path.join(folder_path, new_file_name))
                else:
                    # 連番が一桁の場合は2桁に変更
                    num = int(match.group().strip())
                    if num < 10:
                        new_file_name = f"{base_name[:-len(match.group())]} {num:02}{file_extension}"
                        os.rename(os.path.join(folder_path, file_name), os.path.join(folder_path, new_file_name))
                        
        # 元のzipファイルを削除
        os.remove(zip_file_path)


################################################################################
#Blender UI
################################################################################

bl_info = {
    "name": "Cave Mapper",
    "author": "Cave Mapper",
    "version": (2, 3, 7),
    "blender": (4, 0, 2),
    "location": "3D View > Sidebar",
    "description": "Help to handle 3D scan datas of cave",
    "warning": "",
    "support": "COMMUNITY",
    "doc_url": "",
    "tracker_url": "",
    "category": "User Interface"
}

#################################################################################
#Functions kicked when buttoms are pushed
class import_models(bpy.types.Operator):
    bl_idname = "object.import_models"
    bl_label = "NOP"
    bl_description = "Import models"
    bl_options = {'REGISTER', 'UNDO'}
    
    def execute(self, context):
        bpy.context.window.cursor_set("WAIT")
        
        #make 'Import Models' collection
        import_obj_collection = make_object_collection('Import Models')
        
        #import glb files
        scene = bpy.context.scene
        
        ext_str = "glb"
        folderPath = bpy.path.abspath(scene.folder_path)

        filesPathList = glob.glob(folderPath + "\\*." + ext_str)
        i = 0
        for filePath in filesPathList:
            filename = os.path.split(filePath)[1]
            print(filename)
            print(filePath)
            
            bpy.ops.import_scene.gltf(filepath=filePath, merge_vertices=True, import_pack_images = False)    
            raw_obj = bpy.context.selected_objects[0]
            bpy.context.view_layer.objects.active = raw_obj
            
            print(raw_obj.name)
            print(raw_obj.type)
                    
            obj_name = filename.replace("." + ext_str, "")
            if obj_name != raw_obj.name:
                raw_obj.name = obj_name
                raw_obj = merge_empty_objects(raw_obj)
                raw_obj.data.name = obj_name          
                                
            bpy.ops.object.shade_smooth()
            bpy.context.object.color = setObjColor(i)
                        
            bpy.context.scene.collection.objects.unlink(raw_obj)
            import_obj_collection.objects.link(raw_obj)

            i += 1
        
        bpy.context.window.cursor_set("DEFAULT")
        return {'FINISHED'}

class Unzip_glb(bpy.types.Operator):
    bl_idname = "object.unzip_glb"
    bl_label = "NOP"
    bl_description = "Unzip glb files"
    bl_options = {'REGISTER', 'UNDO'}
    
    def execute(self, context):
        bpy.context.window.cursor_set("WAIT")
        
        #import glb files
        scene = bpy.context.scene
        
        ext_str = "glb"
        folderPath = bpy.path.abspath(scene.folder_path)

        unzip_files(folderPath)
        
        bpy.context.window.cursor_set("DEFAULT")
        return {'FINISHED'}

class Run_GR(bpy.types.Operator):
    bl_idname = "object.run_gr"
    bl_label = "NOP"
    bl_description = "Rough aligment after rough aligment"
    bl_options = {'REGISTER', 'UNDO'}
    
    @classmethod
    def poll(cls, context):
        # オブジェクトが選択されているときのみメニューを表示させる
        if len(bpy.context.selected_objects) == 2:
            if bpy.context.active_object != None:
                return True
        return False

    def execute(self, context):
        bpy.context.window.cursor_set("WAIT")
        
        source_name = bpy.context.active_object.name
        bpy.context.active_object.data.name = bpy.context.active_object.name
        for o in bpy.context.selected_objects:
            if o != bpy.context.active_object:
                target_name = o.name
                o.data.name = o.name
                
        scene = bpy.context.scene
        voxel_size = scene.Aligment_voxel_size
        GR_coef = scene.GR_coef
        Aligment_FPFH_coef = scene.Aligment_FPFH_coef
        Process_GR(source_name,target_name,voxel_size,GR_coef, Aligment_FPFH_coef)
        
        bpy.context.window.cursor_set("DEFAULT")
        return {'FINISHED'}

class Run_ICP(bpy.types.Operator):
    bl_idname = "object.run_icp"
    bl_label = "NOP"
    bl_description = "Refine aligment after rough aligment"
    bl_options = {'REGISTER', 'UNDO'}
    
    @classmethod
    def poll(cls, context):
        # オブジェクトが選択されているときのみメニューを表示させる
        if len(bpy.context.selected_objects) == 2:
            if bpy.context.active_object != None:
                return True
        return False

    def execute(self, context):
        bpy.context.window.cursor_set("WAIT")
        
        source_name = bpy.context.active_object.name
        bpy.context.active_object.data.name = bpy.context.active_object.name
        for o in bpy.context.selected_objects:
            if o != bpy.context.active_object:
                target_name = o.name
                o.data.name = o.name
                
        scene = bpy.context.scene
        voxel_size = scene.Aligment_voxel_size
        ICP_coef = scene.ICP_coef
        Aligment_FPFH_coef = scene.Aligment_FPFH_coef
        Process_ICP(source_name,target_name,voxel_size,ICP_coef, Aligment_FPFH_coef)
        
        bpy.context.window.cursor_set("DEFAULT")
        return {'FINISHED'}

class Run_Hrz_fix(bpy.types.Operator):
    bl_idname = "object.run_hrz_fix"
    bl_label = "NOP"
    bl_description = "Rotete X and Y to zero for fix horizontal after aligment"
    bl_options = {'REGISTER', 'UNDO'}
    
    @classmethod
    def poll(cls, context):
        # オブジェクトが選択されているときのみメニューを表示させる
        if len(bpy.context.selected_objects) == 2:
            if bpy.context.active_object != None:
                return True
        return False

    def execute(self, context):
        bpy.context.window.cursor_set("WAIT")
        
        source_name = bpy.context.active_object.name
        bpy.context.active_object.data.name = bpy.context.active_object.name
        for o in bpy.context.selected_objects:
            if o != bpy.context.active_object:
                target_name = o.name
                o.data.name = o.name
                
        scene = bpy.context.scene
        
        scene = bpy.context.scene
        voxel_size = scene.Aligment_voxel_size
        ICP_coef = scene.ICP_coef
        Aligment_FPFH_coef = scene.Aligment_FPFH_coef
        overlap_center_ndarray = Process_Hrz_fix(source_name,target_name,voxel_size,ICP_coef, Aligment_FPFH_coef)
        
        if overlap_center_ndarray is not None:
            overlap_center_in_blender = overlap_center_ndarray.tolist()
            fix_horizontal_by_overlap_centor(source_name,overlap_center_in_blender)
        bpy.ops.object.mode_set(mode='OBJECT', toggle=False)
        
        bpy.context.window.cursor_set("DEFAULT")
        return {'FINISHED'}

class Run_Set_Target(bpy.types.Operator):
    bl_idname = "object.run_set_target"
    bl_label = "NOP"
    bl_description = "Regist aligned position"
    bl_options = {'REGISTER', 'UNDO'}
    target_obj = None
    
    @classmethod
    def poll(cls, context):
        if len(bpy.context.selected_objects) == 1:
            if bpy.context.active_object != None:
                return True
        return False

    def execute(self, context):    
        scene = bpy.context.scene
        
        scene.target_obj_name = bpy.context.active_object.name
                
        bpy.context.window.cursor_set("DEFAULT")
        return {'FINISHED'}
    
class Run_Partial_ICP(bpy.types.Operator):
    bl_idname = "object.run_partial_icp"
    bl_label = "NOP"
    bl_description = "relux coarse mesh"
    bl_options = {'REGISTER', 'UNDO'}
    
    @classmethod
    def poll(cls, context):
        scene = bpy.context.scene
        if len(bpy.context.selected_objects) == 1:
            if bpy.context.mode == 'EDIT_MESH':
                if bpy.context.tool_settings.mesh_select_mode[0]:
                    return True
                    '''
                    #Not work
                    if scene.target_obj_name != bpy.context.selected_objects[0].name:
                        for obj in bpy.context.scene.objects:                        
                            if obj.name == scene.target_obj:
                    '''
                        
        return False

    def execute(self, context):
        bpy.context.window.cursor_set("WAIT")
        
        scene = bpy.context.scene

        source_name = bpy.context.selected_objects[0].name
        target_name = scene.target_obj_name
        voxel_size = scene.Aligment_voxel_size
        ICP_coef = scene.ICP_coef
        Aligment_FPFH_coef = scene.Aligment_FPFH_coef
        
        Process_Partial_ICP(source_name,target_name,voxel_size,ICP_coef, Aligment_FPFH_coef)
        
        #print(scene.target_obj_name)
        #print(bpy.context.selected_objects[0].name)
                
        bpy.context.window.cursor_set("DEFAULT")
        return {'FINISHED'}

class Run_Apply_propotional_edit(bpy.types.Operator):
    bl_idname = "object.run_apply_prpotional_edit"
    bl_label = "NOP"
    bl_description = "relux coarse mesh"
    bl_options = {'REGISTER', 'UNDO'}
    
    @classmethod
    def poll(cls, context):
        return canAdjustEffect

    def execute(self, context):
        bpy.context.window.cursor_set("WAIT")

        Process_apply_propotional_edit()
        
        bpy.context.window.cursor_set("DEFAULT")
        return {'FINISHED'}

class Run_Decimate(bpy.types.Operator):
    bl_idname = "object.run_decimate"
    bl_label = "NOP"
    bl_description = "down sizing meshes"
    bl_options = {'REGISTER', 'UNDO'}
    
    @classmethod
    def poll(cls, context):
        if len(bpy.context.selected_objects) > 0:
                return True
        return False

    def execute(self, context):
        bpy.context.window.cursor_set("WAIT")
        scene = bpy.context.scene
        decimate_ref_length = scene.decimate_ref_length
        
        Process_decimate(bpy.context.selected_objects, decimate_ref_length)
        
        bpy.context.window.cursor_set("DEFAULT")
        return {'FINISHED'}

class Run_Bake_Texture(bpy.types.Operator):
    bl_idname = "object.run_bake_texture"
    bl_label = "NOP"
    bl_description = "down sizing meshes"
    bl_options = {'REGISTER', 'UNDO'}
    
    @classmethod
    def poll(cls, context):
        if len(bpy.context.selected_objects) > 0:
                return True
        return False

    def execute(self, context):
        bpy.context.window.cursor_set("WAIT")
        scene = bpy.context.scene
        texture_size = scene.union_texture_size
        
        bake_texture_to_union(texture_size)
        
        bpy.context.window.cursor_set("DEFAULT")
        return {'FINISHED'}

class Run_Texture_reduction(bpy.types.Operator):
    bl_idname = "object.run_texture_reduction"
    bl_label = "NOP"
    bl_description = "down sizing textures"
    bl_options = {'REGISTER', 'UNDO'}

    def execute(self, context):
        bpy.context.window.cursor_set("WAIT")
        scene = bpy.context.scene
        image_reduction_ratio = scene.image_reduction_ratio
        
        Process_texture_reduction(image_reduction_ratio)
        
        bpy.context.window.cursor_set("DEFAULT")
        return {'FINISHED'}

class Run_Remesh(bpy.types.Operator):
    bl_idname = "object.run_remesh"
    bl_label = "NOP"
    bl_description = "Union selected meshes"
    bl_options = {'REGISTER', 'UNDO'}
    
    @classmethod
    def poll(cls, context):
        if len(bpy.context.selected_objects) > 0:
                return True
        return False

    def execute(self, context):
        bpy.context.window.cursor_set("WAIT")
        
        scene = bpy.context.scene
        voxel_size = scene.Remesh_voxel_size
        
        mesh_name = "UnionMesh"
        o3d_triangle_mesh = Process_remesh(bpy.context.selected_objects,voxel_size)
        unioned_obj = make_blenderBmesh_from_open3dTriangleMesh(o3d_triangle_mesh,mesh_name)
        
        delete_small_poly_islands(unioned_obj)
        
        apply_smooth(unioned_obj)
                
        bpy.context.window.cursor_set("DEFAULT")
        return {'FINISHED'}


class Run_cs_model_pre_prs(bpy.types.Operator):
    bl_idname = "object.run_cs_model_pre_prs"
    bl_label = "NOP"
    bl_description = "Pre-process befor bake for cross section model"
    bl_options = {'REGISTER', 'UNDO'}
    
    @classmethod
    def poll(cls, context):
        if len(bpy.context.selected_objects) == 1:
            return True
        return False
        '''
        ################################################################
        #write some condition to prevent select wrong obj
        ################################################################
        '''
    
    def execute(self, context):
        bpy.context.window.cursor_set("WAIT")
        scene = bpy.context.scene
        image_size = scene.image_size   
        
        cs_obj = bpy.context.selected_objects[0]
        cs_model_pre_prs(cs_obj)
        
        scene.cs_obj_name = cs_obj.name        
        bpy.context.window.cursor_set("DEFAULT")
        return {'FINISHED'}


class Run_cs_bake_and_composite(bpy.types.Operator):
    bl_idname = "object.run_cbake_and_composite"
    bl_label = "NOP"
    bl_description = "Generate Cross Section Images"
    bl_options = {'REGISTER', 'UNDO'}
    
    def execute(self, context):
        bpy.context.window.cursor_set("WAIT")
        scene = bpy.context.scene
        cs_obj_name = scene.cs_obj_name
        image_size = scene.image_size
        
        #dupulicate cave objcts as Mask Bake Model and Texture Bake Model
        create_material_inner_outer('inner', 1.0)
        create_material_inner_outer('outer', 0.5)
        
        obj_list = get_objs_in_collection('Import Models')
        obj_name_list = []
        if len(obj_list) > 0:
            for obj in obj_list:
                obj_name_list.append(obj.name)
            for obj_name in obj_name_list:
                for obj in bpy.context.scene.objects:
                    if obj.name == obj_name:
                        cave_model_pre_prs(obj)
        
        #resize image
        resize_images(image_size)
        
        for ob in bpy.context.scene.objects:
            if ob.name == cs_obj_name :
                bake_process(ob)
        create_composite()
        bpy.ops.render.render()
        
        #delete dupulicated objects
        delete_collection('Texture Bake Models')
        delete_collection('Mask Bake Models')
        
        
        bpy.context.window.cursor_set("DEFAULT")
        return {'FINISHED'}

#################################################################################
#UI setting
class IMPORT_PT_CustomPanel(bpy.types.Panel):
    bl_label = "Import Models"         # パネルのヘッダに表示される文字列
    bl_space_type = 'VIEW_3D'           # パネルを登録するスペース
    bl_region_type = 'UI'               # パネルを登録するリージョン
    bl_category = "Cave Mapper"         # パネルを登録するタブ名
    #bl_context = "objectmode"           # パネルを表示するコンテキスト
    
    # ヘッダーのカスタマイズ
    def draw_header(self, context):
        layout = self.layout

    # メニューの描画処理
    def draw(self, context):
        layout = self.layout
        scene = context.scene     
        
        layout.use_property_split = True
        layout.use_property_decorate = False  # No animation.
        
        layout.label(text="Folder Path:")
        layout.prop(scene,"folder_path",text = "")
        layout.operator(Unzip_glb.bl_idname, text="Ungip glb files", icon = 'SORTTIME')
        layout.operator(import_models.bl_idname, text="Import Models", icon = 'SORTTIME')

class ALIGHMENT_PT_CustomPanel(bpy.types.Panel):
    bl_label = "Aligment"         # パネルのヘッダに表示される文字列
    bl_space_type = 'VIEW_3D'           # パネルを登録するスペース
    bl_region_type = 'UI'               # パネルを登録するリージョン
    bl_category = "Cave Mapper"         # パネルを登録するタブ名
    #bl_context = "objectmode"           # パネルを表示するコンテキスト
    
    # ヘッダーのカスタマイズ
    def draw_header(self, context):
        layout = self.layout

    # メニューの描画処理
    def draw(self, context):
        layout = self.layout
        scene = context.scene

class CONFIG_PT_CustomPanel(bpy.types.Panel):
    bl_label = "Parameter Setting"         # パネルのヘッダに表示される文字列
    bl_space_type = 'VIEW_3D'           # パネルを登録するスペース
    bl_region_type = 'UI'               # パネルを登録するリージョン
    bl_category = "Cave Mapper"         # パネルを登録するタブ名
    #bl_context = "objectmode"           # パネルを表示するコンテキスト
    bl_parent_id = "ALIGHMENT_PT_CustomPanel"
    bl_options = {'DEFAULT_CLOSED'}

    # ヘッダーのカスタマイズ
    def draw_header(self, context):
        layout = self.layout

    # メニューの描画処理
    def draw(self, context):
        layout = self.layout
        scene = context.scene
        
        layout.use_property_split = True
        layout.use_property_decorate = False  # No animation.

        #Parameters
        layout.prop(scene, "Aligment_voxel_size", text="Voxcel Size")
        layout.prop(scene, "Aligment_FPFH_coef", text="FPFH Coef")
        layout.prop(scene, "GR_coef", text="Rough Coef")
        layout.prop(scene, "ICP_coef", text="Fine Coef")
        

class PROCESS_PT_CustomPanel(bpy.types.Panel):
    bl_label = "Run Process"         # パネルのヘッダに表示される文字列
    bl_space_type = 'VIEW_3D'           # パネルを登録するスペース
    bl_region_type = 'UI'               # パネルを登録するリージョン
    bl_category = "Cave Mapper"         # パネルを登録するタブ名
    #bl_context = "objectmode"           # パネルを表示するコンテキスト
    bl_parent_id = "ALIGHMENT_PT_CustomPanel"

    # ヘッダーのカスタマイズ
    def draw_header(self, context):
        layout = self.layout

    # メニューの描画処理
    def draw(self, context):
        layout = self.layout
        scene = context.scene
        
        layout.use_property_split = True
        layout.use_property_decorate = False  # No animation.

        #Run Buttom
        layout.operator(Run_GR.bl_idname, text="Rough Aligment" , icon = 'SORTTIME')
        layout.operator(Run_ICP.bl_idname, text="Fine Aligment", icon = 'SORTTIME')
        layout.operator(Run_Hrz_fix.bl_idname, text="Fix Horizontal", icon = 'SORTTIME')

class PARTIAL_PROCESS_PT_CustomPanel(bpy.types.Panel):
    bl_label = "Partial Alignment"         # パネルのヘッダに表示される文字列
    bl_space_type = 'VIEW_3D'           # パネルを登録するスペース
    bl_region_type = 'UI'               # パネルを登録するリージョン
    bl_category = "Cave Mapper"         # パネルを登録するタブ名
    #bl_context = "objectmode"           # パネルを表示するコンテキスト
    bl_parent_id = "ALIGHMENT_PT_CustomPanel"

    # ヘッダーのカスタマイズ
    def draw_header(self, context):
        layout = self.layout

    # メニューの描画処理
    def draw(self, context):
        layout = self.layout
        scene = context.scene
        
        layout.use_property_split = True
        layout.use_property_decorate = False  # No animation.

        #Run Buttom
        layout.operator(Run_Set_Target.bl_idname, text="Set Target")
        layout.label(text="Target: " + scene.target_obj_name)
        layout.operator(Run_Partial_ICP.bl_idname, text="Partial Aligment", icon = 'SORTTIME')
        layout.label(text="Propotional Edit")
        layout.prop(scene,"effective_length", text="Eff Length")
        layout.operator(Run_Apply_propotional_edit.bl_idname, text="Apply Deform")

class DECIMATE_PT_CustomPanel(bpy.types.Panel):
    bl_label = "Down sizing"         # パネルのヘッダに表示される文字列
    bl_space_type = 'VIEW_3D'           # パネルを登録するスペース
    bl_region_type = 'UI'               # パネルを登録するリージョン
    bl_category = "Cave Mapper"         # パネルを登録するタブ名
    #bl_context = "objectmode"           # パネルを表示するコンテキスト
    bl_options = {'DEFAULT_CLOSED'}
    
    # ヘッダーのカスタマイズ
    def draw_header(self, context):
        layout = self.layout

    # メニューの描画処理
    def draw(self, context):
        layout = self.layout
        scene = context.scene
        
        layout.use_property_split = True
        layout.use_property_decorate = False  # No animation.
        
        layout.prop(scene, "decimate_ref_length", text="Ref Length")
        layout.operator(Run_Decimate.bl_idname, text="Mesh down sizing", icon = 'SORTTIME')
        layout.separator()
        layout.prop(scene, "image_reduction_ratio", text="Texture reduction")
        layout.operator(Run_Texture_reduction.bl_idname, text="Texture down sizing", icon = 'SORTTIME')

class REMESH_PT_CustomPanel(bpy.types.Panel):
    bl_label = "Mesh Union"         # パネルのヘッダに表示される文字列
    bl_space_type = 'VIEW_3D'           # パネルを登録するスペース
    bl_region_type = 'UI'               # パネルを登録するリージョン
    bl_category = "Cave Mapper"         # パネルを登録するタブ名
    #bl_context = "objectmode"           # パネルを表示するコンテキスト
    
    # ヘッダーのカスタマイズ
    def draw_header(self, context):
        layout = self.layout

    # メニューの描画処理
    def draw(self, context):
        layout = self.layout
        scene = context.scene
        
        layout.use_property_split = True
        layout.use_property_decorate = False  # No animation.
        
        layout.prop(scene, "Remesh_voxel_size", text="Voxcel Size")
        layout.operator(Run_Remesh.bl_idname, text="Union", icon = 'SORTTIME')
        layout.prop(scene, "union_texture_size", text="Texture Size")
        layout.prop(scene, "island_margin", text="Island Margin")
        layout.prop(scene, "union_bake_extrusion", text="Extrucion")
        layout.prop(scene, "union_bake_max_ray_distance", text="Max Ray Distance")
        layout.prop(scene, "bake_base_color", text="Base color")
        layout.operator(Run_Bake_Texture.bl_idname, text="Bake Texture", icon = 'SORTTIME')

class CROSS_SECTION_PT_CustomPanel(bpy.types.Panel):
    bl_label = "Cross Section"         # パネルのヘッダに表示される文字列
    bl_space_type = 'VIEW_3D'           # パネルを登録するスペース
    bl_region_type = 'UI'               # パネルを登録するリージョン
    bl_category = "Cave Mapper"         # パネルを登録するタブ名
    #bl_context = "objectmode"           # パネルを表示するコンテキスト
    
    # ヘッダーのカスタマイズ
    def draw_header(self, context):
        layout = self.layout

    # メニューの描画処理
    def draw(self, context):
        layout = self.layout
        scene = context.scene
        
        layout.use_property_split = True
        layout.use_property_decorate = False  # No animation.
        
        layout.prop(scene, "image_size", text="Image size")
        layout.prop(scene, "scale_length", text="Scale length")
        layout.operator(Run_cs_model_pre_prs.bl_idname, text="Cross Section Pre-Process")
        layout.label(text="Cross Section Object:")
        row = layout.row(align=True)
        row.enabled = False
        row.prop(scene, "cs_obj_name",text="")
        layout.prop(scene, "outline_tickness", text="Line tickness")
        layout.operator(Run_cs_bake_and_composite.bl_idname, text="Generate Images", icon = 'SORTTIME')

# プロパティの初期化
def init_props():
    scene = bpy.types.Scene
    scene.folder_path = StringProperty(
        name="folder path",
        description="folder path of glb files",
        default="",
        subtype = 'FILE_PATH'
    ) 
    scene.Aligment_voxel_size = FloatProperty(
        name="Aligment_voxel_size",
        description="Estimated length between points of the down sized point cloud model for process",
        default=0.05,
        min=0.0,
        subtype = 'DISTANCE'
    ) 
    scene.GR_coef = FloatProperty(
        name="GR_coef",
        description="The value obtained by multiplying the voxel size by this coefficient is the distance threshold for aligment processing.",
        default=5,
        min=0.0,
    )
    scene.Aligment_FPFH_coef = FloatProperty(
        name="Aligment_FPFH_coef",
        description="The value obtained by multiplying the voxel size by this coefficient is the radius for FPFH of downsampled point cloud model..",
        default=10,
        min=0.0,
    ) 
    scene.ICP_coef = FloatProperty(
        name="ICP_coef",
        description="The value obtained by multiplying the voxel size by this coefficient is the distance threshold in ICP processing.",
        default=2,
        min=0.0,
    )
    scene.target_obj_name = StringProperty(
        name="target_obj_name",
        description="Target object name for partial alignment",
        default = "None",
    )
    scene.effective_length = FloatProperty(
        name="effective_length",
        description="effective_length for partial alignment",
        default=1.0,
        min=0.0,
        subtype = 'DISTANCE'
    ) 
    scene.decimate_ref_length = FloatProperty(
        name="decimate_ref_length",
        description="Estimate edges length of down sized mesh",
        default=0.2,
        min=0.0,
        subtype = 'DISTANCE'
    ) 
    scene.image_reduction_ratio = IntProperty(
        name="image_reduction_ratio",
        description="Image size reduction retio",
        default=4,
        min=2,
        max=64
    )
    scene.Remesh_voxel_size = FloatProperty(
        name="Remesh_voxel_size",
        description="Estimated length between points of the down sized model",
        default=0.2,
        min=0.0,
        subtype = 'DISTANCE'
    )
    scene.image_size = IntProperty(
        name="Image_size",
        description="output image size",
        default=2048,
        min=1,
        max=8192,
        subtype = 'PIXEL'
    )
    scene.scale_length = FloatProperty(
        name="Scale_length",
        description="length of scale on cross section image",
        default=10,
        min=0.1,
        subtype = 'DISTANCE'
    ) 
    scene.cs_obj_name = StringProperty(
        name="cs_obj_name",
        description="Cross Section Object",
        default=""
    ) 
    scene.outline_tickness = IntProperty(
        name="outline_tickness",
        description="Outline tickness of cross section images",
        default=2,
        min=1,
        max=10,
        subtype = 'PIXEL'
    ) 
    scene.union_texture_size = IntProperty(
        name="union_texture_size",
        description="pixcel size of unioned model texture width and height",
        default=4096,
        min=1,
        max=8192,
        subtype = 'PIXEL'
    ) 
    scene.bake_base_color = FloatVectorProperty(
        name="bake_base_color",
        description="Base color for no baked areas",
        subtype='COLOR', size=4, # RGBAの場合は4、RGBの場合は3
        min=0.0,
        max=1.0,
        default=(0.0, 1.0, 0.0, 1.0) 
    )
    scene.island_margin = FloatProperty(
        name="island_margin",
        description="Island margin for UV unwrap of unioned model",
        default=0.05,
        min=0,
    )
    scene.union_bake_extrusion = FloatProperty(
        name="union_bake_extrusion",
        description="Extrusion for bake texture of the unioned model",
        default=0.2,
        min=0,
        subtype = 'DISTANCE'
    )
    scene.union_bake_max_ray_distance = FloatProperty(
        name="union_bake_max_ray_distance",
        description="Max Ray Distance for bake texture of the unioned model",
        default=0.5,
        min=0,
        subtype = 'DISTANCE'
    )


# プロパティを削除
def clear_props():
    scene = bpy.types.Scene
    del scene.folder_path
    del scene.Aligment_voxel_size
    del scene.GR_coef
    del scene.Aligment_FPFH_coef
    del scene.ICP_coef
    del scene.target_obj_name
    del scene.effective_length
    del scene.decimate_ref_length
    del scene.image_reduction_ratio
    del scene.Remesh_voxel_size
    del scene.image_size
    del scene.scale_length
    del scene.cs_obj_name
    del scene.outline_tickness
    del scene.union_texture_size
    del scene.bake_base_color
    del scene.union_bake_extrusion
    del scene.union_bake_max_ray_distance
    del scene.island_margin

classes = [
    import_models,
    Unzip_glb,
    Run_GR,
    Run_ICP,
    Run_Hrz_fix,
    Run_Set_Target,
    Run_Partial_ICP,
    Run_Apply_propotional_edit,
    Run_Decimate,
    Run_Texture_reduction,
    Run_Remesh,
    Run_Bake_Texture,
    Run_cs_model_pre_prs,
    Run_cs_bake_and_composite,
    IMPORT_PT_CustomPanel,
    ALIGHMENT_PT_CustomPanel,
    CONFIG_PT_CustomPanel,
    PROCESS_PT_CustomPanel,
    PARTIAL_PROCESS_PT_CustomPanel,
    DECIMATE_PT_CustomPanel,
    REMESH_PT_CustomPanel,
    CROSS_SECTION_PT_CustomPanel
]



def register():
    for c in classes:
        bpy.utils.register_class(c)
    init_props()
    print("add on is registerd")


def unregister():
    clear_props()
    for c in classes:
        bpy.utils.unregister_class(c)
    print("add on is unregisterd")


if __name__ == "__main__":
    register()

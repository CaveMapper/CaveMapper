import bpy
import os
from bpy.props import StringProperty
import xml.etree.ElementTree as ET
import math
import re
import time
import numpy as np
import pkg_resources
import subprocess
import sys
try:
    from staticmap import StaticMap
except ImportError:
    pass

# 地球の楕円体定数（WGS84）
a = 6378137.0  # 長半径（赤道半径）
f = 1 / 298.257223563  # 扁平率

# テクスチャ切替フラグを保持するためのグローバル変数
toggle_flag = 1    
    
def get_northwest_coordinates(xml_file_paths):
    northwest_coords = None

    for file_path in xml_file_paths:
        # XMLファイルのパース
        tree = ET.parse(file_path)
        root = tree.getroot()

        # 名前空間を取得
        namespaces = {'gml': 'http://www.opengis.net/gml/3.2'}

        # lowerCornerの座標を取得
        lower_corner = root.find('.//gml:Envelope/gml:lowerCorner', namespaces)
        if lower_corner is not None:
            # lowerCornerの値を分割してfloat型に変換
            coords = list(map(float, lower_corner.text.split()))
            
            # 現在の座標と比較して北西端を更新
            if northwest_coords is None or (coords[1] < northwest_coords[1] or 
                                            (coords[1] == northwest_coords[1] and coords[0] > northwest_coords[0])):
                northwest_coords = coords

    return northwest_coords


def calculate_hubeny(lat1, lon1, lat2, lon2):
    """
    Hubenyの公式を使用して2地点間の距離を計算
    """
    lat1, lon1, lat2, lon2 = map(math.radians, [lat1, lon1, lat2, lon2])
    lat_avg = (lat1 + lat2) / 2
    delta_lat = lat2 - lat1
    delta_lon = lon2 - lon1
    M = a * (1 - f) / math.pow((1 - f * math.sin(lat_avg)**2), 3 / 2)
    N = a / math.sqrt(1 - f * math.sin(lat_avg)**2)
    distance = math.sqrt((M * delta_lat)**2 + (N * math.cos(lat_avg) * delta_lon)**2)
    return distance

def extract_code_from_filename(filename):
    """
    指定されたファイル名からコード部分（5438-06-95）を抽出する関数
    """
    # ファイル名を "-" で分割
    parts = filename.split('-')
    
    # 必要な部分を結合して返す
    extracted_code = f"{parts[2]}-{parts[3]}-{parts[4]}"
    return extracted_code

def calculate_pixel_dimensions(xml_file_path, zoom_level):
    """
    StaticMapを使ってwebから地理院テクスチャを取得する
    """
    
    # XMLファイルの解析
    tree = ET.parse(xml_file_path)
    root = tree.getroot()

    # 名前空間の設定
    namespaces = {'gml': 'http://www.opengis.net/gml/3.2'}

    # upper cornerとlower cornerの座標を取得
    lower_corner = root.find('.//gml:lowerCorner', namespaces).text.split()
    upper_corner = root.find('.//gml:upperCorner', namespaces).text.split()

    min_lat = float(lower_corner[0])  # 南緯
    min_lon = float(lower_corner[1])  # 西経
    max_lat = float(upper_corner[0])  # 北緯
    max_lon = float(upper_corner[1])  # 東経
    
    # 緯度経度の差分を取得
    lat_diff = abs(max_lat - min_lat)
    lon_diff = abs(max_lon - min_lon)

    # タイル全体のメートル単位のサイズをズームレベルで計算
    #NS_ZL0 = calculate_hubeny(90.0, max_lon, 0.0, max_lon) * 2
    EW_ZL0 = calculate_hubeny(max_lat, 180, max_lat, 0.0) * 2
    
    #NS_rez = NS_ZL0 / (2 ** zoom_level) #m/tile
    EW_rez = EW_ZL0 / (2 ** zoom_level) #m/tile
    
    NS_tileNum = calculate_hubeny(max_lat, max_lon, min_lat, max_lon) / EW_rez
    EW_tileNum = calculate_hubeny(max_lat, max_lon, max_lat, min_lon) / EW_rez
    
    NS_px = int(NS_tileNum * 256)
    EW_px = int(EW_tileNum * 256)
    
    #set save file name
    file_name = os.path.basename(xml_file_path)
    meshname = extract_code_from_filename(file_name)
    directory_path = os.path.dirname(xml_file_path)
    
    #get standard map
    url_list = ('https://cyberjapandata.gsi.go.jp/xyz/std/{z}/{x}/{y}.png','https://cyberjapandata.gsi.go.jp/xyz/seamlessphoto/{z}/{x}/{y}.jpg')
    
    for url_str in url_list:
        ident_str = re.search(r'https://cyberjapandata.gsi.go.jp/xyz/([^/]+)/\{z\}/\{x\}/\{y\}', url_str)
        map = StaticMap(EW_px, NS_px,url_template= url_str)
        image = map.render(zoom=zoom_level, center=[(max_lon+min_lon)*0.5, (max_lat+min_lat)*0.5])  
        
        savename = directory_path + '/' + meshname + '_' + ident_str.group(1) + '.png'
        image.save(savename)

        #DLした画像をblenderにロード
        bpy.data.images.load(savename)
    

def process_xml_and_generate_mesh(xml_file_path, northwest_coords, replace_nodata=False):
    """
    指定されたXMLファイルを解析し、Blenderで地形メッシュを生成

    Parameters:
    - xml_file_path: XMLファイルのパス
    - northwest_coords: 北西座標
    - replace_nodata: Trueの場合、データなし(-9999)を0に置き換える
    """

    origin_lat = northwest_coords[0]
    origin_lon = northwest_coords[1]
    
    # XMLファイルの解析
    tree = ET.parse(xml_file_path)
    root = tree.getroot()

    # 名前空間の設定
    namespaces = {'gml': 'http://www.opengis.net/gml/3.2'}

    # upper cornerとlower cornerの座標を取得
    lower_corner = root.find('.//gml:lowerCorner', namespaces).text.split()
    upper_corner = root.find('.//gml:upperCorner', namespaces).text.split()

    min_lat = float(lower_corner[0])  # 南緯
    min_lon = float(lower_corner[1])  # 西経
    max_lat = float(upper_corner[0])  # 北緯
    max_lon = float(upper_corner[1])  # 東経

    # 距離を計算
    north_south_distance = calculate_hubeny(min_lat, min_lon, max_lat, min_lon)
    east_west_distance = calculate_hubeny(min_lat, min_lon, min_lat, max_lon)

    # gml:lowとgml:highから全体のグリッドサイズを取得
    low = root.find('.//gml:low', namespaces).text.split()
    high = root.find('.//gml:high', namespaces).text.split()

    # 全体のグリッドサイズ（地理的範囲全体）
    grid_size_x = int(high[0]) - int(low[0]) + 1
    grid_size_y = int(high[1]) - int(low[1]) + 1

    # gml:startPointを取得（実際のデータが始まる位置）
    start_point = root.find('.//gml:startPoint', namespaces).text.split()
    start_x = int(start_point[0])
    start_y = int(start_point[1])

    # 実際に生成するメッシュのサイズ（startPointからhighまで）
    mesh_size_x = int(high[0]) - start_x + 1
    mesh_size_y = int(high[1]) - start_y + 1

    # gml:tupleListから標高データを取得
    tuple_list_element = root.find('.//gml:tupleList', namespaces)
    tuple_list_text = tuple_list_element.text.strip()
    elevation_data = [
        float(item.split(',')[1])
        for item in tuple_list_text.split('\n')
    ]

    # データなし(-9999)を0に置き換える処理
    if replace_nodata:
        elevation_data = [0.0 if value == -9999.0 else value for value in elevation_data]

    # メッシュデータ準備

    #頂点の作成
    # numpy配列を作成（startPointからhighまでの範囲）
    x_indices = np.arange(mesh_size_x)
    y_indices = np.arange(mesh_size_y)
    x_grid, y_grid = np.meshgrid(x_indices, y_indices)  # グリッド作成

    # elevation_dataのインデックス
    idx = y_grid * mesh_size_x + x_grid
    z_values = np.array(elevation_data)[idx.flatten()]

    # 全体グリッド内での実際の座標（startPointオフセットを加える）
    actual_x_grid = x_grid + start_x
    actual_y_grid = y_grid + start_y

    # X, Y座標の計算（全体グリッドサイズを使用）
    original_x = (actual_x_grid + 0.5) * east_west_distance / grid_size_x
    original_y = (actual_y_grid + 0.5) * north_south_distance / grid_size_y
    # 座標調整
    adjusted_x = original_x + calculate_hubeny(min_lat, min_lon, min_lat, origin_lon)
    adjusted_y = -original_y - calculate_hubeny(min_lat, min_lon, origin_lat, min_lon)
    # 頂点リストを構築
    verts = np.column_stack((adjusted_x.flatten(), adjusted_y.flatten(), z_values.flatten()))

    # UV座標（全体グリッドサイズを使用）
    u = (actual_x_grid + 0.5) / grid_size_x
    v = 1 - ((actual_y_grid + 0.5) / grid_size_y)
    # UV座標をフラット化してリストに変換
    uv_coords = np.column_stack((u.flatten(), v.flatten())).tolist()

    #面の作成(これはnumpyよりforで回したほうが早かった)
    faces = []

    for y in range(mesh_size_y - 1):
        for x in range(mesh_size_x - 1):
            v1 = y * mesh_size_x + x
            v2 = v1 + 1
            v3 = v1 + mesh_size_x
            v4 = v3 + 1
            faces.append((v1, v2, v4, v3))

    # Blenderでメッシュを作成
    file_name = os.path.basename(xml_file_path)
    meshname = extract_code_from_filename(file_name)
    mesh = bpy.data.meshes.new(meshname)
    mesh.from_pydata(verts, [], faces)
    mesh.update()

    obj = bpy.data.objects.new(meshname, mesh)
    bpy.context.collection.objects.link(obj)

    # UVマップ作成
    uv_layer = mesh.uv_layers.new(name="UVMap")
    for i, loop in enumerate(mesh.loops):
        uv_layer.data[i].uv = uv_coords[loop.vertex_index]
    
    
    bpy.ops.object.select_all(action='DESELECT')
    obj.select_set(True)
    bpy.context.view_layer.objects.active = obj


def apply_material_with_texture(meshname):
    """
    既存のオブジェクトに新規マテリアルを追加し、画像テクスチャを適用する関数
    :param obj: 既存のオブジェクト
    :param image_path: 画像ファイルのパス
    """
    obj = bpy.data.objects.get(meshname) 

    # 新規マテリアルを作成
    material = bpy.data.materials.new(name= "DEM_" + meshname)
    material.use_nodes = True  # ノードを有効化

    # マテリアルノードを取得
    nodes = material.node_tree.nodes
    links = material.node_tree.links

    # すべてのノードを削除
    for node in nodes:
        nodes.remove(node)

    # 必要なノードを追加
    output_node = nodes.new(type="ShaderNodeOutputMaterial")
    principled_node = nodes.new(type="ShaderNodeBsdfPrincipled")
    texture_node = nodes.new(type="ShaderNodeTexImage")

    # ノードの位置を調整（オプション）
    output_node.location = (600, 0)
    principled_node.location = (300, 0)
    texture_node.location = (0, 0)

    # ノードの名前を「DEM_Surface」に設定
    texture_node.name = "DEM_Surface"

    # ノードを接続
    links.new(principled_node.outputs["BSDF"], output_node.inputs["Surface"])
    links.new(texture_node.outputs["Color"], principled_node.inputs["Base Color"])

    # 画像テクスチャを設定
    image = bpy.data.images.get(meshname + "_std.png")
    texture_node.image = image

    # オブジェクトにマテリアルを割り当て
    obj.data.materials.append(material)

def set_new_material(xml_file_path):
    #
    file_name = os.path.basename(xml_file_path)
    meshname = extract_code_from_filename(file_name)
    
    apply_material_with_texture(meshname)

def toggle_texture_in_dem_materials():
    """
    シーン内のマテリアルのうち名前に「DEM_xxxx-xx-xx」を含むものすべてに対し、
    texture_nodeの画像を「xxxx-xx-xx_seamlessphoto.png」と「xxxx-xx-xx_std.png」の間で切り替える。
    """
    global toggle_flag
    
    # 名前に「DEM」が含まれるマテリアルを検索
    dem_materials = [mat for mat in bpy.data.materials if mat.name.startswith("DEM_")]
    
    if toggle_flag < 2:
        set_solid_mode_color_type('TEXTURE')

        # 該当する各マテリアルを処理
        for material in dem_materials:
            if material.use_nodes:  # ノードが有効化されている場合のみ処理
                nodes = material.node_tree.nodes
            
                # Texture Node（ShaderNodeTexImage）を検索
                texture_node = next((node for node in nodes if node.type == "TEX_IMAGE"), None)
            
                if texture_node:
                    # 日付を取得
                    match = re.search(r"(\d{4}-\d{2}-\d{2})", material.name)  # 正規表現でマッチング
                    date_str = match.group(1) if match else None  # マッチすれば抽出、なければNone
                
                    # 使用する画像名を切り替え
                    image_name = f"{date_str}_seamlessphoto.png" if toggle_flag else f"{date_str}_std.png"
                
                    # 新しい画像を設定
                    image = bpy.data.images.get(image_name)
                    if image:
                        texture_node.image = image
                        print(f"マテリアル '{material.name}' のテクスチャを '{image_name}' に変更しました。")
                    else:
                        print(f"画像 '{image_name}' が見つかりませんでした。")

    if toggle_flag == 2:
        set_solid_mode_color_type('MATERIAL')
    
    # フラグを切り替え
    if toggle_flag == 2:
        toggle_flag = 0
    else:
        toggle_flag += 1

def set_solid_mode_color_type(color_type):
    """
    3Dビューのソリッドモードでシェーディングカラーをテクスチャに変更する関数。
    """
    # 3Dビューを取得
    for area in bpy.context.screen.areas:
        if area.type == 'VIEW_3D':  # 3Dビューか確認
            space = area.spaces.active

            # シェーディングモードをソリッドに設定
            space.shading.type = 'SOLID'

            # ソリッドモードのカラーをテクスチャに変更
            space.shading.color_type = color_type  # 'TEXTURE' を設定
            print("3Dビューのソリッドモードのカラーをテクスチャに設定しました。")

def lib_ver_checker(package_name):
    pkg_resources._initialize()
    installed_packages = pkg_resources.WorkingSet()
    package_version = None
    
    for dist in installed_packages:
        if dist.key == package_name:
            package_version = dist.version
            break

    return package_version

def install_packeage(package_name,target_version):
    current_version = lib_ver_checker(package_name)
    print(current_version)
    print(target_version)
    
    if current_version != target_version:
        if current_version != None:
            print(f"{package_name} is installed with different version({current_version}).Uninstalling...")
            subprocess.check_call([sys.executable, "-m", "pip", "uninstall", "-y", package_name])
            print(f"Uninstalling finished")
        print(f"Installing {package_name}=={target_version}...")
        subprocess.check_call([sys.executable, "-m", "pip", "install", f"{package_name}=={target_version}"])
        print(f"Installing finished")
    else:
        print(f"{package_name}=={target_version} is already installed")


##########################################################################
#ここからblender UI実装
##########################################################################

bl_info = {
    "name": "DEM XML Importer",
    "author": "Cave Mapper",
    "version": (1, 0, 3),
    "blender": (4, 0, 2),
    "location": "3D View > Sidebar",
    "description": "国土地理院webサイトから取得したxml形式の数値標高モデルをBlenderに取り込む",
    "warning": "",
    "support": "COMMUNITY",
    "doc_url": "",
    "tracker_url": "",
    "category": "User Interface"
}
              
 # カスタムパネルのクラスを定義
class DEM_XML_PT_CustomPanel(bpy.types.Panel):
    bl_label = "Dem xml 読込"
    bl_idname = "DEM_XML_PT_CustomPanel"
    bl_space_type = "VIEW_3D"
    bl_region_type = "UI"
    bl_category = "Dem xml 読込"

    def draw(self, context):        

        layout = self.layout
        is_static_map = True if lib_ver_checker('staticmap') == '0.5.6' else False
        print(f"is_static_map: {is_static_map}")

        if is_static_map == False:
            layout.operator("demxml.install_python_module", text="Static Map インストール", icon='SORTTIME')
            layout.separator()
            layout.separator()

        if is_static_map == True:
            layout.separator()
            layout.prop(context.scene, "get_texture", text="テクスチャを取得する")

            row = layout.row()
            row.label(text="ズームレベル:")  # テキストを追加
            row.prop(context.scene, "zoom_level", text="")  # ボックスのみを表示

        layout.separator()
        layout.prop(context.scene, "replace_nodata_with_zero", text="データなしを0に置換")
        layout.separator()

        layout.operator("demxml.folder_file_operator", text="フォルダ指定xml読込", icon='SORTTIME')

        if is_static_map == True:
            layout.separator()
            layout.separator()
            layout.separator()
            layout.operator("demxml.change_texture", text="テクスチャ切替")


# フォルダ内ファイル読込のオペレータークラス
class DEMXML_OT_FolderFileOperator(bpy.types.Operator):
    bl_idname = "demxml.folder_file_operator"
    bl_label = "Folder XML Import"
    bl_description = "Select a folder and list all XML files within"

    directory: StringProperty(subtype="DIR_PATH")  # フォルダパスを保持

    def execute(self, context):
        
        # 処理開始時刻を記録
        start_time = time.time()

        bpy.context.window.cursor_set("WAIT")
        
        # 指定されたディレクトリのXMLファイルを取得
        if not os.path.isdir(self.directory):
            self.report({'ERROR'}, "Invalid directory path")
            return {'CANCELLED'}

        # XMLファイルをリスト化
        xml_files = [
            os.path.join(self.directory, file)
            for file in os.listdir(self.directory)
            if file.endswith(".xml")
        ]
        
        northwest_coords = get_northwest_coordinates(xml_files)
        
        """""""""""""""""""""""""""""""""""""""""""""""""""""""""""
        MAIN
        """""""""""""""""""""""""""""""""""""""""""""""""""""""""""
        for xml_file in xml_files:
            process_xml_and_generate_mesh(xml_file, northwest_coords, context.scene.replace_nodata_with_zero)

            print(f"context.scene.get_texture: {context.scene.get_texture}")
            if lib_ver_checker('staticmap') == '0.5.6':
                if context.scene.get_texture == True:
                    # テクスチャ取得
                    zoom_level = context.scene.zoom_level
                    calculate_pixel_dimensions(xml_file, zoom_level)

                    set_new_material(xml_file)

                    set_solid_mode_color_type('TEXTURE')

        bpy.context.space_data.clip_end = 50000
        bpy.ops.view3d.view_selected()
        bpy.context.window.cursor_set("DEFAULT")

        # 処理終了時刻を記録
        end_time = time.time()
        # 処理時間を計算して表示
        elapsed_time = end_time - start_time
        print(f"処理時間: {elapsed_time:.4f}秒")
                
        # リスト内容を出力
        print(f"northwest_coords: {northwest_coords}")
        self.report({'INFO'}, f"Found {len(xml_files)} XML files in the directory.")
        return {'FINISHED'}

    def invoke(self, context, event):
        # ディレクトリ選択ダイアログを開く
        context.window_manager.fileselect_add(self)
        self.directory = ""  # 選択されたディレクトリを保存する
        return {'RUNNING_MODAL'}

# テクスチャ切替のオペレータークラス
class DEMXML_OT_ChangeTexture(bpy.types.Operator):
    bl_idname = "demxml.change_texture"
    bl_label = "Change texture"
    bl_description = "Change texture"

    def execute(self, context):
        toggle_texture_in_dem_materials()                
        return {'FINISHED'}

# static mapセットアップのオペレータークラス
class DEMXML_OT_InstallPythonModule(bpy.types.Operator):
    bl_idname = "demxml.install_python_module"
    bl_label = "Install python module"
    bl_description = "地形図および航空写真のテクスチャを取得するには、PythonモジュールのStatic Mapをインストールする必要があります。このボタンを押すとインストールされます。"

    def execute(self, context):
        bpy.context.window.cursor_set("WAIT")
        install_packeage('staticmap','0.5.6')
        bpy.context.window.cursor_set("DEFAULT")
        return {'FINISHED'}

# クラスの登録と解除
classes = [DEM_XML_PT_CustomPanel, DEMXML_OT_FolderFileOperator, DEMXML_OT_ChangeTexture, DEMXML_OT_InstallPythonModule] 


def register():  
    # プロパティを登録
    bpy.types.Scene.xml_file_paths = []

    #プロパティを設定
    bpy.types.Scene.get_texture = bpy.props.BoolProperty(
        name="get_texture",
        description="国土地理院から対応する範囲の地形図と航空写真を取得し、テクスチャとして設定します。インターネットに接続して使う必要があります。",
        default=True
    )

    bpy.types.Scene.zoom_level = bpy.props.IntProperty(
        name="zoom_level",
        description="取得する地形図と航空写真のズームレベルを設定します。値が大きいほど、サイズが大きく詳細な画像を取得しますが、処理が重くなります",
        default=16,
        min=14,  # 最小値
        max=18  # 最大値
    )

    bpy.types.Scene.replace_nodata_with_zero = bpy.props.BoolProperty(
        name="replace_nodata_with_zero",
        description="データなし(-9999)の値を0に置き換えてメッシュを生成します",
        default=False
    )

    for cls in classes:
        bpy.utils.register_class(cls)


   
def unregister():
    for cls in classes:
        bpy.utils.unregister_class(cls)

    # プロパティを解除
    del bpy.types.Scene.xml_file_paths

    del bpy.types.Scene.get_texture
    del bpy.types.Scene.zoom_level
    del bpy.types.Scene.replace_nodata_with_zero

if __name__ == "__main__":
    register()
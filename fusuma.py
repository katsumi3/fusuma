import bpy, bmesh, bpy.ops
from math import pi, radians, degrees, sqrt, cos, acos, tan, sin, atan
from mathutils import Vector, Matrix, Euler
from bpy.props import FloatProperty, EnumProperty, BoolProperty, IntProperty

bl_info = {
    "name": "襖を付けるアドオン",
    "author": "勝己（kastumi）",
    "version": (1, 0),
    "blender": (2, 81, 0),
    "location": "3Dビューポート > 追加 > メッシュ",
    "description": "襖を付けるアドオン",
    "warning": "",
    "support": "COMMUNITY",
    "wiki_url": "",
    "tracker_url": "",
    "category": "Object"
}

############################################################


def set_emp(p, mat):
    msh = p.data

    x = []
    y = []
    z = []

    for e in msh.edges:
        id_0 = e.vertices[0]
        id_1 = e.vertices[1]
        v = msh.vertices
        c_p = (mat @ v[id_0].co + mat @ v[id_1].co) / 2
        x.append(c_p[0])
        y.append(c_p[1])
        z.append(c_p[2])

    id = z.index(min(z))
    emp_x = x[id]
    emp_y = y[id]
    emp_z = min(z)

    emp_p = Vector((emp_x, emp_y, emp_z))

    m_id_0 = msh.edges[id].vertices[0]
    m_id_1 = msh.edges[id].vertices[1]
    muki = mat @ msh.vertices[m_id_0].co - mat @ msh.vertices[m_id_1].co
    norm = msh.polygons[0].normal
    mx_inv = mat.inverted()
    mx_norm = mx_inv.transposed().to_3x3()
    world_no = mx_norm @ norm
    world_no.normalize()

    cro = muki.cross(world_no)
    cro.normalize()
    muki.normalize()

    mat_emp = Matrix([[muki.x, -cro.x, world_no.x, emp_p.x],
                      [muki.y, -cro.y, world_no.y, emp_p.y],
                      [muki.z, -cro.z, world_no.z, emp_p.z], [0, 0, 0, 1]])

    return mat_emp


#def center_mat-->辺の中心のmatrix_worldの値を返す


def center_mat(xv, yv, p_mat, p_norm):
    xv = xv.vert.co
    yv = yv.vert.co
    loc = (p_mat @ xv + p_mat @ yv) / 2
    muki = (p_mat @ yv - p_mat @ xv)
    mx_inv = p_mat.inverted()
    mx_norm = mx_inv.transposed().to_3x3()
    world_no = mx_norm @ p_norm
    world_no.normalize()
    cro = muki.cross(world_no)
    cro.normalize()
    muki.normalize()
    m = Matrix([[muki.x, -cro.x, world_no.x, loc.x],
                [muki.y, -cro.y, world_no.y, loc.y],
                [muki.z, -cro.z, world_no.z, loc.z], [0, 0, 0, 1]])
    return m


###########################
def shear_rad(bm, rad, id, n, pos):
    bm.faces.ensure_lookup_table()
    axis = 'Z'
    if axis == 'Z':
        selected = [v.co.y for v in bm.verts if v.select]
    elif axis == 'Y':
        selected = [v.co.z for v in bm.verts if v.select]
    t = max(selected) - min(selected)
    shear_va = tan(rad)
    va = [t * shear_va, 0, 0]
    #    bpy.ops.transform.shear(value=shear_va,
    #                            orient_axis=axis,
    #                            orient_axis_ortho='X',
    #                            orient_type='LOCAL',
    #                            orient_matrix_type='GLOBAL')

    obj = bpy.context.object
    for v in bm.verts:
        if v.select:
            if v.index == 3 or v.index == 2:
                if pos == "1":
                    v.co = v.co + Vector(
                        [c / s for c, s in zip(va, obj.scale)])
                elif pos == "3":
                    v.co = v.co - Vector(
                        [c / s for c, s in zip(va, obj.scale)])
            elif v.index == 7 or v.index == 6:
                if pos == "1":
                    v.co = v.co + Vector(
                        [c / s for c, s in zip(va, obj.scale)])
                elif pos == "3":
                    v.co = v.co - Vector(
                        [c / s for c, s in zip(va, obj.scale)])


################################################
#def obj_length()->辺の長さにobjの大きさを合わせる
def obj_length(mat, edge):
    l = (mat @ edge.verts[0].co - mat @ edge.verts[1].co).length
    scale = [l]
    scale_mat = Matrix.Scale(scale[0], 4, (1, 0, 0))
    return scale_mat


#############################################
def end_face(ids, width, length_adj, xv, bme, t, rad, n, pos):
    bpy.ops.mesh.select_all(action='DESELECT')
    id = ids
    bme.faces[id].select_set(True)
    #bpy.ops.transform.translate(value=(0, 0, length_adj), orient_type='NORMAL')
    obj = bpy.context.object
    va = Vector((0, 0, length_adj))
    mat = obj.matrix_world
    m = mat.copy()

    norm = bme.faces[id].normal
    mx_inv = mat.inverted()
    mx_norm = mx_inv.transposed().to_3x3()
    world_norm = mx_norm @ norm
    world_norm.normalize()
    m[0][2] = world_norm[0]
    m[1][2] = world_norm[1]
    m[2][2] = world_norm[2]
    gro = m @ va
    lo = mx_inv @ gro
    for v in bme.faces[id].verts:
        v.co = v.co + lo

    if round(degrees(rad)) != 0:
        shear_rad(bme, rad, id, n, pos)


def edg_length3(mat, edge):
    l = (mat @ edge.verts[0].co - mat @ edge.verts[1].co).length
    return l


def ca_angle(loop, mat):
    v0 = mat @ loop.vert.co - mat @ loop.link_loop_next.vert.co
    v1 = mat @ loop.link_loop_prev.vert.co - mat @ loop.vert.co
    rad = pi - v0.angle(v1)
    return rad


def norm_move(obj, z_c):

    a = 0
    b = 0
    c = z_c

    val = Vector((a, b, c))

    msh = obj.data
    mat = obj.matrix_world
    m = mat.copy()

    norm = msh.polygons[0].normal
    mx_inv = mat.inverted()
    mx_norm = mx_inv.transposed().to_3x3()
    world_norm = mx_norm @ norm
    world_norm.normalize()

    msh = obj.data
    muki0 = mat @ msh.vertices[msh.loops[0].vertex_index].co
    muki1 = mat @ msh.vertices[msh.loops[1].vertex_index].co

    muki = (muki1 - muki0).normalized()

    #muki = Vector((1,0,0))

    cro = muki.cross(world_norm).normalized()
    loc = obj.location
    m = Matrix([[muki.x, -cro.x, world_norm.x, loc.x],
                [muki.y, -cro.y, world_norm.y, loc.y],
                [muki.z, -cro.z, world_norm.z, loc.z], [0, 0, 0, 1]])

    #別のオブジェクトをノーマルの向きにそって移動する場合
    #bpy.data.objects['モンキー'].matrix_world = m
    #bpy.data.objects['モンキー'].location = m @ val

    #自分自身を移動させる場合（回転等必要がない）
    return m @ val


def uv_rot(loop_no, plane):
    #UVテクスチャをz_min_noを下にして並べる
    plane = bpy.context.object
    msh = plane.data
    p_mat = plane.matrix_world.copy()
    norm = msh.polygons[0].normal
    mx_inv = p_mat.inverted()
    mx_norm = mx_inv.transposed().to_3x3()
    world_norm = mx_norm @ norm
    world_norm.normalize()

    vertex_no = msh.loops[loop_no].vertex_index
    muki0 = p_mat @ msh.vertices[vertex_no].co
    if loop_no + 1 == len(msh.loops):
        muki1 = p_mat @ msh.vertices[msh.loops[0].vertex_index].co
    else:
        muki1 = p_mat @ msh.vertices[msh.loops[loop_no + 1].vertex_index].co

    muki = (muki1 - muki0).normalized()

    cro = muki.cross(world_norm).normalized()
    loc = plane.location

    m = Matrix([[muki.x, -cro.x, world_norm.x, loc.x],
                [muki.y, -cro.y, world_norm.y, loc.y],
                [muki.z, -cro.z, world_norm.z, loc.z], [0, 0, 0, 1]])

    new_mat = m.inverted() @ p_mat

    uv_p = []
    for l in msh.loops:
        p = new_mat @ msh.vertices[l.vertex_index].co
        uv_p.append([p.x, p.y])

    max_x = max(b[0] for b in uv_p)
    min_x = min(b[0] for b in uv_p)
    dif_x = abs(max_x - min_x)
    max_y = max(b[1] for b in uv_p)
    min_y = min(b[1] for b in uv_p)
    dif_y = abs(max_y - min_y)
    for p in uv_p:
        p[0] = (p[0] - min_x) / dif_x
        p[1] = (p[1] - min_y) / dif_y

    bpy.ops.object.mode_set(mode='EDIT')
    bm = bmesh.from_edit_mesh(msh)
    bm.faces.ensure_lookup_table()
    uv_layer = bm.loops.layers.uv.active

    for v in bm.faces[0].loops:
        v[uv_layer].uv = uv_p[v.index]
    bmesh.update_edit_mesh(msh)
    bpy.ops.object.mode_set(mode='OBJECT')


def set_hikide(w, z_min_no, inv_vh, yz_move, end_objs, mai, plane, plane_t):
    #襖の引き手を付ける
    #wは1から始まるので注意
    if w % 2 == 1:
        #辺0が短いとき　z_min_no%2==0　になる
        #inv_vh は反転しない時が1、反転する時が0
        #yz_move == "y"はTrueの時が1
        if z_min_no % 2 == 0 and inv_vh == 1 and yz_move == "y":
            wood = end_objs[-round(mai / 4)]
            l = abs(wood.data.vertices[7].co.x - wood.data.vertices[3].co.x)
            hikide_pos_x = (l / 19) * 1.5

        elif z_min_no % 2 == 0 and inv_vh == 1 and yz_move == "z":
            wood = end_objs[-(len(plane.data.vertices)) + round(mai / 4)]
            l = abs(wood.data.vertices[7].co.x - wood.data.vertices[3].co.x)
            hikide_pos_x = (l / 19) * 1.5

        elif z_min_no % 2 == 0 and inv_vh == 0 and yz_move == "y":
            wood = end_objs[-(len(plane.data.vertices))]
            l = -abs(wood.data.vertices[7].co.x - wood.data.vertices[3].co.x)
            hikide_pos_x = (l / 19) * 1.5

        elif z_min_no % 2 == 0 and inv_vh == 0 and yz_move == "z":
            wood = end_objs[-(len(plane.data.vertices))]
            l = abs(wood.data.vertices[7].co.x - wood.data.vertices[3].co.x)
            hikide_pos_x = (l / 19) * 1.5

        elif z_min_no % 2 == 1 and inv_vh == 1 and yz_move == "y":
            wood = end_objs[-(len(plane.data.vertices))]
            l = abs(wood.data.vertices[7].co.x - wood.data.vertices[3].co.x)
            hikide_pos_x = (l / 19) * 1.5

        elif z_min_no % 2 == 1 and inv_vh == 1 and yz_move == "z":
            wood = end_objs[-(len(plane.data.vertices))]
            l = abs(wood.data.vertices[7].co.x - wood.data.vertices[3].co.x)
            hikide_pos_x = (l / 19) * 1.5

        elif z_min_no % 2 == 1 and inv_vh == 0 and yz_move == "y":
            wood = end_objs[-(len(plane.data.vertices)) + round(mai / 4)]
            l = -abs(wood.data.vertices[7].co.x - wood.data.vertices[3].co.x)
            hikide_pos_x = (l / 19) * 1.5

        elif z_min_no % 2 == 1 and inv_vh == 0 and yz_move == "z":
            wood = end_objs[-(len(plane.data.vertices)) + round(mai / 4)]
            l = abs(wood.data.vertices[7].co.x - wood.data.vertices[3].co.x)
            hikide_pos_x = (l / 19) * 1.5

    elif w % 2 == 0:
        #辺0が短いとき　z_min_no%2==0　になる
        #inv_vh は反転しない時が1、反転する時が0
        #yz_move == "y"はTrueの時が1
        if z_min_no % 2 == 0 and inv_vh == 1 and yz_move == "y":
            wood = end_objs[-(len(plane.data.vertices)) + round(mai / 4)]
            l = -abs(wood.data.vertices[7].co.x - wood.data.vertices[3].co.x)
            hikide_pos_x = (l / 19) * 1.5

        elif z_min_no % 2 == 0 and inv_vh == 1 and yz_move == "z":
            wood = end_objs[-round(mai / 4)]
            l = abs(wood.data.vertices[7].co.x - wood.data.vertices[3].co.x)
            hikide_pos_x = (l / 19) * 1.5

        elif z_min_no % 2 == 0 and inv_vh == 0 and yz_move == "y":
            wood = end_objs[-(len(plane.data.vertices)) + round(mai / 2)]
            l = abs(wood.data.vertices[7].co.x - wood.data.vertices[3].co.x)
            hikide_pos_x = (l / 19) * 1.5

        elif z_min_no % 2 == 0 and inv_vh == 0 and yz_move == "z":
            wood = end_objs[-(len(plane.data.vertices)) + round(mai / 2)]
            l = -abs(wood.data.vertices[7].co.x - wood.data.vertices[3].co.x)
            hikide_pos_x = (l / 19) * 1.5

        elif z_min_no % 2 == 1 and inv_vh == 1 and yz_move == "y":
            wood = end_objs[-(len(plane.data.vertices)) + round(mai / 2)]
            l = -abs(wood.data.vertices[7].co.x - wood.data.vertices[3].co.x)
            hikide_pos_x = (l / 19) * 1.5

        elif z_min_no % 2 == 1 and inv_vh == 1 and yz_move == "z":
            wood = end_objs[-(len(plane.data.vertices)) + round(mai / 2)]
            l = -abs(wood.data.vertices[7].co.x - wood.data.vertices[3].co.x)
            hikide_pos_x = (l / 19) * 1.5

        elif z_min_no % 2 == 1 and inv_vh == 0 and yz_move == "y":
            wood = end_objs[-round(mai / 4)]
            l = abs(wood.data.vertices[7].co.x - wood.data.vertices[3].co.x)
            hikide_pos_x = (l / 19) * 1.5

        elif z_min_no % 2 == 1 and inv_vh == 0 and yz_move == "z":
            wood = end_objs[-round(mai / 4)]
            l = -abs(wood.data.vertices[7].co.x - wood.data.vertices[3].co.x)
            hikide_pos_x = (l / 19) * 1.5

        hikide_pos_x = (l / 19) * 1.5

    p_m = plane.modifiers.new(type="BOOLEAN", name="bool")
    currentPath = bpy.utils.script_paths()[2] + "/addons/"
    filename = "hikide.blend"
    path = currentPath + filename + "/"
    bpy.ops.wm.append(directory=path + "Object/",
                      link=False,
                      filename="hikide")
    bpy.context.view_layer.objects.active = bpy.context.selected_objects[0]
    hikide = bpy.context.object
    p_m.object = hikide
    p_m.operation = 'DIFFERENCE'
    hikide.location = wood.location
    hikide.rotation_euler = wood.rotation_euler
    val = [hikide_pos_x, 0.069, plane_t / 2 + 0.001]
    va = [d / s for d, s in zip(val, wood.scale)]
    mat = wood.matrix_world.copy()
    hikide.location = mat @ Vector(va)

    co = bpy.context.collection

    msh = bpy.data.meshes[hikide.data.name].copy()
    hikide2 = bpy.data.objects.new(name='hikide', object_data=msh)

    hikide2.location = hikide.location
    hikide2.rotation_euler = hikide.rotation_euler
    hikide2.rotation_euler[1] = hikide2.rotation_euler[1] + pi
    co.objects.link(hikide2)
    val = [0, 0, plane_t + 0.001 * 2]
    va = [d / s for d, s in zip(val, hikide2.scale)]

    bpy.context.view_layer.update()
    hiki_mat = hikide2.matrix_world.copy()
    hikide2.location = hiki_mat @ Vector(va)
    #引き手2のモディファイアがここではつけれないので別途def set_hikide2で作る
    return hikide, hikide2


def set_hikide2(plane, hikide):
    p_m = plane.modifiers[-1]
    p_m.object = hikide
    p_m.operation = 'DIFFERENCE'


class hasigo:
    def __init__(self):
        self.waku_type = 0
        self.sw = 0
        self.cont = 0
        self.p_len = 0
        self.angle = 0.0
        self.another_angle = 0.0
        self.other_angle = 0.0
        self.another_width = 0.0
        self.width = 0.0
        self.is_conv = 0
        self.other_is_conv = 0
        self.p_mat = 0.0
        self.another_edge = 0.0
        self.edge = 0.0
        self.rad = 0.0
        self.length_adj = 0.0
        self.n = 0.0

    def tijimi(self):
        #print("aaaaaaaaaaaaaaaaaaaaaa")
        if round(degrees(self.angle)) in {180, 0}:
            self.length_adj = 0
            self.rad = 0

        elif self.waku_type == "3":
            self.length_adj = 0
            self.rad = (self.angle / 2 - pi / 2)
            if self.is_conv == 0:
                self.rad = -(self.angle / 2 - pi / 2)

        elif edg_length3(
                self.p_mat, self.another_edge) < self.another_width / sin(
                    self.angle) and self.is_conv and 90 < degrees(self.angle):
            self.rad = (self.angle / 2 - pi / 2)
            self.length_adj = 0
            #print("el1")

        elif edg_length3(self.p_mat,
                         self.another_edge) < self.another_width / sin(
                             self.angle) and not self.is_conv:
            self.rad = -(self.angle / 2 - pi / 2)
            self.length_adj = 0

        elif edg_length3(self.p_mat, self.edge) < self.width / sin(
                self.angle) and not self.is_conv:
            self.rad = -(self.angle / 2 - pi / 2)
            self.length_adj = 0

        elif edg_length3(self.p_mat, self.edge) < self.another_width / sin(
                self.angle) and self.is_conv:
            self.rad = (self.angle / 2 - pi / 2)
            self.length_adj = 0

        elif edg_length3(self.p_mat, self.another_edge) * abs(tan(
                self.angle)) < self.another_width and self.is_conv and round(
                    degrees(self.angle)) != 0:
            self.rad = (self.angle - pi / 2)
            self.another_width = edg_length3(
                self.p_mat, self.another_edge) * abs(tan(self.angle))
            self.length_adj = self.sw * self.is_conv * (-self.another_width /
                                                        sin(self.angle))

        elif self.waku_type == '1' and self.cont == 0 and self.p_len % 2 == 1 and self.n % 2 == 0:
            #０個目で面の頂点が奇数の場合
            self.length_adj = 0
            self.rad = [-1, 1][self.is_conv] * self.angle / 2 - pi / 2
            #print(3)

        elif self.waku_type == '2' and self.cont == 0 and self.p_len % 2 == 1 and self.n % 2 == 0:
            #０個目で面の頂点が奇数の場合
            print("type2_fist_kisuu")
            self.rad = [-1, 1][self.is_conv] * (self.angle - pi / 2)
            self.length_adj = [0, 1][self.sw] * [-1, 1][self.is_conv] * (
                -self.width / sin(self.angle))

        elif self.waku_type == '1' and self.cont == self.p_len - 1 and self.p_len % 2 == 1 and self.n % 2 == 1:
            #最後の1個目で面の頂点が奇数の場合

            self.rad = [-1, 1][self.is_conv] * self.angle / 2 - pi / 2
            self.length_adj = 0
            #print("el8")

        elif self.waku_type == '2' and self.cont == self.p_len - 1 and self.p_len % 2 == 1 and self.n % 2 == 1:
            #最後の1個目で面の頂点が奇数の場合
            self.rad = [-1, 1][self.is_conv] * (self.angle - pi / 2)
            self.length_adj = self.sw * [0, -1][self.is_conv] * (
                self.width / sin(self.angle))

        elif 0 <= self.cont < self.p_len:
            if self.sw == 0 and self.is_conv == 0:
                if self.waku_type == '2' and self.cont == 0:
                    print("waku_type2")
                    self.rad = [-1, 1][self.is_conv] * (self.angle - pi / 2)
                    self.length_adj = [1, -1][self.sw] * [
                        -1, 1
                    ][self.is_conv] * (-self.width / sin(self.angle))

                else:
                    print("futu-if")
                    self.rad = self.sw * [-1, 1][self.is_conv] * (self.angle -
                                                                  pi / 2)
                    self.length_adj = (self.another_width / sin(self.angle))

            else:
                if self.waku_type == '2' and self.cont == 0:
                    #print("futuu-type2")
                    self.rad = [-1, 1][self.is_conv] * (self.angle - pi / 2)
                    self.length_adj = [0, -1][self.sw] * [
                        1, -1
                    ][self.is_conv] * (-self.another_width / sin(self.angle))
                else:
                    #普通の場合
                    self.rad = [-1, 1][self.is_conv] * (self.angle - pi / 2)
                    self.length_adj = self.sw * self.is_conv * (
                        -self.another_width / sin(self.angle))
                    #print("self.length_adj",self.length_adj)

            #print("angle", degrees(self.angle))
        #print("end")


class FUSUMA_OT_CreateObject(bpy.types.Operator):

    bl_idname = "object.fusuma_create_object"
    bl_label = "襖"
    bl_description = "襖を付けます"
    bl_options = {'REGISTER', 'UNDO'}
    pri_set: EnumProperty(name="枠の形状",
                          description="移動軸を設定します",
                          default='0',
                          items=[
                              ('0', "手動", "手動にします"),
                              ('1', "ふすま", "ふすまにします"),
                              ('2', "掃き出し窓", "掃き出し窓にします"),
                          ])
    yz_move: EnumProperty(name="分割方向",
                          description="分割する方向を設定します",
                          default='y',
                          items=[
                              ('y', "Y", "Y方向に分割します"),
                              ('z', "Z", "Z方向に分割します"),
                          ])

    mai: IntProperty(
        name="枚数",
        description="襖の枚数を設定します",
        min=1,
        default=1,
    )
    plane_t: FloatProperty(
        name="平面の厚み",
        description="厚みを設定します",
        default=0.032,
    )
    v_width: FloatProperty(
        name="縦の枠の幅",
        description="幅を設定します",
        default=0.018,
    )
    h_width: FloatProperty(
        name="横の枠の幅",
        description="幅を設定します",
        default=0.025,
    )
    t0: FloatProperty(
        name="厚み",
        description="厚みを設定します",
        default=0.017,
    )
    t1: FloatProperty(
        name="厚み",
        description="厚みを設定します",
        default=0.019,
    )
    waku_color: EnumProperty(name="内枠の素材",
                             description="内枠の素材を設定します",
                             default='0',
                             items=[
                                 ('0', "木目", "木目調にします"),
                                 ('1', "黒漆", "黒漆にします"),
                                 ('2', "アルミ", "アルミサッシにします"),
                             ])

    hiki: BoolProperty(
        name="引き手",
        description="引き手を付けるかどうかを設定します",
        default=False,
    )
    inv: BoolProperty(
        name="反転",
        description="反転するかを設定します",
        default=False,
    )
    sep: BoolProperty(
        name="テクスチャ分割",
        description="テクスチャを分割するかを設定します",
        default=False,
    )
    soto: BoolProperty(
        name="外枠追加",
        description="外枠を追加するかを設定します",
        default=False,
    )
    soto_v_width: FloatProperty(
        name="外枠の縦の枠の幅",
        description="外枠の幅を設定します",
        default=0.018,
    )
    soto_h_width: FloatProperty(
        name="外枠の横の枠の幅",
        description="外枠の幅を設定します",
        default=0.025,
    )
    soto_t0: FloatProperty(
        name="外枠の厚み",
        description="外枠の厚みを設定します",
        default=0.099,
    )
    soto_t1: FloatProperty(
        name="外枠の厚み",
        description="外枠の厚みを設定します",
        default=0.097,
    )
    soto_waku_color: EnumProperty(name="外枠の素材",
                                  description="外枠の素材を設定します",
                                  default='0',
                                  items=[
                                      ('0', "木目", "木目調にします"),
                                      ('1', "黒漆", "黒漆にします"),
                                      ('2', "アルミ", "アルミサッシにします"),
                                  ])

    # メニューを実行したときに呼ばれる関数
    def execute(self, context):

        #初期設定####################################
        waku_type = "1"  #1:梯子状　2:風車風　#3平留め継ぎ(額縁風)
        #waku_type = self.waku_type
        #v_width = 8  # 梯子の縦になってる木の幅
        #h_width = 1  # 梯子の横の方の木の幅
        #width = 0  # 枠の木の幅（仮）
        #inv_vh = 0
        #t = 1.5  # 厚み
        mai = self.mai
        hikide = 1
        pri_set = self.pri_set
        ##プリセット
        #0：手動
        #1：ふすま
        #v_width=18mm h_width=25mm t0=17mm u_fukasa=2mm sa=1mm
        #2:掃き出し窓
        #v_width=50mm h_width=40mm t0=21mm u_fukasa=10mm sa=1.5mm
        yz_move = self.yz_move

        sep = self.sep
        soto = self.soto
        waku_pos = "1"

        if pri_set == "0":
            #0：手動
            print("nani mo nasi")
            plane_t = self.plane_t

        elif pri_set == "1":
            #1：ふすま
            self.v_width = 0.018
            self.h_width = 0.025
            u_fukasa = 0.002
            self.plane_t = self.v_width - u_fukasa * 2
            self.t0 = 0.017
            sa = 0.001
            self.t1 = self.t0 - sa * 2
            self.soto_v_width = 0.018
            self.soto_h_width = 0.025
            soto_sa = 0.001
            self.soto_t0 = 0.099
            self.soto_t1 = self.soto_t0 - soto_sa * 2
            self.hiki = True
            self.waku_color = "1"
            self.soto_waku_color = "0"
        elif pri_set == "2":
            #2:掃き出し窓
            self.v_width = 0.050
            self.h_width = 0.040
            u_fukasa = 0.002
            self.plane_t = self.v_width - u_fukasa * 2
            self.t0 = 0.021
            sa = 0.001
            self.t1 = self.t0 - sa * 2
            self.soto_v_width = 0.050
            self.soto_h_width = 0.040
            self.soto_t0 = 0.021
            soto_sa = 0.001
            self.soto_t1 = self.t0 - soto_sa * 2
            self.hiki = False
            self.waku_color = "2"
            self.soto_waku_color = "2"

        plane_t = self.plane_t
        t = [0, 0]
        soto_t = [0, 0]
        v_width = self.v_width
        h_width = self.h_width
        soto_v_width = self.soto_v_width
        soto_h_width = self.soto_h_width
        hiki = self.hiki
        waku_color = self.waku_color
        soto_waku_color = self.soto_waku_color
        if self.inv == False:
            inv_vh = 1
            t[0] = self.t0
            t[1] = self.t1
            soto_t[0] = self.soto_t0
            soto_t[1] = self.soto_t1
            dp_width = self.v_width
            dp0_width = self.h_width
            soto_dp_width = self.soto_v_width
            soto_dp0_width = self.soto_h_width
        else:
            inv_vh = 0
            t[0] = self.t1
            t[1] = self.t0
            dp_width = self.h_width
            dp0_width = self.v_width
            soto_t[0] = self.soto_t1
            soto_t[1] = self.soto_t0
            soto_dp_width = self.soto_h_width
            soto_dp0_width = self.soto_v_width

        if self.yz_move == "z":
            dp_width, dp0_width = dp0_width, dp_width
            soto_dp_width, soto_dp0_width = soto_dp0_width, soto_dp_width
        if waku_color == "0":
            ki_name = "ki"
        elif waku_color == "1":
            ki_name = "ki_black"
        elif waku_color == "2":
            ki_name = "ki_alumi"
        if soto_waku_color == "0":
            soto_ki_name = "ki"
        elif soto_waku_color == "1":
            soto_ki_name = "ki_black"
        elif soto_waku_color == "2":
            soto_ki_name = "ki_alumi"
        ############################################

        orig_plane = bpy.context.object
        if soto == True:
            soto_waku = orig_plane.copy()
            soto_waku.data = orig_plane.data.copy()
        co = bpy.context.collection
        #平面にマテリアルが設定されてなかったらマテリアルとuvを追加する
        currentPath = bpy.utils.script_paths()[2] + "/addons/"
        filename = "hikide.blend"
        path = currentPath + filename + "/"
        start_mats = [m.name for m in bpy.data.materials]
        if len(orig_plane.material_slots) == 0:
            if pri_set == "2":
                bpy.ops.wm.append(directory=path + "Material/",
                                  link=False,
                                  filename="glass")
            else:
                bpy.ops.wm.append(directory=path + "Material/",
                                  link=False,
                                  filename="fusuma_pic")
            bpy.ops.object.material_slot_add()
            end_mats = [m.name for m in bpy.data.materials]
            new_mat = list(set(end_mats) - set(start_mats))

            fusuma_pic = bpy.data.materials[new_mat[0]]
            orig_plane.active_material = fusuma_pic

        orig_p_mat = orig_plane.matrix_world
        msh = orig_plane.data
        #ｚの値が小さいloopを一番下と仮定する（z_min_no）
        z = []
        for l in msh.loops:
            index_no0 = l.vertex_index
            if l.vertex_index == len(msh.loops) - 1:
                index_no1 = msh.loops[0].vertex_index
            else:
                index_no1 = msh.loops[index_no0 + 1].vertex_index
            z0 = orig_p_mat @ msh.vertices[index_no0].co
            z1 = orig_p_mat @ msh.vertices[index_no1].co
            z_center = (z0 + z1) / 2
            z.append(z_center[2])
        z_min_no = z.index(min(z))
        z_min_vertex_no = msh.loops[z_min_no].vertex_index

        #UVテクスチャの座標設定
        uv_rot(z_min_no, orig_p_mat)

        muki0 = orig_p_mat @ msh.vertices[z_min_vertex_no].co
        if z_min_no + 1 == len(msh.loops):
            muki1 = orig_p_mat @ msh.vertices[msh.loops[0].vertex_index].co
        else:
            muki1 = orig_p_mat @ msh.vertices[msh.loops[z_min_no +
                                                        1].vertex_index].co
        muki = (muki1 - muki0).normalized()
        norm = orig_plane.data.polygons[0].normal
        mx_inv = orig_p_mat.inverted()
        mx_norm = mx_inv.transposed().to_3x3()
        world_norm = mx_norm @ norm
        world_norm.normalize()
        cro = muki.cross(world_norm).normalized()
        loc = orig_plane.location

        m = Matrix([[muki.x, -cro.x, world_norm.x, loc.x],
                    [muki.y, -cro.y, world_norm.y, loc.y],
                    [muki.z, -cro.z, world_norm.z, loc.z], [0, 0, 0, 1]])

        x_axis = []
        y_axis = []
        z_axis = []
        for v in msh.vertices:
            vvv = m.inverted() @ orig_p_mat @ v.co + loc
            x_axis.append(vvv.x)
            y_axis.append(vvv.y)
            z_axis.append(vvv.z)

        #ローカル座標での各座標の最小の場所をグローバル座標にする
        zero_p = m @ (Vector((min(x_axis), min(y_axis), min(z_axis))) - loc)
        lo_zero_p = orig_p_mat.inverted() @ zero_p

        if yz_move == "y":
            max_p = m @ (Vector((max(x_axis), min(y_axis), min(z_axis))) - loc)
            max_width = (zero_p - max_p).length
            print("max_width =", max_width)
            w0 = (max_width + dp_width * (mai // 2)) / mai
            trans_val = (w0 / max_width, 1, 1)

        elif yz_move == "z":
            max_p = m @ (Vector((min(x_axis), max(y_axis), min(z_axis))) - loc)
            max_width = (zero_p - max_p).length
            w0 = (max_width + dp_width * (mai // 2)) / mai
            trans_val = (1, w0 / max_width, 1)

#        bpy.ops.transform.resize(
#            value = trans_val,
#            orient_type='LOCAL',
#            center_override = zero_p)
#        #print(zero_pppp,"vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv")

        for v in msh.vertices:
            vvv = m.inverted() @ orig_p_mat @ v.co + loc
            xx = vvv.x * trans_val[0]
            yy = vvv.y * trans_val[1]
            zz = vvv.z * trans_val[2]
            v.co = orig_p_mat.inverted() @ (m @ (Vector((xx, yy, zz)) - loc))

        vvv = m.inverted() @ orig_p_mat @ lo_zero_p + loc
        xx = vvv.x * trans_val[0]
        yy = vvv.y * trans_val[1]
        zz = vvv.z * trans_val[2]
        new_zero_p = m @ (Vector((xx, yy, zz)) - loc)
        print("new_zero_p = ", new_zero_p)
        print("zero_p = ", zero_p)
        dist = zero_p - new_zero_p
        orig_plane.location = orig_plane.location + dist

        w = 1
        wn = 0
        boko = [0, 0, 1, 1]
        #print("_____________\start/____________")
        emps = []
        all_planes = [orig_plane]
        all_utigawa = []
        if hiki == 1:
            hiki2 = []
        for w in range(1, mai + 1, 1):
            start_objs = [n for n in bpy.context.view_layer.objects]
            if w != 1:
                if w % 4 == 2 or w % 4 == 0:
                    wn = wn + w0 - dp_width
                elif w % 4 == 1 or w % 4 == 3:
                    wn = wn + w0
                bpy.ops.object.select_all(action='DESELECT')
                orig_plane.select_set(True)
                bpy.context.view_layer.objects.active = orig_plane

                if yz_move == "y":
                    dp_val = Vector((wn, 0, boko[w % 4] * max(t)))
                    emp_p = zero_p + Vector((0, w0 / 2 + w0 * (w - 1), 0))

                elif yz_move == "z":
                    dp_val = Vector((0, wn, boko[w % 4] * max(t)))
                    emp_p = zero_p + Vector((w0 / 2 + w0 * (w - 1), 0, 0))


#                bpy.ops.object.duplicate_move(
#                    OBJECT_OT_duplicate={
#                        "linked":False, "mode":'TRANSLATION'},
#                            TRANSFORM_OT_translate={"value":dp_val,
#                                    "orient_type":'LOCAL'})

                src_obj = bpy.context.object

                new_plane = src_obj.copy()
                new_plane.data = src_obj.data.copy()
                bpy.context.view_layer.update()
                new_plane.matrix_world = m.inverted() @ new_plane.matrix_world
                new_plane.location = new_plane.location + dp_val
                co.objects.link(new_plane)
                bpy.context.view_layer.update()
                new_plane.matrix_world = m @ new_plane.matrix_world
                all_planes.append(new_plane)

                src_obj.select_set(False)
                new_plane.select_set(True)
                bpy.context.view_layer.objects.active = new_plane

            plane = bpy.context.object
            msh = plane.data
            bpy.ops.object.mode_set(mode='EDIT')
            obj = bpy.context.active_object
            msh = obj.data
            bm = bmesh.from_edit_mesh(msh)
            bm.faces.ensure_lookup_table()

            if yz_move == "y":
                emp_p = zero_p + Vector((0, w0 / 2 + (w - 1) * w0, 0))
            elif yz_move == "z":
                emp_p = zero_p + Vector((w0 / 2 + (w - 1) * w0, 0, 0))

            uv_layer = bm.loops.layers.uv.active

            mat = plane.matrix_world
            quat = mat.decompose()[1]
            euler = quat.to_euler()
            euler[0] = 0
            euler[2] = euler[1]
            euler[1] = 0

            if sep == True:
                if w == 1:
                    uv_x = []
                    uv_y = []
                    for vert in bm.faces[0].loops:
                        uv_x.append(vert[uv_layer].uv[0])
                        uv_y.append(vert[uv_layer].uv[1])
                    if yz_move == "y":
                        if inv_vh == 1:
                            uv_width = max(uv_x) - min(uv_x)
                        else:
                            uv_width = max(uv_y) - min(uv_y)
                    elif yz_move == "z":
                        if inv_vh == 1:
                            uv_width = max(uv_y) - min(uv_y)
                        else:
                            uv_width = max(uv_x) - min(uv_x)

                shift_uv_x = uv_x[-1:] + uv_x[:-1]
                shift_uv_y = uv_y[-1:] + uv_y[:-1]
                for vert, s_x, s_y in zip(bm.faces[0].loops, shift_uv_x,
                                          shift_uv_y):
                    #uvは一番最初（w=1）にuvの形を決めて後は縦か横に移動していく
                    #inv_vh == 1のとき反転しない
                    if w == 1:
                        uv_vert = vert[uv_layer].uv
                        if yz_move == "y":
                            if inv_vh == 1:
                                uv_s = (uv_vert[0] - min(uv_x)) / mai
                                uv_adj = min(uv_x)
                                vert[uv_layer].uv[0] = uv_s + uv_adj
                            else:
                                if z_min_no % 2 == 0:
                                    #時計回しに横に回転
                                    vert[uv_layer].uv[0] = s_y
                                    vert[uv_layer].uv[1] = (
                                        s_x - min(uv_x)) / mai + min(uv_x)
                                else:
                                    vert[uv_layer].uv[0] = s_x
                                    #vert[uv_layer].uv[1] = (s_y - min(uv_y)) / mai + min(uv_y) + (max(uv_y) - min(uv_y)) / mai
                                    vert[uv_layer].uv[1] = (
                                        s_y - min(uv_y)) / mai + min(uv_y) + (
                                            max(uv_y) - min(uv_y)) / mai

                        elif yz_move == "z":
                            if inv_vh == 1:
                                uv_s = (uv_vert[1] - min(uv_y)) / mai
                                uv_adj = min(uv_y) + (w - 1) * uv_width / mai
                                vert[uv_layer].uv[1] = uv_s + uv_adj
                            else:
                                if z_min_no % 2 == 0:
                                    vert[uv_layer].uv[0] = (
                                        s_y - min(uv_y)) / mai + min(uv_y)
                                    vert[uv_layer].uv[1] = max(uv_y) - s_x
                                else:
                                    vert[uv_layer].uv[0] = (
                                        s_x - min(uv_x)) / mai + min(uv_x)
                                    vert[uv_layer].uv[1] = s_y

                    else:
                        if yz_move == "y":
                            if inv_vh == 1:
                                print("vert[uv_layer].uv=",
                                      vert[uv_layer].uv[1])
                                vert[uv_layer].uv[0] = vert[uv_layer].uv[0] + (
                                    w - 1) * (uv_width / mai)
                            else:
                                vert[uv_layer].uv[1] = vert[uv_layer].uv[1] + (
                                    w - 1) * (uv_width / mai)

                        elif yz_move == "z":
                            if inv_vh == 1:
                                vert[uv_layer].uv[1] = vert[uv_layer].uv[1] + (
                                    w - 1) * (uv_width / mai)
                            else:
                                vert[uv_layer].uv[0] = vert[uv_layer].uv[0] + (
                                    w - 1) * (uv_width / mai)

                bmesh.update_edit_mesh(msh)

            bpy.ops.object.mode_set(mode='OBJECT')

            bpy.ops.object.select_all(action='DESELECT')
            plane.select_set(True)
            bpy.context.view_layer.objects.active = plane

            def waku(plane, waku_pos, v_width, h_width, t, ki_name):
                #bmshをオブジェクトモードのまま使う
                p_mat = plane.matrix_world
                bm = bmesh.new()
                bm.from_mesh(plane.data)
                bm.faces.ensure_lookup_table()
                loops = bm.faces[0].loops
                p_norm = bm.faces[0].normal

                p_len = len(loops)
                #cont = 2 ; xv=loops[cont]
                #if 1==1:

                utigawa = []
                for cont, xv in enumerate(loops):
                    currentPath = bpy.utils.script_paths()[2] + "/addons/"
                    filename = "hikide.blend"
                    path = currentPath + filename + "/"
                    bpy.ops.wm.append(directory=path + "Object/",
                                      link=False,
                                      filename=ki_name)

                    bpy.context.view_layer.objects.active = bpy.context.selected_objects[
                        0]
                    obj = bpy.context.object
                    #縦か横か？
                    print("z_min_no=", z_min_no)
                    if (xv.index + inv_vh + (z_min_no % 2 - 1)) % 2 == 0:
                        width = h_width
                        another_width = v_width
                    else:
                        width = v_width
                        another_width = h_width

                    obj.scale = [
                        obj.scale[0] * 1, obj.scale[1] * width,
                        obj.scale[2] * t[cont % 2]
                    ]

                    for v in obj.data.vertices:
                        verts = []
                        for s, vv in zip(obj.scale, v.co):
                            verts.append(s * vv)
                        v.co = verts
                    obj.scale = [1, 1, 1]

                    yv = xv.link_loop_next
                    obj.matrix_world = center_mat(xv, yv, p_mat, p_norm)

                    #オブジェクトを辺の長さに合わせる
                    scale_mat = obj_length(p_mat, xv.edge)
                    obj.data.transform(scale_mat)
                    obj.data.update()
                    #位置調節
                    #                bpy.ops.transform.translate(value=offset,
                    #                                            orient_type='LOCAL',
                    #                                            orient_matrix_type='LOCAL')
                    msh = obj.data
                    if plane.scale.z < 0:
                        obj.rotation_euler.x = obj.rotation_euler.x + pi
                        for v in msh.vertices:
                            if v.select:
                                if waku_pos == "1":
                                    v.co = [
                                        v.co.x + 0 / obj.scale[0],
                                        v.co.y + (width / 2) / obj.scale[1],
                                        v.co.z - 0 / obj.scale[2]
                                    ]
                                if waku_pos == "3":
                                    v.co = [
                                        v.co.x + 0 / obj.scale[0],
                                        v.co.y - (width / 2) / obj.scale[1],
                                        v.co.z - 0 / obj.scale[2]
                                    ]

                    else:
                        for v in msh.vertices:
                            if v.select:
                                if waku_pos == "1":
                                    v.co = [
                                        v.co.x + 0 / obj.scale[0],
                                        v.co.y + (width / 2) / obj.scale[1],
                                        v.co.z + 0 / obj.scale[2]
                                    ]
                                elif waku_pos == "3":
                                    v.co = [
                                        v.co.x + 0 / obj.scale[0],
                                        v.co.y - (width / 2) / obj.scale[1],
                                        v.co.z + 0 / obj.scale[2]
                                    ]

                    bpy.ops.object.mode_set(mode='EDIT')
                    obm = bmesh.from_edit_mesh(obj.data)
                    obm.faces.ensure_lookup_table()
                    obm.edges.ensure_lookup_table()
                    obm.verts.ensure_lookup_table()
                    #print("xv.edge.calc_length()",xv.edge.calc_length())
                    #
                    #面[0]をどれだけ回転させてどれだけ移動するか
                    #index=(id)
                    index = (0, 2)
                    n = xv.index + inv_vh + (z_min_no % 2 - 1)
                    length_adj = [0, 0]
                    rad = [0, 0]
                    zero = hasigo()
                    if waku_type == "1":
                        if waku_pos == "1":
                            zero.sw = (n % 2 == 0)
                            zero.another_width = another_width
                            zero.width = width
                        elif waku_pos == "3":
                            zero.sw = (n % 2 == 1)
                            zero.another_width = -another_width
                            zero.width = -width
                    elif waku_type == "2":
                        if waku_pos == "1":
                            zero.sw = 0
                            zero.another_width = another_width
                            zero.width = width
                        elif waku_pos == "3":
                            zero.sw = 1
                            zero.another_width = -another_width
                            zero.width = -width
                    zero.waku_type = waku_type
                    zero.cont = cont
                    zero.p_len = p_len
                    zero.angle = ca_angle(xv, p_mat)
                    zero.another_angle = ca_angle(xv.link_loop_prev, p_mat)
                    zero.other_angle = ca_angle(xv.link_loop_next, p_mat)
                    zero.is_conv = xv.is_convex
                    zero.other_is_conv = xv.link_loop_next.is_convex
                    zero.p_mat = p_mat
                    zero.another_edge = xv.link_loop_prev.edge
                    zero.edge = xv.edge
                    zero.n = 0
                    zero.tijimi()

                    one = hasigo()
                    if waku_type == "1":
                        if waku_pos == "1":
                            one.sw = (n % 2 == 0)
                            one.another_width = another_width
                            one.width = width
                        elif waku_pos == "3":
                            one.sw = (n % 2 == 1)
                            one.another_width = -another_width
                            one.width = width
                    elif waku_type == "2":
                        if waku_pos == "1":
                            one.sw = 1
                            one.another_width = another_width
                            one.width = width
                        elif waku_pos == "3":
                            one.sw = 0
                            one.another_width = another_width
                            one.width = width
                    one.waku_type = waku_type
                    one.cont = cont
                    one.p_len = p_len
                    one.angle = ca_angle(xv.link_loop_next, p_mat)
                    one.another_angle = ca_angle(
                        xv.link_loop_next.link_loop_next, p_mat)
                    one.other_angle = ca_angle(xv, p_mat)
                    one.is_conv = xv.link_loop_next.is_convex
                    one.other_is_conv = xv.link_loop_prev.is_convex
                    one.p_mat = p_mat
                    one.another_edge = xv.link_loop_next.edge
                    one.edge = xv.edge
                    one.n = 1
                    one.tijimi()

                    rad[0], length_adj[0] = zero.rad, zero.length_adj
                    rad[1], length_adj[1] = one.rad, one.length_adj

                    rad = [-rad[0], rad[1]]
                    for i, ids in enumerate(index):
                        #print("length_adj[i]",length_adj[i])
                        end_face(ids, width, length_adj[i], xv, obm, t, rad[i],
                                 n, waku_pos)

                    if cont == 0 and (z_min_no +
                                      1) % 2 == 0 and p_len % 2 == 1:
                        mat = bpy.context.object.matrix_world.copy()
                        v3 = obm.verts[3].co.copy()
                        v3[2] = 0
                        utigawa.append(mat @ obm.verts[3].co)

                    if (cont + 1) % 2 == (z_min_no + 1) % 2:
                        mat = bpy.context.object.matrix_world.copy()

                        v3 = obm.verts[3].co.copy()
                        v3[2] = 0
                        v7 = obm.verts[7].co.copy()
                        v7[2] = 0
                        utigawa.append(mat @ v3)
                        utigawa.append(mat @ v7)
                    bpy.ops.object.mode_set(mode='OBJECT')
                return utigawa

            #def waku---->end
            utigawa = waku(plane, waku_pos, v_width, h_width, t, ki_name)
            end_objs = [n for n in bpy.context.view_layer.objects]

            #襖の引き手を付ける

            if hiki == 1:
                hikide, hikide2 = set_hikide(w, z_min_no, inv_vh, yz_move,
                                             end_objs, mai, plane, plane_t)
                hiki2.append(hikide2)

            all_utigawa.append(utigawa)

            emp = bpy.data.objects.new('empty', None)
            co.objects.link(emp)
            emp.empty_display_size = 0.5
            #エンプティを矢印に
            emp.empty_display_type = 'ARROWS'

            moto_mat = emp.matrix_world
            p_mat = plane.matrix_world
            mat_emp = set_emp(plane, p_mat)
            emp.matrix_world = moto_mat @ mat_emp
            emps.append(emp)

            chil_objs = list(set(end_objs) - set(start_objs))
            plane.parent = emp
            if hiki == 1:
                hikide.parent = emp
                hikide2.parent = emp
                hikide.matrix_parent_inverse = emp.matrix_world.inverted()
                hikide2.matrix_parent_inverse = emp.matrix_world.inverted()
            plane.matrix_parent_inverse = emp.matrix_world.inverted()
            ####################################################

            #それぞれの板ごとのエンプティ
            for o in chil_objs:
                o.parent = emp
                o.matrix_parent_inverse = emp.matrix_world.inverted()

        if soto == True:
            #外枠をつける
            co = bpy.context.collection
            mat = soto_waku.matrix_world.copy()
            soto_waku = bpy.data.objects.new(name='aaa',
                                             object_data=soto_waku.data)
            co.objects.link(soto_waku)
            soto_waku.matrix_world = mat

            bpy.ops.object.select_all(action='DESELECT')
            soto_waku.select_set(True)
            bpy.context.view_layer.objects.active = soto_waku
            start_objs = [n for n in bpy.context.view_layer.objects]
            waku(soto_waku, "3", soto_v_width, soto_h_width, soto_t,
                 soto_ki_name)
            bpy.data.objects.remove(soto_waku)
            end_objs = [n for n in bpy.context.view_layer.objects]
            chil_objs = list(set(end_objs) - set(start_objs))
            if mai > 1:
                val = [0, 0, max(t) / 2]

                for o in chil_objs:
                    va = [d / s for d, s in zip(val, o.scale)]
                    mat = o.matrix_world.copy()
                    o.location = mat @ Vector(va)
        #全部の親になるエンプティ
        if mai > 1 or soto == True:

            emp = bpy.data.objects.new('empty', None)
            co.objects.link(emp)
            emp.empty_display_size = 1
            #エンプティを矢印に
            emp.empty_display_type = 'ARROWS'
            sum = Vector((0, 0, 0))
            for e in emps:
                sum = sum + e.location
            emp_p = sum / len(emps)

            e_mat = emps[0].matrix_world.copy()
            if mai == 1:
                e_mat_z = emp_p[2]
            else:
                e_mat_z = (emps[0].location.z + emps[1].location.z) / 2
            e_mat[0][3] = emp_p[0]
            e_mat[1][3] = emp_p[1]
            e_mat[2][3] = e_mat_z

            emp.matrix_world = e_mat
            for e in emps:
                e.parent = emp
                e.matrix_parent_inverse = emp.matrix_world.inverted()

            for o in chil_objs:
                o.parent = emp
                o.matrix_parent_inverse = emp.matrix_world.inverted()

        #平面の頂点を枠に合わせて移動する

        for n, (a_plane, a_utigawa) in enumerate(zip(all_planes, all_utigawa)):

            p_len = len(a_plane.data.loops)
            if (z_min_no + inv_vh) % 2 == 0 and p_len % 2 == 1:
                start = 1
            elif 1 == (z_min_no + inv_vh) % 2:
                start = 0
            else:
                start = 0

            for i, l in enumerate(a_plane.data.loops, start):

                a_plane.data.vertices[
                    l.vertex_index].co = a_plane.matrix_world.inverted(
                    ) @ a_utigawa[i - 1]
            new_obj = a_plane.copy()
            new_obj.data = a_plane.data.copy()
            co.objects.link(new_obj)
            a_plane.location = norm_move(a_plane, plane_t / 2)
            new_obj.location = norm_move(new_obj, -plane_t / 2)
            if hiki == 1:
                #裏面の引き手のモディファイアをつける
                print("n=", n)
                print(hiki2)
                set_hikide2(new_obj, hiki2[n])

        return {'FINISHED'}


def menu_fn(self, context):
    self.layout.separator()
    self.layout.operator(FUSUMA_OT_CreateObject.bl_idname)


# Blenderに登録するクラス
classes = [
    FUSUMA_OT_CreateObject,
]


# アドオン有効化時の処理
def register():
    for c in classes:
        bpy.utils.register_class(c)
    bpy.types.VIEW3D_MT_mesh_add.append(menu_fn)
    print("fusuma: アドオン『fusuma』が有効化されました。")


# アドオン無効化時の処理
def unregister():
    bpy.types.VIEW3D_MT_mesh_add.remove(menu_fn)
    for c in classes:
        bpy.utils.unregister_class(c)
    print("fusuma: アドオン『fusuma』が無効化されました。")


# メイン処理
if __name__ == "__main__":
    register()
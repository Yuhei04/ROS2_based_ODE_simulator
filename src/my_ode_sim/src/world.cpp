#include "my_ode_sim/world.hpp"
#include "my_ode_sim/robot.hpp"
#include "my_ode_sim/lidar.hpp"

#include <drawstuff/drawstuff.h>

#ifdef dDOUBLE
#define dsDrawBox      dsDrawBoxD
#define dsDrawCylinder dsDrawCylinderD
#define dsDrawSphere   dsDrawSphereD
#endif

namespace world
{
    static WorldType g_world_type = WorldType::SIMPLE;
    static void initSimpleWorld();
    static void initCorridorWorld();
    static void drawSimpleWorld();
    static void drawCorridorWorld();


    // ===== 廊下パラメータ（world 全体で共有）=====
    static const dReal CORRIDOR_LEN = 20.0;  // 20m
    static const dReal CORRIDOR_W   = 1.8;   // 廊下幅
    static const dReal WALL_THICK  = 0.2;
    static const dReal WALL_H      = 2.0;

    // ===== simple world 用 ドア凹み =====
    static const dReal SIMPLE_WALL_LEN   = 10.0;
    static const dReal SIMPLE_WALL_W     = 1.8;

    static const dReal SIMPLE_DOOR_X     = 5.0;   // 凹み中心
    static const dReal SIMPLE_DOOR_WIDTH = 1.0;
    static const dReal SIMPLE_DOOR_DEPTH = 0.6;

    // ===== simple world 用 壁分割の長さ（描画でも使う）=====
    static dReal simple_len1 = 0.0;
    static dReal simple_len2 = 0.0;

    // simple world 用 inner wall
    static dGeomID wall1_inner = nullptr;

    // --- このモジュールだけで持つ ODE ハンドルたち ---
    static dWorldID      g_world;
    static dSpaceID      g_space;
    static dJointGroupID g_contactgroup;

    // 壁
    static dGeomID wall1, wall2, wall3;
    
    // 障害物
    static dGeomID obs1 = nullptr;

    // --- 近接コールバック（Ray と通常接触） ---
    static void nearCallback(void* /*data*/, dGeomID o1, dGeomID o2)
    {
        const int N = 10;
        dContact contact[N];

        dBodyID b1 = dGeomGetBody(o1);
        dBodyID b2 = dGeomGetBody(o2);

        int class1 = dGeomGetClass(o1);
        int class2 = dGeomGetClass(o2);

        // --- ① LiDAR Ray の場合 ---
        if (class1 == dRayClass || class2 == dRayClass) {
            dGeomID rayGeom = (class1 == dRayClass) ? o1 : o2;
            dGeomID objGeom = (rayGeom == o1) ? o2 : o1;

            // 自分のロボットに当たった Ray は無視
            if (robot::isRobotGeom(objGeom)) {
                return;
            }

            int n = dCollide(o1, o2, N, &contact[0].geom, sizeof(dContact));
            if (n <= 0) return;

            dReal minDepth = 1e9;
            for (int i = 0; i < n; ++i) {
                if (contact[i].geom.depth < minDepth) {
                    minDepth = contact[i].geom.depth;
                }
            }

            // LiDAR 側に「この Ray が minDepth 先でヒットした」と伝える
            lidar::onRayHit(rayGeom, minDepth);
            return;
        }

        // --- ② 通常の剛体同士の接触 ---

        // ロボット内部同士（ジョイントで接続）は無視
        if (b1 && b2 && dAreConnectedExcluding(b1, b2, dJointTypeContact)) return;

        int n = dCollide(o1, o2, N, &contact[0].geom, sizeof(dContact));
        for (int i = 0; i < n; ++i) {
            contact[i].surface.mode =
                dContactSlip1 | dContactSlip2 |
                dContactSoftERP | dContactSoftCFM | dContactApprox1;

            contact[i].surface.mu        = dInfinity;
            contact[i].surface.slip1     = 0.001;
            contact[i].surface.slip2     = 0.001;
            contact[i].surface.soft_erp  = 1.0;
            contact[i].surface.soft_cfm  = 1e-5;

            dJointID c = dJointCreateContact(g_world, g_contactgroup, &contact[i]);
            dJointAttach(c,
                dGeomGetBody(contact[i].geom.g1),
                dGeomGetBody(contact[i].geom.g2));
        }
    }

    // --- 初期化 ---
    void init(WorldType type)
    {
        g_world_type = type;

        dInitODE();

        g_world = dWorldCreate();
        g_space = dHashSpaceCreate(0);
        g_contactgroup = dJointGroupCreate(0);

        dWorldSetGravity(g_world, 0, 0, -9.81);

        dWorldSetLinearDamping(g_world, 0.1);
        dWorldSetAngularDamping(g_world, 0.1);

        // 地面
        dCreatePlane(g_space, 0, 0, 1, 0);
        
        // world切り替え
        if (g_world_type == WorldType::SIMPLE) {
            initSimpleWorld();
        } else {
            initCorridorWorld();
        }
    }

    static void initSimpleWorld()
    {
        // CorridorWorld と同じ座標系：
        // X: 廊下方向, Y: 左右（+が左壁）, Z: 上

        // 左壁の凹み位置（X方向）
        // 凹み区間は「壁を置かない」
        simple_len1 = SIMPLE_DOOR_X - SIMPLE_DOOR_WIDTH / 2.0;
        simple_len2 = SIMPLE_WALL_LEN - (SIMPLE_DOOR_X + SIMPLE_DOOR_WIDTH / 2.0);

        // 念のため（負になると描画も衝突もおかしくなる）
        if (simple_len1 < 0) simple_len1 = 0;
        if (simple_len2 < 0) simple_len2 = 0;

        // 左壁（前半）
        if (simple_len1 > 0.001) {
            wall1 = dCreateBox(g_space, simple_len1, WALL_THICK, WALL_H);
            dGeomSetPosition(
                wall1,
                simple_len1 / 2.0,            // X中心
                +SIMPLE_WALL_W / 2.0,         // 左壁（+Y）
                WALL_H / 2.0
            );
        } else {
            wall1 = nullptr;
        }

        // 左壁（後半）
        if (simple_len2 > 0.001) {
            wall2 = dCreateBox(g_space, simple_len2, WALL_THICK, WALL_H);
            dGeomSetPosition(
                wall2,
                (SIMPLE_DOOR_X + SIMPLE_DOOR_WIDTH / 2.0) + simple_len2 / 2.0,
                +SIMPLE_WALL_W / 2.0,
                WALL_H / 2.0
            );
        } else {
            wall2 = nullptr;
        }

        // 右壁（一本）
        wall3 = dCreateBox(g_space, SIMPLE_WALL_LEN, WALL_THICK, WALL_H);
        dGeomSetPosition(
            wall3,
            SIMPLE_WALL_LEN / 2.0,
            -SIMPLE_WALL_W / 2.0,            // 右壁（-Y）
            WALL_H / 2.0
        );

        // simple world では inner wall は不要（使わないなら nullptr のままでOK）
        wall1_inner = nullptr;

        // 障害物なし
        obs1 = nullptr;
    }



    static void initCorridorWorld()
    {
        // 左壁
        wall1 = dCreateBox(g_space, CORRIDOR_LEN, WALL_THICK, WALL_H);
        dGeomSetPosition(wall1,
            CORRIDOR_LEN / 2.0,
            +CORRIDOR_W / 2.0,
            WALL_H / 2.0);

        // 右壁
        wall2 = dCreateBox(g_space, CORRIDOR_LEN, WALL_THICK, WALL_H);
        dGeomSetPosition(wall2,
            CORRIDOR_LEN / 2.0,
            -CORRIDOR_W / 2.0,
            WALL_H / 2.0);

        // 終端壁
        wall3 = dCreateBox(g_space,
            WALL_THICK, CORRIDOR_W, WALL_H);
        dGeomSetPosition(wall3,
            CORRIDOR_LEN + 0.1,
            0.0,
            WALL_H / 2.0);

        // 下駄箱
        obs1 = dCreateBox(g_space, 0.6, 0.4, 1.8);
        dGeomSetPosition(obs1,
            12.0,
            +CORRIDOR_W / 2.0 - 0.2,
            0.9);
    }


    // --- 1ステップ進める ---
    void step(dReal step_size)
    {
        dSpaceCollide(g_space, 0, &nearCallback);
        dWorldStep(g_world, step_size);
        dJointGroupEmpty(g_contactgroup);
    }

    // --- 描画（壁・障害物） ---
    void draw()
    {
        if (g_world_type == WorldType::SIMPLE) {
            drawSimpleWorld();
        } else {
            drawCorridorWorld();
        }
    }

    static void drawSimpleWorld()
    {
        static bool once = true;
        if (once) {
            printf("[drawSimpleWorld] called\n");
            printf("simple_len1=%.3f simple_len2=%.3f\n", simple_len1, simple_len2);
            once = false;
        }

        dsSetColor(1, 1, 1);

        if (wall1) {
            dReal s1[3] = {simple_len1, WALL_THICK, WALL_H};
            dsDrawBox(dGeomGetPosition(wall1), dGeomGetRotation(wall1), s1);
        }

        if (wall2) {
            dReal s2[3] = {simple_len2, WALL_THICK, WALL_H};
            dsDrawBox(dGeomGetPosition(wall2), dGeomGetRotation(wall2), s2);
        }

        if (wall3) {
            dReal s3[3] = {SIMPLE_WALL_LEN, WALL_THICK, WALL_H};
            dsDrawBox(dGeomGetPosition(wall3), dGeomGetRotation(wall3), s3);
        }
    }

    static void drawCorridorWorld()
    {
        dReal s[3] = {CORRIDOR_LEN, WALL_THICK, WALL_H};
        dsDrawBox(dGeomGetPosition(wall1), dGeomGetRotation(wall1), s);
        dsDrawBox(dGeomGetPosition(wall2), dGeomGetRotation(wall2), s);

        dReal e[3] = {WALL_THICK, CORRIDOR_W, WALL_H};
        dsDrawBox(dGeomGetPosition(wall3), dGeomGetRotation(wall3), e);

        dReal o[3] = {0.6, 0.4, 1.8};
        dsDrawBox(dGeomGetPosition(obs1), dGeomGetRotation(obs1), o);
    }


    // --- 終了処理 ---
    void shutdown()
    {
        dJointGroupDestroy(g_contactgroup);
        dSpaceDestroy(g_space);
        dWorldDestroy(g_world);
        dCloseODE();
    }

    dWorldID getWorld() { return g_world; }
    dSpaceID getSpace() { return g_space; }

} // namespace world
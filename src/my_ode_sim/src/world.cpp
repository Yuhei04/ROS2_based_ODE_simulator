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
    // --- このモジュールだけで持つ ODE ハンドルたち ---
    static dWorldID      g_world;
    static dSpaceID      g_space;
    static dJointGroupID g_contactgroup;

    // 壁 & 障害物
    static dGeomID wall1, wall2, wall3, wall4;
    static dGeomID obs1,  obs2;

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

            contact[i].surface.mu        = 1.0;
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
    void init()
    {
        dInitODE();

        g_world = dWorldCreate();
        g_space = dHashSpaceCreate(0);
        g_contactgroup = dJointGroupCreate(0);

        dWorldSetGravity(g_world, 0, 0, -9.81);

        // 地面
        dCreatePlane(g_space, 0, 0, 1, 0);

        // --- 壁 ---
        wall1 = dCreateBox(g_space, 0.2, 10.0, 2.0);
        dGeomSetPosition(wall1, -5.0, 0.0, 1.0);

        wall2 = dCreateBox(g_space, 0.2, 10.0, 2.0);
        dGeomSetPosition(wall2,  5.0, 0.0, 1.0);

        wall3 = dCreateBox(g_space, 10.0, 0.2, 2.0);
        dGeomSetPosition(wall3, 0.0, -5.0, 1.0);

        wall4 = dCreateBox(g_space, 10.0, 0.2, 2.0);
        dGeomSetPosition(wall4, 0.0,  5.0, 1.0);

        // --- 障害物 ---
        obs1 = dCreateBox(g_space, 1.0, 1.0, 1.5);
        dGeomSetPosition(obs1, 0.0, 0.0, 0.75);

        obs2 = dCreateBox(g_space, 1.5, 0.3, 1.0);
        dGeomSetPosition(obs2, 2.0, 1.0, 0.5);
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
        if (wall1) {
            dReal s1[3] = {0.2, 10.0, 2.0};
            dsDrawBox(dGeomGetPosition(wall1), dGeomGetRotation(wall1), s1);
        }
        if (wall2) {
            dReal s2[3] = {0.2, 10.0, 2.0};
            dsDrawBox(dGeomGetPosition(wall2), dGeomGetRotation(wall2), s2);
        }
        if (wall3) {
            dReal s3[3] = {10.0, 0.2, 2.0};
            dsDrawBox(dGeomGetPosition(wall3), dGeomGetRotation(wall3), s3);
        }
        if (wall4) {
            dReal s4[3] = {10.0, 0.2, 2.0};
            dsDrawBox(dGeomGetPosition(wall4), dGeomGetRotation(wall4), s4);
        }
        if (obs1) {
            dReal o1[3] = {1.0, 1.0, 1.5};
            dsDrawBox(dGeomGetPosition(obs1), dGeomGetRotation(obs1), o1);
        }
        if (obs2) {
            dReal o2[3] = {1.5, 0.3, 1.0};
            dsDrawBox(dGeomGetPosition(obs2), dGeomGetRotation(obs2), o2);
        }
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
#include "my_ode_sim/robot.hpp"

namespace robot
{
    // --- パラメータ定義（元 main.cpp の値） ---
    const dReal CHASSIS_L   = 0.6;
    const dReal CHASSIS_W   = 0.4;
    const dReal CHASSIS_H   = 0.2;
    const dReal CHASSIS_X0  = 0.5;  //ロボット初期位置
    const dReal CHASSIS_Y0  = 0.4;

    const dReal WHEEL_RADIUS = 0.12;
    const dReal WHEEL_WIDTH  = 0.06;

    const dReal TRACK          = 0.5;
    const dReal WHEEL_OFFSET_Y = TRACK / 2.0;
    const dReal WHEEL_OFFSET_X = CHASSIS_L / 2.0 - 0.05;
    const dReal CASTER_OFFSET_X = CHASSIS_L / 2.0 - 0.05;
    const dReal CASTER_RADIUS   = 0.05;

    // --- このファイル内だけで管理するグローバル（static） ---
    static dBodyID  chassisBody;
    static dGeomID  chassisGeom;

    static dBodyID  leftWheelBody, rightWheelBody;
    static dGeomID  leftWheelGeom, rightWheelGeom;
    static dJointID leftJoint, rightJoint;

    static dBodyID  casterBody;
    static dGeomID  casterGeom;
    static dJointID casterJoint;

    void create(dWorldID world, dSpaceID space)
    {
        // --- シャーシ ---
        chassisBody = dBodyCreate(world);
        dMass m;
        dMassSetBox(&m, 1.0, CHASSIS_L, CHASSIS_W, CHASSIS_H);
        dMassAdjust(&m, 5.0);
        dBodySetMass(chassisBody, &m);
        dBodySetPosition(
            chassisBody,
            CHASSIS_X0,
            CHASSIS_Y0,
            CHASSIS_H / 2.0 + 0.02
        );

        chassisGeom = dCreateBox(space, CHASSIS_L, CHASSIS_W, CHASSIS_H);
        dGeomSetBody(chassisGeom, chassisBody);

        // --- 車輪 ---
        dMass mw;
        dMassSetCylinder(&mw, 1.0, 1, WHEEL_RADIUS, WHEEL_WIDTH);
        dMassAdjust(&mw, 1.0);

        // 左後輪
        leftWheelBody = dBodyCreate(world);
        dBodySetMass(leftWheelBody, &mw);
        dBodySetPosition(
            leftWheelBody,
            CHASSIS_X0 - WHEEL_OFFSET_X,
            CHASSIS_Y0 + WHEEL_OFFSET_Y,
            WHEEL_RADIUS
        );

        leftWheelGeom = dCreateCylinder(space, WHEEL_RADIUS, WHEEL_WIDTH);
        dGeomSetBody(leftWheelGeom, leftWheelBody);

        // 右後輪
        rightWheelBody = dBodyCreate(world);
        dBodySetMass(rightWheelBody, &mw);
        dBodySetPosition(
            rightWheelBody,
            CHASSIS_X0 - WHEEL_OFFSET_X,
            CHASSIS_Y0 - WHEEL_OFFSET_Y,
            WHEEL_RADIUS
        );

        rightWheelGeom = dCreateCylinder(space, WHEEL_RADIUS, WHEEL_WIDTH);
        dGeomSetBody(rightWheelGeom, rightWheelBody);

        // タイヤの向きをそろえる（X軸まわり90°回転）
        {
            dMatrix3 R;
            dRFromAxisAndAngle(R, 1, 0, 0, M_PI / 2.0);
            dBodySetRotation(leftWheelBody, R);
            dBodySetRotation(rightWheelBody, R);
        }

        // --- シャーシと車輪のジョイント ---
        leftJoint = dJointCreateHinge(world, 0);
        dJointAttach(leftJoint, chassisBody, leftWheelBody);
        dJointSetHingeAnchor(
            leftJoint,
            CHASSIS_X0 - WHEEL_OFFSET_X,
            CHASSIS_Y0 + WHEEL_OFFSET_Y,
            WHEEL_RADIUS
        );
        dJointSetHingeAxis(leftJoint, 0, 1, 0);

        rightJoint = dJointCreateHinge(world, 0);
        dJointAttach(rightJoint, chassisBody, rightWheelBody);
        dJointSetHingeAnchor(
            rightJoint,
            CHASSIS_X0 - WHEEL_OFFSET_X,
            CHASSIS_Y0 - WHEEL_OFFSET_Y,
            WHEEL_RADIUS
        );
        dJointSetHingeAxis(rightJoint, 0, 1, 0);

        // --- キャスタ（前方） ---
        {
            dMass mc;
            casterBody = dBodyCreate(world);
            dMassSetSphere(&mc, 1.0, CASTER_RADIUS);
            dMassAdjust(&mc, 0.5);
            dBodySetMass(casterBody, &mc);

            dBodySetPosition(
                casterBody,
                CHASSIS_X0 + CASTER_OFFSET_X,
                CHASSIS_Y0,
                CASTER_RADIUS
            );

            casterGeom = dCreateSphere(space, CASTER_RADIUS);
            dGeomSetBody(casterGeom, casterBody);

            casterJoint = dJointCreateBall(world, 0);
            dJointAttach(casterJoint, chassisBody, casterBody);
            dJointSetBallAnchor(
                casterJoint,
                CHASSIS_X0 + CASTER_OFFSET_X,
                CHASSIS_Y0,
                CASTER_RADIUS
            );
        }
    }

    // --- アクセサ ---
    dBodyID getChassisBody()   { return chassisBody; }
    dBodyID getLeftWheelBody() { return leftWheelBody; }
    dBodyID getRightWheelBody(){ return rightWheelBody; }
    dBodyID getCasterBody()    { return casterBody; }

    dJointID getLeftJoint()    { return leftJoint; }
    dJointID getRightJoint()   { return rightJoint; }

    bool isRobotGeom(dGeomID g)
    {
        return (g == chassisGeom ||
                g == leftWheelGeom ||
                g == rightWheelGeom ||
                g == casterGeom);
    }
}
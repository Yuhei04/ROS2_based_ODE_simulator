#pragma once

#include <ode/ode.h>

namespace world
{
    // ===== world の種類 =====
    enum class WorldType {
        SIMPLE,     // いままでの四角い壁 world
        CORRIDOR    // 廊下 + 下駄箱 world
    };

    // ODE world / space / contactgroup の初期化
    // world_type によって環境を切り替える
    void init(WorldType world_type);

    // 1 シミュレーションステップ進める（dSpaceCollide + dWorldStep）
    void step(dReal step_size);

    // 壁や障害物など「環境」の描画
    void draw();

    // 終了処理（joint group / space / world / dCloseODE）
    void shutdown();

    // 他のモジュールから world / space にアクセスするとき用
    dWorldID getWorld();
    dSpaceID getSpace();
}

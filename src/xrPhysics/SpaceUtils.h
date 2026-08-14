#pragma once

#include "xrPhysicsCore/IPhysicsCore.h"

IC void spatialParsFromShape(PhysicsShapeHandle shape, Fvector& center, Fvector& AABB, float& radius)
{
    if (!shape) 
    {
        center.set(0.f, 0.f, 0.f);
        AABB.set(0.f, 0.f, 0.f);
        radius = 0.f;
        return;
    }

    // Получаем центр и half-extents (размеры от центра до краев) напрямую из ядра
    GetPhysicsCore()->GetCDBModelBounds(shape, center, AABB);
    
    // Радиус описывающей сферы (по максимальному габариту)
    radius = _max(AABB.x, _max(AABB.y, AABB.z));
}

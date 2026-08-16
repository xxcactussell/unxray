#pragma once
#include <Jolt/Jolt.h>
#include <Jolt/Physics/Collision/BroadPhase/BroadPhaseLayer.h>
#include <Jolt/Physics/Collision/ObjectLayer.h>

namespace Layers {
    static constexpr JPH::ObjectLayer NON_MOVING = 0;
    static constexpr JPH::ObjectLayer MOVING = 1;
    static constexpr JPH::ObjectLayer RAGDOLL = 2;
    static constexpr JPH::uint NUM_LAYERS = 3;
}

namespace BroadPhaseLayers {
    static constexpr JPH::BroadPhaseLayer NON_MOVING(0);
    static constexpr JPH::BroadPhaseLayer MOVING(1);
    static constexpr JPH::BroadPhaseLayer RAGDOLL(2);
    static constexpr JPH::uint NUM_LAYERS(3);
}


class BPLayerInterfaceImpl final : public JPH::BroadPhaseLayerInterface {
public:
    BPLayerInterfaceImpl() {
        // Создаем массив маппинга
        m_object_to_broad_phase[Layers::NON_MOVING] = BroadPhaseLayers::NON_MOVING;
        m_object_to_broad_phase[Layers::MOVING] = BroadPhaseLayers::MOVING;
        m_object_to_broad_phase[Layers::RAGDOLL] = BroadPhaseLayers::RAGDOLL;
    }

    virtual JPH::uint GetNumBroadPhaseLayers() const override {
        return BroadPhaseLayers::NUM_LAYERS;
    }

    virtual JPH::BroadPhaseLayer GetBroadPhaseLayer(JPH::ObjectLayer inLayer) const override {
        JPH_ASSERT(inLayer < Layers::NUM_LAYERS);
        return m_object_to_broad_phase[inLayer];
    }

#if defined(JPH_EXTERNAL_PROFILE) || defined(JPH_PROFILE_ENABLED)
    virtual const char* GetBroadPhaseLayerName(JPH::BroadPhaseLayer inLayer) const override {
        switch ((JPH::BroadPhaseLayer::Type)inLayer) {
            case (JPH::BroadPhaseLayer::Type)BroadPhaseLayers::NON_MOVING: return "NON_MOVING";
            case (JPH::BroadPhaseLayer::Type)BroadPhaseLayers::MOVING:     return "MOVING";
            case (JPH::BroadPhaseLayer::Type)BroadPhaseLayers::RAGDOLL:    return "RAGDOLL";
            default:                                                       JPH_ASSERT(false); return "INVALID";
        }
    }
#endif

private:
    JPH::BroadPhaseLayer m_object_to_broad_phase[Layers::NUM_LAYERS];
};


class ObjectVsBroadPhaseLayerFilterImpl : public JPH::ObjectVsBroadPhaseLayerFilter {
public:
    virtual bool ShouldCollide(JPH::ObjectLayer inLayer1, JPH::BroadPhaseLayer inLayer2) const override {
        switch (inLayer1) {
            case Layers::NON_MOVING:
                // Статика сталкивается ТОЛЬКО с динамикой и рэгдоллами
                return inLayer2 == BroadPhaseLayers::MOVING || inLayer2 == BroadPhaseLayers::RAGDOLL;
            case Layers::MOVING:
                // Динамика сталкивается со всем
                return true;
            case Layers::RAGDOLL:
                // Рэгдоллы сталкиваются со всем
                return true;
            default:
                JPH_ASSERT(false);
                return false;
        }
    }
};

class ObjectLayerPairFilterImpl : public JPH::ObjectLayerPairFilter {
public:
    virtual bool ShouldCollide(JPH::ObjectLayer inObject1, JPH::ObjectLayer inObject2) const override {
        switch (inObject1) {
            case Layers::NON_MOVING:
                // Статика сталкивается ТОЛЬКО с динамикой и рэгдоллами
                return inObject2 == Layers::MOVING || inObject2 == Layers::RAGDOLL; 
            case Layers::MOVING:
                // Динамика сталкивается со всем
                return true;
            case Layers::RAGDOLL:
                // Рэгдоллы сталкиваются со всем
                return true;
            default:
                JPH_ASSERT(false);
                return false;
        }
    }
};

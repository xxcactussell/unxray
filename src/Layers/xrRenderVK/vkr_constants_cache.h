#pragma once

namespace xray::render::RENDER_NAMESPACE
{
class ECORE_API R_constants
{
private:
    alignas(16) u8 m_storage[65536]{};
    u32 m_version{ 1 };

    ICF void set(R_constant* C, R_constant_load& L, const Fmatrix& A)
    {
        if (L.index == u16(-1)) return;
        m_version++;
        u8* ptr = m_storage + L.index;
        switch (L.cls)
        {
        case RC_2x4:
            if (L.index + 2 * sizeof(Fvector4) <= sizeof(m_storage))
            {
                Fvector4* it = reinterpret_cast<Fvector4*>(ptr);
                it[0].set(A._11, A._21, A._31, A._41);
                it[1].set(A._12, A._22, A._32, A._42);
            }
            break;
        case RC_3x4:
            if (L.index + 3 * sizeof(Fvector4) <= sizeof(m_storage))
            {
                Fvector4* it = reinterpret_cast<Fvector4*>(ptr);
                it[0].set(A._11, A._21, A._31, A._41);
                it[1].set(A._12, A._22, A._32, A._42);
                it[2].set(A._13, A._23, A._33, A._43);
            }
            break;
        case RC_4x4:
        default:
            if (L.index + 4 * sizeof(Fvector4) <= sizeof(m_storage))
            {
                Fvector4* it = reinterpret_cast<Fvector4*>(ptr);
                it[0].set(A._11, A._12, A._13, A._14);
                it[1].set(A._21, A._22, A._23, A._24);
                it[2].set(A._31, A._32, A._33, A._34);
                it[3].set(A._41, A._42, A._43, A._44);
            }
            break;
        }
    }

    ICF void seta(R_constant* C, R_constant_load& L, u32 e, const Fmatrix& A)
    {
        if (L.index == u16(-1)) return;
        m_version++;
        u32 stride = 4 * sizeof(Fvector4);
        if (L.cls == RC_2x4) stride = 2 * sizeof(Fvector4);
        else if (L.cls == RC_3x4) stride = 3 * sizeof(Fvector4);
        u32 offset = L.index + stride * e;
        if (offset + stride <= sizeof(m_storage))
        {
            Fvector4* it = reinterpret_cast<Fvector4*>(m_storage + offset);
            if (L.cls == RC_2x4)
            {
                it[0].set(A._11, A._21, A._31, A._41);
                it[1].set(A._12, A._22, A._32, A._42);
            }
            else if (L.cls == RC_3x4)
            {
                it[0].set(A._11, A._21, A._31, A._41);
                it[1].set(A._12, A._22, A._32, A._42);
                it[2].set(A._13, A._23, A._33, A._43);
            }
            else
            {
                it[0].set(A._11, A._12, A._13, A._14);
                it[1].set(A._21, A._22, A._23, A._24);
                it[2].set(A._31, A._32, A._33, A._34);
                it[3].set(A._41, A._42, A._43, A._44);
            }
        }
    }

    ICF void set(R_constant* C, R_constant_load& L, const Fvector4& A)
    {
        if (L.index != u16(-1) && (L.index + sizeof(A)) <= sizeof(m_storage))
        {
            m_version++;
            Fvector4* it = reinterpret_cast<Fvector4*>(m_storage + L.index);
            *it = A;
        }
    }

    ICF void seta(R_constant* C, R_constant_load& L, u32 e, const Fvector4& A)
    {
        if (L.index == u16(-1)) return;
        u32 offset = L.index + sizeof(Fvector4) * e;
        if (offset + sizeof(Fvector4) <= sizeof(m_storage))
        {
            m_version++;
            Fvector4* it = reinterpret_cast<Fvector4*>(m_storage + offset);
            *it = A;
        }
    }

    ICF void set(R_constant* C, R_constant_load& L, float x, float y, float z, float w)
    {
        Fvector4 v{ x, y, z, w };
        set(C, L, v);
    }

    ICF void set(R_constant* C, R_constant_load& L, float A)
    {
        if (L.index != u16(-1) && (L.index + sizeof(A)) <= sizeof(m_storage))
        {
            m_version++;
            CopyMemory(m_storage + L.index, &A, sizeof(A));
        }
    }

    ICF void set(R_constant* C, R_constant_load& L, int A)
    {
        if (L.index != u16(-1) && (L.index + sizeof(A)) <= sizeof(m_storage))
        {
            m_version++;
            CopyMemory(m_storage + L.index, &A, sizeof(A));
        }
    }

public:
    const void* GetData() const { return m_storage; }
    size_t GetSize() const { return sizeof(m_storage); }
    u32 GetVersion() const { return m_version; }

    ICF void set(R_constant* C, const Fmatrix& A)
    {
        if (C->destination & RC_dest_pixel) { set(C, C->ps, A); }
        if (C->destination & RC_dest_vertex) { set(C, C->vs, A); }
        if (C->destination & RC_dest_geometry) { set(C, C->gs, A); }
    }

    ICF void set(R_constant* C, const Fvector4& A)
    {
        if (C->destination & RC_dest_pixel) { set(C, C->ps, A); }
        if (C->destination & RC_dest_vertex) { set(C, C->vs, A); }
        if (C->destination & RC_dest_geometry) { set(C, C->gs, A); }
    }

    ICF void set(R_constant* C, float x, float y, float z, float w)
    {
        if (C->destination & RC_dest_pixel) { set(C, C->ps, x, y, z, w); }
        if (C->destination & RC_dest_vertex) { set(C, C->vs, x, y, z, w); }
        if (C->destination & RC_dest_geometry) { set(C, C->gs, x, y, z, w); }
    }

    ICF void set(R_constant* C, float A)
    {
        if (C->destination & RC_dest_pixel) { set(C, C->ps, A); }
        if (C->destination & RC_dest_vertex) { set(C, C->vs, A); }
        if (C->destination & RC_dest_geometry) { set(C, C->gs, A); }
    }

    ICF void set(R_constant* C, int A)
    {
        if (C->destination & RC_dest_pixel) { set(C, C->ps, A); }
        if (C->destination & RC_dest_vertex) { set(C, C->vs, A); }
        if (C->destination & RC_dest_geometry) { set(C, C->gs, A); }
    }

    ICF void seta(R_constant* C, u32 e, const Fmatrix& A)
    {
        if (C->destination & RC_dest_pixel) { seta(C, C->ps, e, A); }
        if (C->destination & RC_dest_vertex) { seta(C, C->vs, e, A); }
        if (C->destination & RC_dest_geometry) { seta(C, C->gs, e, A); }
    }

    ICF void seta(R_constant* C, u32 e, const Fvector4& A)
    {
        if (C->destination & RC_dest_pixel) { seta(C, C->ps, e, A); }
        if (C->destination & RC_dest_vertex) { seta(C, C->vs, e, A); }
        if (C->destination & RC_dest_geometry) { seta(C, C->gs, e, A); }
    }

    ICF void seta(R_constant* C, u32 e, float x, float y, float z, float w)
    {
        Fvector4 v{ x, y, z, w };
        seta(C, e, v);
    }
};
} // namespace xray::render::RENDER_NAMESPACE

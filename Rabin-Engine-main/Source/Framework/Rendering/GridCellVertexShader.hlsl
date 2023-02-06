//#pragma pack_matrix(row_major)

cbuffer Constants
{
    float4x4 view;
    float4x4 proj;
    float scale;
    float unused0;
    float unused1;
    float unused2;
};

struct InstancedVertex
{
    float3 position : POSITION;
    float3 normal : NORMAL;
    float3 iPosition : I_POSITION;
    float4 iColor : I_COLOR;
};

struct Output
{
    float4 color : COLOR;
    float4 position : SV_POSITION;
};

Output GCVSMain(InstancedVertex In)
{
    Output Out;

    matrix <float, 4, 4> model = {  scale,  0.0f,   0.0f,   In.iPosition.x,
                                    0.0f,   scale,  0.0f,   In.iPosition.y,
                                    0.0f,   0.0f,   scale,  In.iPosition.z,
                                    0.0f,   0.0f,   0.0f,   1.0f };

    float4 pos = mul(model, float4(In.position, 1.0f));
    pos = mul(view, pos);
    pos = mul(proj, pos);

    Out.position = pos;
    Out.color = In.iColor;

    return Out;
}
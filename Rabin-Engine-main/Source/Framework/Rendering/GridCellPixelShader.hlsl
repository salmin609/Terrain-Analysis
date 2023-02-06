struct Input
{
    float4 color : COLOR;
};

float4 GCPSMain(Input IN) : SV_TARGET
{
    return IN.color;
}
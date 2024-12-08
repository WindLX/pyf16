#include "test_utils.h"
#include "hifi_F16_AeroData.h"

int frsys_step()
{
    printf("f16 model test step\n");
    double val[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    hifi_C(0.5, 0.5, 0.5, val);
    for (int i = 0; i < 6; i++)
    {
        printf("val[%d] = %f\n", i, val[i]);
    }
    DOUBLE_EQ(val[0], -0.045144);
    DOUBLE_EQ(val[1], -0.064519);
    DOUBLE_EQ(val[2], -0.063967);
    DOUBLE_EQ(val[3], -0.013983);
    DOUBLE_EQ(val[4], 0.002128);
    DOUBLE_EQ(val[5], -0.000905);
    return 0;
}

int main()
{
    int r = fr_main(frsys_step);
    return r;
}
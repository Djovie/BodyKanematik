#include <kinematics.h>

int main()
{
    const char *log;
    bool result = false;

    uint16_t model_number = 0;
    dxl_wb.init(port_name, baud_rate, &log);

    for (int ser = 0; ser <= (int)range(dxl_id); ser++)
    {
        dxl_wb.ping(dxl_id[ser], &model_number, &log);
        if (!result)
        {
            printf("%s\n", log);
            printf("Failed to ping\n");
        }

        dxl_wb.addSyncWriteHandler(dxl_id[0], "Goal_Position", &log);
        if (!result)
        {
            printf("%s\n", log);
            printf("Failed to add sync write handler\n");
        }
    }

    //   loop program
    while (1)
    {
        gerak_body_x(10,30000);    
    }

    return 0;
}

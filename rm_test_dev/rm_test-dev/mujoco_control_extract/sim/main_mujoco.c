#include "sim_adapter.h"

#include <stdio.h>

int main(int argc, char **argv)
{
    (void)argc;
    (void)argv;

    SimController_Init();

    /*
     * TODO:
     * 1. Load MJCF with MuJoCo.
     * 2. Read qpos/qvel/body pose into SimControllerState.
     * 3. Call SimController_SetState/SetCommand/Step.
     * 4. Write SimControllerOutput to MuJoCo actuators.
     */
    printf("rm_controller skeleton is ready. Connect MuJoCo model loop here.\\n");
    return 0;
}


#include "TASK/DrivceTask.h"
#include "UTD.h"


using namespace std;

int main() {
    UTD camera_task(CameraCaptureTask);
    UTD calibrate_task(CalibrationTask);

    camera_task.AsyncRun();
//    calibrate_task.AsyncRun();

    while(true);
}

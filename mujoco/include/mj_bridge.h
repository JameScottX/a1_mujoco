#ifndef MJ_BRIDGE_H_
#define MJ_BRIDGE_H_

#include "mjxmacro.h"
#include "uitools.h"

#include <stdlib.h>
#include <limits.h>
#include <stdio.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

#include "eigen_helpers.h"


#define JIT_NUM 3        
#define _PI_ 3.14159265358979323846 
#define TIME_STEP 2                
#define VIDEO_FRAMERATE "30"

#define ID_NAME_LOOKUP(model, idvar, objtype, name)                     \
    do {                                                                \
        idvar = mj_name2id(model, objtype, #name);                      \
        if (-1 == idvar) {                                              \
            fprintf(stderr, "Could not find body named " #name "\n");   \
        }                                                               \
    } while (0)


extern long long time_once;

extern char mjb_comd_title[1024];
extern char mjb_comd_info[1024];
extern bool mjb_ctl_flag;


struct VideoRec{
    enum {
        hold = 0x01,
        init,
        rec,
        close,
    };
    FILE *pipe_video_out;
    int video_width;
    int video_height;
    unsigned char *frame;
    unsigned char record_status = hold;
    long end_t = 0;
    long sleep_time = 0;
    long sleep_peroid = 25000;
};

extern VideoRec video_rec;

double peroid_control(bool mode, long long micro_second_peroid);
long long get_microseconds(void);
void sensor_get_from_mj(mjModel *m, mjData *d, double time_step_);
void mj_main_init(mjModel *m, mjData *d);
void mj_main_loop(mjModel *m, mjData *d, double time_step_);

void foot_sim_force(mjModel *m, mjData *d, double cfrc[24]);
void body_sim_hold(mjModel *m, mjData *d);
void body_sim_release(mjModel *m, mjData *d);
void driver_sim_input(const mjModel* m, mjData *d);
void mj_infotext(char* title, char* content);

void mj_init_recording(VideoRec *sim, const char* videofile, int width, int height, GLFWwindow* window);
void mj_record_frame(VideoRec *sim, GLFWwindow* window, mjrContext *con);
void mj_close_recording(VideoRec *sim);


#endif



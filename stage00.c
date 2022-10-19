/***************************************************************
                           stage00.c
                
  first acomplished attempt to render and move nick around
             and make camera follow him!
***************************************************************/

#include <math.h>
#include <nusys.h>
#include <string.h> // Needed for CrashSDK compatibility
#include "config.h"
#include "structs.h"
#include "helper.h"
#include "sausage64.h"
#include "texcube.h"
#include "palette.h"
#include "nick.h"
#include "willy.h"
#include "axisMdl.h"
#include "debug.h"


/*********************************
              Macros
*********************************/

#define USB_BUFFER_SIZE 256

/*********************************
        Function Prototypes
*********************************/

OSTime get_time();
void time_management(TimeData time);

void move_entity(Entity *entity, Camera camera, NUContData cont[1]);
int lim(u32 input);
void move_entity_cbuttons(Entity *entity, Camera *camera, NUContData cont[1]);

float rad(float angle);
float deg(float rad);

void handle_camera_cbuttons(Camera *camera, NUContData cont[1]);

void handle_camera_analog_stick(Camera *camera, NUContData cont[1]);
void get_distances(Camera *camera);
void get_cam_position(Camera *camera, Entity entity);
void move_cam(Camera *camera, Entity entity, NUContData cont[1]);
void set_light(LightData *light);
void set_cam(Camera *camera, Entity entity);

void animate_nick(NUContData cont[1]);
void animate_willy(NUContData cont[1]);
void nick_animcallback(u16 anim);
void willy_animcallback(u16 anim);

void draw_animated_entity(AnimatedEntity *entity);
void draw_static_entity(StaticEntity *static_entity);
void draw_world(AnimatedEntity entity, Camera *camera, LightData *light);

void draw_debug_data();

/*********************************
             Globals
*********************************/

// temp in global scope for debugging
int forward_speed;
int side_speed;

//Variables
TimeData time_data = {
    cur_frame_index: 0,
};
float animspeed;

// Camera
Camera cam = {
    distance_from_entity: 900,
    pitch: 30,
    angle_around_entity: 0,
};

LightData light_data = {
    angle: { 0, 0, -90},
    ambcol: 100,
};


// Entities
AnimatedEntity nick = {
    entity: {
        pos: { 0, 0, 0},
    }
};

Mtx nickMtx[MESHCOUNT_nick];

AnimatedEntity willy = {
    entity: {
        pos: { 400, 400, 0},
    }
};

Mtx willyMtx[MESHCOUNT_willy];

StaticEntity axis = {
    entity: {
        pos: { 0, 0, 0},
    }
};


// USB
static char uselight = TRUE;
static char drawaxis = TRUE;
static char freezelight = FALSE;
static char usb_buffer[USB_BUFFER_SIZE];


/*==============================
    stage00_init
    Initialize the stage
==============================*/

void stage00_init(void){
    // Initialize entities
    sausage64_initmodel(&nick.helper, MODEL_nick, nickMtx);
    sausage64_set_anim(&nick.helper, ANIMATION_nick_idle); 
    sausage64_set_animcallback(&nick.helper, nick_animcallback);

    sausage64_initmodel(&willy.helper, MODEL_willy, willyMtx);
    sausage64_set_anim(&willy.helper, ANIMATION_willy_run); 
    sausage64_set_animcallback(&willy.helper, willy_animcallback);
    
    // Set nick's animation speed based on region
    #if TV_TYPE == PAL    
        animspeed = 0.66;
    #else
        animspeed = 0.5;
    #endif
}

OSTime get_time(){

    OSTime time = (s32)OS_CYCLES_TO_USEC(osGetTime()) / 1000000;
    return time;
}

/*==============================
    time_managment
    calculates FPS and frame_duration variable    
==============================*/

void time_managment(TimeData *time){

    time->cur_frame = osGetTime();
    if (time->cur_frame_index == 0) {
        time->last_frame = time->frame_times[FRAMETIME_COUNT - 1];
    } else {
        time->last_frame = time->frame_times[time->cur_frame_index];
    }
    time->cur_frame_index++;

    time->frame_times[time->cur_frame_index] = time->cur_frame;


    if (time->cur_frame_index >= FRAMETIME_COUNT) {
        time->cur_frame_index = 0;
    }

    time->frame_duration = OS_CYCLES_TO_USEC(time->cur_frame - time->last_frame) / 1000000.0f;

    time->FPS = ((f32)FRAMETIME_COUNT * 1000000.0f) / (s32)OS_CYCLES_TO_USEC(time->cur_frame - time->last_frame);
}


/*==============================
    move_entity
    Moves entity with analog stick
==============================*/

void move_entity(Entity *entity, Camera camera, NUContData cont[1]){
	
	if (fabs(cont->stick_x) < 7){cont->stick_x = 0;}
	if (fabs(cont->stick_y) < 7){cont->stick_y = 0;}


	if ( cont->stick_x != 0 || cont->stick_y != 0) {
    	entity->yaw = deg(atan2(cont->stick_x, -cont->stick_y) - rad(camera.angle_around_entity));
        entity->speed = 1000;
    }

    if ( cont->stick_x == 0 && cont->stick_y == 0) {
        entity->speed = 0;
    }
    
    float frame_distance = time_data.frame_duration * entity->speed;

    entity->pos[0] += frame_distance * sin(rad(entity->yaw));
    entity->pos[1] -= frame_distance * cos(rad(entity->yaw));

}


/*==============================
    lim
    auxiliary function for c button movement1º
==============================*/

int lim(u32 input){
    if (input == 0) {return 0;}
    else {return 1;}
}


/*==============================
    move_entity_cbuttons

    Moves entity with c buttons
==============================*/

void move_entity_cbuttons(Entity *entity, Camera *camera, NUContData cont[1]){

    forward_speed = lim(contdata[0].button & U_CBUTTONS) - lim(contdata[0].button & D_CBUTTONS);
    side_speed = lim(contdata[0].button & L_CBUTTONS) - lim(contdata[0].button & R_CBUTTONS);

	if (forward_speed != 0 || side_speed != 0) {
    	entity->yaw = deg(atan2(-side_speed, -forward_speed) - rad(camera->angle_around_entity));
    }

    float frame_distance_forward = time_data.frame_duration * forward_speed * 500;
    float frame_distance_side = time_data.frame_duration * side_speed * 500;

    if (forward_speed != 0){

        entity->pos[0] += frame_distance_forward * sin(rad(camera->angle_around_entity));
        entity->pos[1] += frame_distance_forward * cos(rad(camera->angle_around_entity));
    }

    if (side_speed != 0){

        entity->pos[0] += frame_distance_side * sin(rad(camera->angle_around_entity - 90));
        entity->pos[1] += frame_distance_side * cos(rad(camera->angle_around_entity - 90));
    }
}

  
/*==============================
    rad & deg
    convert angles to radians
==============================*/

float rad(float angle){
	float radian = M_PI / 180 * angle;
	return radian;
}

float deg(float rad){
	float angle = 180 / M_PI * rad;
	return angle;
}


 /* 
     Nintendo's official button names 
    U_JPAD
    L_JPAD
    R_JPAD
    D_JPAD
    START_BUTTON
    A_BUTTON
    B_BUTTON
    U_CBUTTONS
    L_CBUTTONS
    R_CBUTTONS
    D_CBUTTONS
    L_TRIG
    R_TRIG
    Z_TRIG
*/


/*==============================
    handle_camera_cbuttons
    handles pitch and distance_from_entity 
    and angle_around_entity variables
==============================*/

void handle_camera_cbuttons(Camera *camera, NUContData cont[1]){

    if (cont[0].trigger & U_CBUTTONS && camera->distance_from_entity == 2000){
        camera->distance_from_entity = 1200;
        camera->pitch = 35;
    } else
    if (cont[0].trigger & U_CBUTTONS && camera->distance_from_entity == 1200){
        camera->distance_from_entity = 700;
        camera->pitch = 30;
    }

    if (cont[0].trigger & D_CBUTTONS && camera->distance_from_entity == 700){
        camera->distance_from_entity = 1200;
        camera->pitch = 35;
    } else
    if (cont[0].trigger & D_CBUTTONS && camera->distance_from_entity == 1200){
        camera->distance_from_entity = 2000;
        camera->pitch = 40;
    }

    if (cont[0].trigger & L_CBUTTONS){
        camera->angle_around_entity += 45;
    }
   
    if (cont[0].trigger & R_CBUTTONS && camera->angle_around_entity == 0){
        camera->angle_around_entity = 360;
        camera->angle_around_entity -= 45;
    }else
    if (cont[0].trigger & R_CBUTTONS){
        camera->angle_around_entity -= 45;
    }

    if (camera->angle_around_entity == 360){
        camera->angle_around_entity = 0;
    }
}


/*==============================
    handle_camera_analog_stick

    moves camera with analog stick
==============================*/

void handle_camera_analog_stick(Camera *camera, NUContData cont[1]){

    if (fabs(cont->stick_x) < 7){cont->stick_x = 0;}
    if (fabs(cont->stick_y) < 7){cont->stick_y = 0;}

    camera->angle_around_entity += cont->stick_x / 50;
    camera->pitch += cont->stick_y / 50;

    if (cam.angle_around_entity > 360) {cam.angle_around_entity  = 0;}
    if (cam.angle_around_entity < 0) {cam.angle_around_entity  = 360;}

    if (cam.angle_around_entity  > 360) {cam.angle_around_entity  = 0;}
    if (cam.angle_around_entity  < 0) {cam.angle_around_entity  = 360;}

    if (cam.pitch > 80) {cam.pitch = 80;}
    if (cam.pitch < -80) {cam.pitch = -80;}
}


/*==============================
    get_distances
    calculates vertical and horizontal
    distances from entity
==============================*/

void get_distances(Camera *camera){

    camera->horizontal_distance_from_entity = camera->distance_from_entity * cos(rad(camera->pitch));
	camera->vertical_distance_from_entity = camera->distance_from_entity * sin(rad(camera->pitch));
}


/*==============================
    get_cam_position
    calculates camera coordinates
==============================*/

void get_cam_position(Camera *camera, Entity entity){

    camera->pos[0] = entity.pos[0] - camera->horizontal_distance_from_entity * sin(rad(camera->angle_around_entity));
    camera->pos[1] = entity.pos[1] - camera->horizontal_distance_from_entity * cos(rad(camera->angle_around_entity));
    camera->pos[2] = camera->vertical_distance_from_entity + entity.pos[2];
}


/*==============================
    move_cam
    Controls camera movement
==============================*/

void move_cam(Camera *camera, Entity entity, NUContData cont[1]){

    //handle_camera_cbuttons(camera, cont);
    //handle_angle_around_entity(camera, cont);
    handle_camera_analog_stick
(camera, cont);
    get_distances(camera);
    get_cam_position(camera, entity);
}


/*==============================
    set_light
    Sets the lights 
==============================*/

void set_light(LightData *light){

    int i;
    
    // Setup the lights
    if (!uselight)
        light->ambcol = 255;
    for (i=0; i<3; i++){
        light->amb.l.col[i] = light->ambcol;
        light->amb.l.colc[i] = light->ambcol;
        light->dir.l.col[i] = 255;
        light->dir.l.colc[i] = 255;
    }
    // handle the light direction so it's always projecting from the camera's position
    if (!freezelight){
        light->dir.l.dir[0] = -127*sinf(light->angle[0]*0.0174532925);
        light->dir.l.dir[1] = 127*sinf(light->angle[2]*0.0174532925)*cosf(light->angle[0]*0.0174532925);
        light->dir.l.dir[2] = 127*cosf(light->angle[2]*0.0174532925)*cosf(light->angle[0]*0.0174532925);
    }
    // Send the light struct to the RSP
    gSPNumLights(glistp++, NUMLIGHTS_1);
    gSPLight(glistp++, &light->dir, 1);
    gSPLight(glistp++, &light->amb, 2);
    gDPPipeSync(glistp++);
}


/*==============================
    set_cam
    Sets the camera 
==============================*/

void set_cam(Camera *camera, Entity entity){

    // Setup the cam.projection matrix
    guPerspective(
    	&camera->projection, &camera->normal, 
        45, (float)SCREEN_WD / (float)SCREEN_HT, 
    	10.0, 10000.0, 0.01);
    
    guLookAt(
    	&camera->viewpoint,
    	camera->pos[0], camera->pos[1], camera->pos[2],
    	entity.pos[0], entity.pos[1], entity.pos[2] + 120,
    	0, 0, 1
  	);

    // Apply the cam.projection matrix
    gSPMatrix(glistp++, &camera->projection, G_MTX_PROJECTION | G_MTX_LOAD | G_MTX_NOPUSH);
    gSPMatrix(glistp++, &camera->viewpoint, G_MTX_PROJECTION | G_MTX_MUL | G_MTX_NOPUSH);
    gSPPerspNormalize(glistp++, &camera->normal);

    // Initialize the model matrix
    guMtxIdent(&camera->modeling);
    gSPMatrix(glistp++, &camera->modeling, G_MTX_MODELVIEW | G_MTX_LOAD | G_MTX_NOPUSH);
}


/*==============================
    animate_nick & animate_willy
    link entity animations to controller input
==============================*/

void animate_nick(NUContData cont[1]){

    if (cont[0].trigger & A_BUTTON && sausage64_get_currentanim(&nick.helper) != ANIMATION_nick_roll && sausage64_get_currentanim(&nick.helper) != ANIMATION_nick_jumpUP){
        sausage64_set_anim(&nick.helper, ANIMATION_nick_jumpUP);
    }

    if (cont[0].trigger & B_BUTTON && sausage64_get_currentanim(&nick.helper) != ANIMATION_nick_roll && sausage64_get_currentanim(&nick.helper) != ANIMATION_nick_jumpUP){
        sausage64_set_anim(&nick.helper, ANIMATION_nick_roll);
    }

    if (((forward_speed != 0 || side_speed != 0) && sausage64_get_currentanim(&nick.helper) != ANIMATION_nick_roll ) && sausage64_get_currentanim(&nick.helper) != ANIMATION_nick_run  && sausage64_get_currentanim(&nick.helper) != ANIMATION_nick_jumpUP){
    	sausage64_set_anim(&nick.helper, ANIMATION_nick_run); 
    }

    /*
    } if (((cont->stick_x != 0 || cont->stick_y != 0) && sausage64_get_currentanim(&nick.helper) != ANIMATION_nick_roll ) && sausage64_get_currentanim(&nick.helper) != ANIMATION_nick_run  && sausage64_get_currentanim(&nick.helper) != ANIMATION_nick_jumpUP){
    	sausage64_set_anim(&nick.helper, ANIMATION_nick_run); 
    }
    */
   
    if (((forward_speed == 0 && side_speed == 0) && sausage64_get_currentanim(&nick.helper) != ANIMATION_nick_roll ) && sausage64_get_currentanim(&nick.helper) != ANIMATION_nick_idle  && sausage64_get_currentanim(&nick.helper) != ANIMATION_nick_jumpUP) {
    	sausage64_set_anim(&nick.helper, ANIMATION_nick_idle);
    }
    
}


void animate_willy(NUContData cont[1]){

    if (cont[0].trigger & A_BUTTON && sausage64_get_currentanim(&willy.helper) != ANIMATION_willy_roll && sausage64_get_currentanim(&willy.helper) != ANIMATION_willy_jump){
        sausage64_set_anim(&willy.helper, ANIMATION_willy_jump);
    }

    if (cont[0].trigger & B_BUTTON && sausage64_get_currentanim(&willy.helper) != ANIMATION_willy_roll && sausage64_get_currentanim(&willy.helper) != ANIMATION_willy_jump){
        sausage64_set_anim(&willy.helper, ANIMATION_willy_roll);
    }
	
    if (((cont->stick_x != 0 || cont->stick_y != 0) && sausage64_get_currentanim(&willy.helper) != ANIMATION_willy_roll ) && sausage64_get_currentanim(&willy.helper) != ANIMATION_willy_run  && sausage64_get_currentanim(&willy.helper) != ANIMATION_willy_jump){
    	sausage64_set_anim(&willy.helper, ANIMATION_willy_run); 
    }
    
    if (((cont->stick_x == 0 && cont->stick_y == 0) && sausage64_get_currentanim(&willy.helper) != ANIMATION_willy_roll ) && sausage64_get_currentanim(&willy.helper) != ANIMATION_willy_idle  && sausage64_get_currentanim(&willy.helper) != ANIMATION_willy_jump) {
    	sausage64_set_anim(&willy.helper, ANIMATION_willy_idle);
    }
}


/*==============================
    animcallback
    Called before an animation finishes
==============================*/

void nick_animcallback(u16 anim){

    // Go to idle animation when we finished attacking
    switch(anim)
    {
        case ANIMATION_nick_roll:
        case ANIMATION_nick_jumpUP:
            sausage64_set_anim(&nick.helper, ANIMATION_nick_idle);
            break;
    }
}

void willy_animcallback(u16 anim)
{
    // Go to idle animation when we finished attacking
    switch(anim)
    {
        case ANIMATION_willy_roll:
        case ANIMATION_willy_jump:
            //sausage64_set_anim(&willy.helper, ANIMATION_willy_idle);
            break;
    }
}


/*==============================
    draw_animated_entity
    draws animated entities
==============================*/

void draw_animated_entity(AnimatedEntity *animated_entity){

    Entity *entity = &animated_entity->entity;
    guTranslate(&entity->pos_mtx, entity->pos[0], entity->pos[1], entity->pos[2]);
    guRotate(&entity->rot_mtx[0], entity->pitch, 1, 0, 0);
    guRotate(&entity->rot_mtx[1], entity->yaw, 0, 0, 1);

    gSPMatrix(glistp++, OS_K0_TO_PHYSICAL(&entity->pos_mtx), G_MTX_MODELVIEW | G_MTX_LOAD | G_MTX_PUSH);
    gSPMatrix(glistp++, OS_K0_TO_PHYSICAL(&entity->rot_mtx[0]), G_MTX_MODELVIEW | G_MTX_MUL | G_MTX_NOPUSH);
    gSPMatrix(glistp++, OS_K0_TO_PHYSICAL(&entity->rot_mtx[1]), G_MTX_MODELVIEW | G_MTX_MUL | G_MTX_NOPUSH);

    sausage64_drawmodel(&glistp, &animated_entity->helper);
}


/*==============================
    draw_static_entity
    draws static entities
==============================*/

void draw_static_entity(StaticEntity *static_entity){
    
    Entity *entity = &static_entity->entity;

    guTranslate(&entity->pos_mtx, entity->pos[0], entity->pos[1], entity->pos[2]);
    guRotate(&entity->rot_mtx[0], 0, 1, 0, 0);
    guRotate(&entity->rot_mtx[1], 0, 0, 0, 1);

    gSPMatrix(glistp++, OS_K0_TO_PHYSICAL(&entity->pos_mtx), G_MTX_MODELVIEW | G_MTX_LOAD | G_MTX_PUSH);
    gSPMatrix(glistp++, OS_K0_TO_PHYSICAL(&entity->rot_mtx[0]), G_MTX_MODELVIEW | G_MTX_MUL | G_MTX_NOPUSH);
    gSPMatrix(glistp++, OS_K0_TO_PHYSICAL(&entity->rot_mtx[1]), G_MTX_MODELVIEW | G_MTX_MUL | G_MTX_NOPUSH);
    
    gSPDisplayList(glistp++, gfx_axis);
}


/*==============================
    draw_world
    Draws entities 
==============================*/

void draw_world(AnimatedEntity highlighted, Camera *camera, LightData *light){

    // Initialize the RCP to draw stuff nicely
    gDPSetCycleType(glistp++, G_CYC_1CYCLE);
    gDPSetDepthSource(glistp++, G_ZS_PIXEL);
    gSPClearGeometryMode(glistp++,0xFFFFFFFF);
    gSPSetGeometryMode(glistp++, G_SHADE | G_ZBUFFER | G_CULL_BACK | G_SHADING_SMOOTH | G_LIGHTING);
    gSPTexture(glistp++, 0xFFFF, 0xFFFF, 0, G_TX_RENDERTILE, G_ON);
    gDPSetRenderMode(glistp++, G_RM_AA_ZB_OPA_SURF, G_RM_AA_ZB_OPA_SURF);
    gDPSetCombineMode(glistp++, G_CC_PRIMITIVE, G_CC_PRIMITIVE);
    gDPSetTexturePersp(glistp++, G_TP_PERSP);
    gDPSetTextureFilter(glistp++, G_TF_BILERP);
    gDPSetTextureConvert(glistp++, G_TC_FILT);
    gDPSetTextureLOD(glistp++, G_TL_TILE);
    gDPSetTextureDetail(glistp++, G_TD_CLAMP);
    gDPSetTextureLUT(glistp++, G_TT_NONE);

    //set view matrix and lights
    set_cam(camera, highlighted.entity);

    set_light(light);

    //draw the entities
    draw_static_entity(&axis);

    draw_animated_entity(&nick);

    draw_animated_entity(&willy);
    
    // Syncronize the RCP and CPU and specify that our display list has ended
    gDPFullSync(glistp++);
    gSPEndDisplayList(glistp++);

    // Ensure the cache lines are valid
    osWritebackDCache(&camera->projection, sizeof(&camera->projection));
    osWritebackDCache(&camera->modeling, sizeof(camera->modeling));
}


/*==============================
    draw_debug_data
    Draws debug data
==============================*/

void draw_debug_data(){

    nuDebConTextPos(NU_DEB_CON_WINDOW0, 1, 1);
    nuDebConPrintf(NU_DEB_CON_WINDOW0, "frame duration %d", (int) (time_data.frame_duration * 10000));

    nuDebConTextPos(NU_DEB_CON_WINDOW0, 1, 2);
    nuDebConPrintf(NU_DEB_CON_WINDOW0, "cam angle around entity %d", (int)cam.angle_around_entity);
    nuDebConTextPos(NU_DEB_CON_WINDOW0, 1, 3);
    nuDebConPrintf(NU_DEB_CON_WINDOW0, "cam pitch %d", (int)cam.pitch);
    
    /*
    nuDebConTextPos(NU_DEB_CON_WINDOW0, 1, 4);
    nuDebConPrintf(NU_DEB_CON_WINDOW0, "diff %llu", time_data.cur_frame - time_data.last_frame);
    nuDebConTextPos(NU_DEB_CON_WINDOW0, 1, 5);
    nuDebConPrintf(NU_DEB_CON_WINDOW0, "horizontal distance %d", (int)cam.horizontal_distance_from_entity);
    nuDebConTextPos(NU_DEB_CON_WINDOW0, 1, 6);
    nuDebConPrintf(NU_DEB_CON_WINDOW0, "vertical distance %d", (int)cam.vertical_distance_from_entity);
    nuDebConTextPos(NU_DEB_CON_WINDOW0, 1, 7);
    nuDebConPrintf(NU_DEB_CON_WINDOW0, "cam x %d", (int)cam.pos[0]);
    nuDebConTextPos(NU_DEB_CON_WINDOW0, 1, 8);
    nuDebConPrintf(NU_DEB_CON_WINDOW0, "cam y %d", (int)cam.pos[1]);
    nuDebConTextPos(NU_DEB_CON_WINDOW0, 1, 9);
    nuDebConPrintf(NU_DEB_CON_WINDOW0, "cam z %d", (int)cam.pos[2]);
    nuDebConTextPos(NU_DEB_CON_WINDOW0, 1, 10);
    nuDebConPrintf(NU_DEB_CON_WINDOW0, "time %d", (int)get_time());
    */
}


/*==============================
    stage00_update
    Update stage variables every frame
==============================*/

void stage00_update(void){
    
    // Poll for USB commands
    debug_pollcommands();  

    //alculate fps
    time_managment(&time_data);

    // Read the controller
    nuContDataGetEx(contdata, 0);

    //handle movement
    //move_entity(&nick.entity, cam, contdata);
    move_entity_cbuttons
(&nick.entity, &cam, contdata);

    //move_entity(&willy.entity, cam, contdata);

    move_cam(&cam, nick.entity, contdata);

    //Handle animation
    animate_nick(contdata);
   
    // Advacnce animations
    sausage64_advance_anim(&willy.helper, animspeed);
    
    sausage64_advance_anim(&nick.helper, animspeed);

}


/*==============================
    stage00_draw
    Draw the stage
==============================*/

void stage00_draw(void){
    
    // Assign our glist pointer to our glist array for ease of access
    glistp = glist;

    // Initialize the RCP and framebuffer
    rcp_init();
    fb_clear(16, 32, 32);

    draw_world(nick, &cam, &light_data);    

    // Ensure we haven't gone over the display list size and start the graphics task
    debug_assert((glistp-glist) < GLIST_LENGTH);
    #if TV_TYPE != PAL
        nuGfxTaskStart(glist, (s32)(glistp - glist) * sizeof(Gfx), NU_GFX_UCODE_F3DEX, NU_SC_NOSWAPBUFFER);
    #else
        nuGfxTaskStart(glist, (s32)(glistp - glist) * sizeof(Gfx), NU_GFX_UCODE_F3DEX, NU_SC_SWAPBUFFER);
    #endif
    
    // Draw the menu (doesn't work on PAL)
    #if TV_TYPE != PAL
        nuDebConClear(NU_DEB_CON_WINDOW0);
        draw_debug_data();
        nuDebConDisp(NU_SC_SWAPBUFFER);
    #endif
}
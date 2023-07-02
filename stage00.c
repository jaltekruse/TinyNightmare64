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
#include "palette.h"
#include "nick.h"
#include "ground_block.h"

#include "axisMdl.h"

/*********************************
              Macros
*********************************/

#define COS_45 0.7071

/*********************************
        Function Prototypes
*********************************/

float rad(float angle);
float deg(float rad);

void time_management(TimeData *time);

void move_entity_analog_stick(Entity *entity, Camera camera, NUContData cont[1]);
void handle_camera_c_buttons(Camera *camera, NUContData cont[1]);

void move_entity_c_buttons(Entity *entity, Camera camera, NUContData cont[1]);
void handle_camera_analog_stick(Camera *camera, NUContData cont[1]);

void get_cam_position(Camera *camera, Entity *entity);
void move_cam(Camera *camera, Entity* entity, NUContData cont[1]);

void set_light(LightData *light);
void set_cam(Camera *camera, Entity entity);

void set_entity_state(AnimatedEntity * animated_entity, entity_state new_state);
void animate_nick(NUContData cont[1]);
void nick_animcallback(u16 anim);
void willy_animcallback(u16 anim);

void draw_animated_entity(AnimatedEntity *entity);
void draw_static_entity(StaticEntity *static_entity);
void draw_world(AnimatedEntity *entity, Camera *camera, LightData *light);

void draw_debug_data();

/*********************************
             Globals
*********************************/

//Variables
TimeData time_data = {
    FPS_index: 0,
    frame_duration: 0.03f
};
float animspeed;

// Camera
Camera cam = {
    distance_from_entity: 2000,
    pitch: 20,
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
        type: NICK,
        health: 100,
        damage: 10,
        scale: 3,
        ammo: 10
    }
};

Mtx nickMtx[MESHCOUNT_nick];

#define SCENERY_COUNT 1
StaticEntity scenery[SCENERY_COUNT] = {
 {entity : {pos : {-300, 300, 30}, scale : 1}, mesh : gfx_ground}
};

static char uselight = FALSE;
static char drawaxis = TRUE;
static char freezelight = TRUE;

/*==============================
    rad & deg
    convert between angles and radians
==============================*/

float rad(float angle){
	float radian = M_PI / 180 * angle;
	return radian;
}

float deg(float rad){
	float angle = 180 / M_PI * rad;
	return angle;
}

// https://en.wikipedia.org/wiki/Fast_inverse_square_root
float Q_rsqrt( float number )
{
	long i;
	float x2, y;
	const float threehalfs = 1.5F;

	x2 = number * 0.5F;
	y  = number;
	i  = * ( long * ) &y;                       // evil floating point bit level hacking
	i  = 0x5f3759df - ( i >> 1 );               // what the fuck? 
	y  = * ( float * ) &i;
	y  = y * ( threehalfs - ( x2 * y * y ) );   // 1st iteration
//	y  = y * ( threehalfs - ( x2 * y * y ) );   // 2nd iteration, this can be removed

	return y;
}

/*==============================
    move_entity
    Moves entity with analog stick
==============================*/

void move_entity_analog_stick(Entity *entity, Camera camera, NUContData cont[1]){

    // dead zone to avoid stick drift
	if (fabs(cont->stick_x) < 7){cont->stick_x = 0;}
	if (fabs(cont->stick_y) < 7){cont->stick_y = 0;}

    if ((cont->stick_x != 0 || cont->stick_y != 0)) {
        entity->yaw = deg(atan2(cont->stick_x, -cont->stick_y) - rad(camera.angle_around_entity));
        entity->speed = 1/Q_rsqrt(cont->stick_x * cont->stick_x + cont->stick_y * cont->stick_y) * 12;
    }

    if ( cont->stick_x == 0 && cont->stick_y == 0) {
        entity->speed = 0;
    }
}

void move_entity_one_frame(Entity *entity){

    float frame_distance = time_data.frame_duration * entity->speed;
    entity->pos[0] += frame_distance * sin(rad(entity->yaw));
    entity->pos[1] -= frame_distance * cos(rad(entity->yaw));
}

/*==============================
    get_cam_position
    calculates camera coordinates
==============================*/

void get_cam_position(Camera *camera, Entity *entity){

    camera->horizontal_distance_from_entity = camera->distance_from_entity * cos(rad(camera->pitch));
	camera->vertical_distance_from_entity = camera->distance_from_entity * sin(rad(camera->pitch));

    camera->pos[0] = entity->pos[0] - camera->horizontal_distance_from_entity * sin(rad(camera->angle_around_entity));
    camera->pos[1] = entity->pos[1] - camera->horizontal_distance_from_entity * cos(rad(camera->angle_around_entity));
    camera->pos[2] = camera->vertical_distance_from_entity + 1/Q_rsqrt(entity->pos[2]);

    if ((camera->vertical_distance_from_entity + entity->pos[2]) < 5){cam.pos[2] = 5;}
}


/*==============================
    move_cam
    Controls camera movement
==============================*/

void move_cam(Camera *camera, Entity *entity, NUContData cont[1]){

    //handle_camera_c_buttons(camera, cont);
    //handle_camera_analog_stick(camera, cont);
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
    	entity.pos[0], entity.pos[1], 1/Q_rsqrt(entity.pos[2]) + 120,
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

void update_animation_based_on_state(AnimatedEntity * animated_entity) {
}

void set_entity_state(AnimatedEntity * animated_entity, entity_state new_state) {
}

/*==============================
    animate_nick & animate_willy
    link entity animations to controller input
==============================*/

void handle_controller_input(NUContData cont[1], AnimatedEntity* entity){

    //cont[0].trigger & D_CBUTTONS, U_CBUTTONS
    if (cont[0].trigger & R_TRIG) {
    }
    if (cont[0].trigger & A_BUTTON) set_entity_state(entity, JUMP);
    if (cont[0].trigger & B_BUTTON) set_entity_state(entity, ROLL);
    if (entity->entity.speed > 900) {
        set_entity_state(entity, RUN);
    } else if (cont->stick_x != 0 || cont->stick_y != 0) {
        set_entity_state(entity, WALK);
    }

    //handle movement
    move_entity_analog_stick(&entity->entity, cam, contdata);
    //move_entity_c_buttons(entity.entity, cam, contdata);

    move_cam(&cam, &entity->entity, contdata);
}

/*==============================
    animcallback
    Called before an animation finishes
==============================*/

void nick_animcallback(u16 anim){
}

/*==============================
    draw_animated_entity
    draws animated entities
==============================*/

void draw_animated_entity(AnimatedEntity *animated_entity){

    guTranslate(&animated_entity->entity.pos_mtx, animated_entity->entity.pos[0], animated_entity->entity.pos[1], animated_entity->entity.pos[2]);
    guRotate(&animated_entity->entity.rot_mtx[0], animated_entity->entity.pitch, 1, 0, 0);
    guRotate(&animated_entity->entity.rot_mtx[1], animated_entity->entity.yaw, 0, 0, 1);
    float scale = animated_entity->entity.scale;
    guScale(&animated_entity->entity.scale_mtx, scale, scale, scale);

    gSPMatrix(glistp++, OS_K0_TO_PHYSICAL(&animated_entity->entity.pos_mtx), G_MTX_MODELVIEW | G_MTX_MUL | G_MTX_PUSH);
    gSPMatrix(glistp++, OS_K0_TO_PHYSICAL(&animated_entity->entity.rot_mtx[0]), G_MTX_MODELVIEW | G_MTX_MUL | G_MTX_PUSH);
    gSPMatrix(glistp++, OS_K0_TO_PHYSICAL(&animated_entity->entity.rot_mtx[1]), G_MTX_MODELVIEW | G_MTX_MUL | G_MTX_PUSH);
    gSPMatrix(glistp++, OS_K0_TO_PHYSICAL(&animated_entity->entity.scale_mtx), G_MTX_MODELVIEW | G_MTX_MUL | G_MTX_PUSH);

    sausage64_drawmodel(&glistp, &animated_entity->helper);

    gSPPopMatrix(glistp++, G_MTX_MODELVIEW);
    gSPPopMatrix(glistp++, G_MTX_MODELVIEW);
    gSPPopMatrix(glistp++, G_MTX_MODELVIEW);
    gSPPopMatrix(glistp++, G_MTX_MODELVIEW);
}

/*==============================
    draw_static_entity
    draws static entities
==============================*/

void draw_static_entity(StaticEntity *static_entity){

    guTranslate(&static_entity->entity.pos_mtx, static_entity->entity.pos[0], static_entity->entity.pos[1], static_entity->entity.pos[2]);
    guRotate(&static_entity->entity.rot_mtx[0], static_entity->entity.pitch, 1, 0, 0);
    guRotate(&static_entity->entity.rot_mtx[1], static_entity->entity.yaw, 0, 0, 1);
    float scale = static_entity->entity.scale;
    guScale(&static_entity->entity.scale_mtx, scale, scale, scale);

    gSPMatrix(glistp++, OS_K0_TO_PHYSICAL(&static_entity->entity.pos_mtx), G_MTX_MODELVIEW | G_MTX_MUL | G_MTX_PUSH);
    gSPMatrix(glistp++, OS_K0_TO_PHYSICAL(&static_entity->entity.rot_mtx[0]), G_MTX_MODELVIEW | G_MTX_MUL | G_MTX_PUSH);
    gSPMatrix(glistp++, OS_K0_TO_PHYSICAL(&static_entity->entity.rot_mtx[1]), G_MTX_MODELVIEW | G_MTX_MUL | G_MTX_PUSH);
    gSPMatrix(glistp++, OS_K0_TO_PHYSICAL(&static_entity->entity.scale_mtx), G_MTX_MODELVIEW | G_MTX_MUL | G_MTX_PUSH);
    
    gSPDisplayList(glistp++, static_entity->mesh);

    gSPPopMatrix(glistp++, G_MTX_MODELVIEW);
    gSPPopMatrix(glistp++, G_MTX_MODELVIEW);
    gSPPopMatrix(glistp++, G_MTX_MODELVIEW);
    gSPPopMatrix(glistp++, G_MTX_MODELVIEW);
}

void set_pt(float* dest, float* src) {
    dest[0] = src[0];
    dest[1] = src[1];
    dest[2] = src[2];
}

float dot(float *u, float *v) {
    return u[0] * v[0] + u[1] * v[1]; 
}

void vector(float *dest, float *p1, float *p2) {
    dest[0] = p2[0] - p1[0];
    dest[1] = p2[1] - p1[1];
}

void detect_collisions() {
}

/*==============================
    draw_world
    Draws entities 
==============================*/

void draw_world(AnimatedEntity *highlighted, Camera *camera, LightData *light){

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
    set_cam(camera, highlighted->entity);

    set_light(light);

    //draw the entities

    for (int i = 0; i < SCENERY_COUNT; i++) {
        draw_static_entity(&scenery[i]);
    }

    draw_animated_entity(&nick);

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


int angle_to_player;

void draw_debug_data(){

    nuDebConTextPos(NU_DEB_CON_WINDOW0, 1, 1);
    nuDebConPrintf(NU_DEB_CON_WINDOW0, "FPS %d", (int)time_data.FPS);

    nuDebConTextPos(NU_DEB_CON_WINDOW0, 1, 2);
    nuDebConPrintf(NU_DEB_CON_WINDOW0, "angle to player %d", angle_to_player);
}


/*==============================
    stage00_init
    Initialize the stage
==============================*/

void stage00_init(void){

    // Initialize entities
    sausage64_initmodel(&nick.helper, MODEL_nick, nickMtx);
    sausage64_set_anim(&nick.helper, ANIMATION_nick_idle); 
    sausage64_set_animcallback(&nick.helper, nick_animcallback);

    // Set nick's animation speed based on region
    #if TV_TYPE == PAL    
        animspeed = 0.66;
    #else
        animspeed = 0.5;
    #endif
}

/*==============================
    stage00_update
    Update stage variables every frame
==============================*/


void stage00_update(void){
    
    // Read the controller
    nuContDataGetEx(contdata, 0);

    //Handle animation
    handle_controller_input(contdata, &nick);

    detect_collisions();

    move_entity_one_frame(&nick.entity);
   
    // Advacnce animations
    
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

    draw_world(&nick, &cam, &light_data);    

    // Ensure we haven't gone over the display list size and start the graphics task
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

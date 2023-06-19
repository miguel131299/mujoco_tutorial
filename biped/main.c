

#include<stdbool.h> //for bool
//#include<unistd.h> //for usleep
//#include <math.h>

#include "mujoco/mujoco.h"
#include "GLFW/glfw3.h"
#include "stdio.h"
#include "stdlib.h"
#include "string.h"

#include "utility.c"

int fsm_hip;
int fsm_knee1;
int fsm_knee2;

#define fsm_leg1_swing 0
#define fsm_leg2_swing 1

#define fsm_knee1_stance 0
#define fsm_knee1_retract 1
#define fsm_knee2_stance 0
#define fsm_knee2_retract 1

int step_no;

//simulation end time
double simend = 60;

//related to writing data to a file
FILE *fid;
int loop_index = 0;
const int data_frequency = 10; //frequency at which data is written to a file


// char xmlpath[] = "../myproject/template_writeData/pendulum.xml";
// char datapath[] = "../myproject/template_writeData/data.csv";


//Change the path <template_writeData>
//Change the xml file
char path[] = "./";
char xmlfile[] = "biped.xml";


char datafile[] = "data.csv";


// MuJoCo data structures
mjModel* m = NULL;                  // MuJoCo model
mjData* d = NULL;                   // MuJoCo data
mjvCamera cam;                      // abstract camera
mjvOption opt;                      // visualization options
mjvScene scn;                       // abstract scene
mjrContext con;                     // custom GPU context

// mouse interaction
bool button_left = false;
bool button_middle = false;
bool button_right =  false;
double lastx = 0;
double lasty = 0;

// holders of one step history of time and position to calculate dertivatives
mjtNum position_history = 0;
mjtNum previous_time = 0;

// controller related variables
float_t ctrl_update_freq = 100;
mjtNum last_update = 0.0;
mjtNum ctrl;

// keyboard callback
void keyboard(GLFWwindow* window, int key, int scancode, int act, int mods)
{
    // backspace: reset simulation
    if( act==GLFW_PRESS && key==GLFW_KEY_BACKSPACE )
    {
        mj_resetData(m, d);
        mj_forward(m, d);
    }
}

// mouse button callback
void mouse_button(GLFWwindow* window, int button, int act, int mods)
{
    // update button state
    button_left =   (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT)==GLFW_PRESS);
    button_middle = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_MIDDLE)==GLFW_PRESS);
    button_right =  (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT)==GLFW_PRESS);

    // update mouse position
    glfwGetCursorPos(window, &lastx, &lasty);
}


// mouse move callback
void mouse_move(GLFWwindow* window, double xpos, double ypos)
{
    // no buttons down: nothing to do
    if( !button_left && !button_middle && !button_right )
        return;

    // compute mouse displacement, save
    double dx = xpos - lastx;
    double dy = ypos - lasty;
    lastx = xpos;
    lasty = ypos;

    // get current window size
    int width, height;
    glfwGetWindowSize(window, &width, &height);

    // get shift key state
    bool mod_shift = (glfwGetKey(window, GLFW_KEY_LEFT_SHIFT)==GLFW_PRESS ||
                      glfwGetKey(window, GLFW_KEY_RIGHT_SHIFT)==GLFW_PRESS);

    // determine action based on mouse button
    mjtMouse action;
    if( button_right )
        action = mod_shift ? mjMOUSE_MOVE_H : mjMOUSE_MOVE_V;
    else if( button_left )
        action = mod_shift ? mjMOUSE_ROTATE_H : mjMOUSE_ROTATE_V;
    else
        action = mjMOUSE_ZOOM;

    // move camera
    mjv_moveCamera(m, action, dx/height, dy/height, &scn, &cam);
}


// scroll callback
void scroll(GLFWwindow* window, double xoffset, double yoffset)
{
    // emulate vertical mouse motion = 5% of window height
    mjv_moveCamera(m, mjMOUSE_ZOOM, 0, -0.05*yoffset, &scn, &cam);
}


//****************************
//This function is called once and is used to get the headers
void init_save_data()
{
  //write name of the variable here (header)
   fprintf(fid,"t, ");

   //Don't remove the newline
   fprintf(fid,"\n");
}

//***************************
//This function is called at a set frequency, put data here
void save_data(const mjModel* m, mjData* d)
{
  //data here should correspond to headers in init_save_data()
  //seperate data by a space %f followed by space
  fprintf(fid,"%f ",d->time);

  //Don't remove the newline
  fprintf(fid,"\n");
}

/******************************/
void set_torque_control(const mjModel* m,int actuator_no,int flag)
{
  if (flag==0)
    m->actuator_gainprm[10*actuator_no+0]=0;
  else
    m->actuator_gainprm[10*actuator_no+0]=1;
}
/******************************/


/******************************/
void set_position_servo(const mjModel* m,int actuator_no,double kp)
{
  m->actuator_gainprm[10*actuator_no+0]=kp;
  m->actuator_biasprm[10*actuator_no+1]=-kp;
}
/******************************/

/******************************/
void set_velocity_servo(const mjModel* m,int actuator_no,double kv)
{
  m->actuator_gainprm[10*actuator_no+0]=kv;
  m->actuator_biasprm[10*actuator_no+2]=-kv;
}
/******************************/

//**************************
void init_controller(const mjModel* m, mjData* d)
{
 //

 d->qpos[4] = 0.5;
 d->ctrl[0] = d->qpos[4];
 //mj_forward(m,d);

 fsm_hip = fsm_leg2_swing;
 fsm_knee1 = fsm_knee1_stance;
 fsm_knee2 = fsm_knee2_stance;

 step_no = 0;

}

//**************************
void mycontroller(const mjModel* m, mjData* d)
{
  //write control here

  int body_no;
  double quat0, quatx, quaty, quatz;
  double euler_x, euler_y, euler_z;
  double abs_leg1, abs_leg2;
  double z_foot1, z_foot2;

  double x = d->qpos[0]; double vx = d->qvel[0];
  double z = d->qpos[1]; double vz = d->qvel[1];
  double q1 = d->qpos[2]; double u1 = d->qvel[2];
  double l1 = d->qpos[3]; double l1dot = d->qvel[3];
  double q2 = d->qpos[4]; double u2 = d->qvel[4];
  double l2 = d->qpos[5]; double l2dot = d->qvel[5];

  //state estimation
   body_no = 1;
   quat0 = d->xquat[4*body_no]; quatx = d->xquat[4*body_no+1];
   quaty = d->xquat[4*body_no+2]; quatz = d->xquat[4*body_no+3];
   quat2euler(quat0, quatx, quaty, quatz, &euler_x, &euler_y, &euler_z);
   //printf("Body = %d; euler angles = %f %f %f \n",body_no,euler_x,euler_y,euler_z);
   abs_leg1 = -euler_y;

   body_no = 3;
   quat0 = d->xquat[4*body_no]; quatx = d->xquat[4*body_no+1];
   quaty = d->xquat[4*body_no+2]; quatz = d->xquat[4*body_no+3];
   quat2euler(quat0, quatx, quaty, quatz, &euler_x, &euler_y, &euler_z);
   //printf("Body = %d; euler angles = %f %f %f \n",body_no,euler_x,euler_y,euler_z);
   abs_leg2 = -euler_y;

   //x = d->qpos[3*body_no]; y = d->qpos[3*body_no + 1];
   body_no = 2;
   z_foot1 = d->xpos[3*body_no+2];
   //printf("%f \n",z_foot1);

   body_no = 4;
   z_foot2 = d->xpos[3*body_no+2];

   //All transitions here
   if (fsm_hip == fsm_leg2_swing && z_foot2 < 0.05 && abs_leg1 < 0)
   {
     fsm_hip = fsm_leg1_swing;
     step_no +=1;
     printf("step_no = %d \n",step_no);
   }
   if (fsm_hip == fsm_leg1_swing && z_foot1 < 0.05 && abs_leg2 < 0)
   {
     fsm_hip = fsm_leg2_swing;
     step_no +=1;
     printf("step_no = %d \n",step_no);
   }

   if (fsm_knee1 == fsm_knee1_stance && z_foot2 < 0.05 && abs_leg1<0)
   {
     fsm_knee1 = fsm_knee1_retract;
   }
   if (fsm_knee1 == fsm_knee1_retract && abs_leg1 > 0.1)
   {
     fsm_knee1 = fsm_knee1_stance;
   }

   if (fsm_knee2 == fsm_knee2_stance && z_foot1 < 0.05 && abs_leg2<0)
   {
     fsm_knee2 = fsm_knee2_retract;
   }
   if (fsm_knee2 == fsm_knee2_retract && abs_leg2 > 0.1)
   {
     fsm_knee2 = fsm_knee2_stance;
   }


   //All actions here
   if (fsm_hip == fsm_leg1_swing)
   {
     d->ctrl[0] = -0.5;
   }

   if (fsm_hip == fsm_leg2_swing)
   {
     d->ctrl[0] = 0.5;
   }

   if (fsm_knee1 == fsm_knee1_stance)
   {
     d->ctrl[2] = 0;
   }
   if (fsm_knee1 == fsm_knee1_retract)
   {
     d->ctrl[2] = -0.25;
   }

   if (fsm_knee2 == fsm_knee2_stance)
   {
     d->ctrl[4] = 0;
   }
   if (fsm_knee2 == fsm_knee2_retract)
   {
     d->ctrl[4] = -0.25;
   }



  //write data here (dont change/dete this function call; instead write what you need to save in save_data)
  if ( loop_index%data_frequency==0)
    {
      save_data(m,d);
    }
  loop_index = loop_index + 1;
}


//************************
// main function
int main(int argc, const char** argv)
{

    // activate software
    mj_activate("mjkey.txt");

    char xmlpath[100]={};
    char datapath[100]={};

    strcat(xmlpath,path);
    strcat(xmlpath,xmlfile);

    strcat(datapath,path);
    strcat(datapath,datafile);


    // load and compile model
    char error[1000] = "Could not load binary model";

    // check command-line arguments
    if( argc<2 )
        m = mj_loadXML(xmlpath, 0, error, 1000);

    else
        if( strlen(argv[1])>4 && !strcmp(argv[1]+strlen(argv[1])-4, ".mjb") )
            m = mj_loadModel(argv[1], 0);
        else
            m = mj_loadXML(argv[1], 0, error, 1000);
    if( !m )
        mju_error_s("Load model error: %s", error);

    // make data
    d = mj_makeData(m);


    // init GLFW
    if( !glfwInit() )
        mju_error("Could not initialize GLFW");

    // create window, make OpenGL context current, request v-sync
    GLFWwindow* window = glfwCreateWindow(1244, 700, "Demo", NULL, NULL);
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);

    // initialize visualization data structures
    mjv_defaultCamera(&cam);
    mjv_defaultOption(&opt);
    mjv_defaultScene(&scn);
    mjr_defaultContext(&con);
    mjv_makeScene(m, &scn, 2000);                // space for 2000 objects
    mjr_makeContext(m, &con, mjFONTSCALE_150);   // model-specific context

    // install GLFW mouse and keyboard callbacks
    glfwSetKeyCallback(window, keyboard);
    glfwSetCursorPosCallback(window, mouse_move);
    glfwSetMouseButtonCallback(window, mouse_button);
    glfwSetScrollCallback(window, scroll);

    //double arr_view[] = {89.608063, -11.588379, 8, 0.000000, 0.000000, 2.000000}; //view the left side (for ll, lh, left_side)
    double arr_view[] = {120.893610, -15.810496, 8.000000, 0.000000, 0.000000, 2.000000};
    cam.azimuth = arr_view[0];
    cam.elevation = arr_view[1];
    cam.distance = arr_view[2];
    cam.lookat[0] = arr_view[3];
    cam.lookat[1] = arr_view[4];
    cam.lookat[2] = arr_view[5];

    // install control callback
    mjcb_control = mycontroller;

    fid = fopen(datapath,"w");
    init_save_data();
    init_controller(m,d);

    double gamma = 0.1; //ramp slope
    double gravity = 9.81;
    m->opt.gravity[2] = -gravity*cos(gamma);
    m->opt.gravity[0] = gravity*sin(gamma);

    // use the first while condition if you want to simulate for a period.
    while( !glfwWindowShouldClose(window))
    {
        // advance interactive simulation for 1/60 sec
        //  Assuming MuJoCo can simulate faster than real-time, which it usually can,
        //  this loop will finish on time for the next frame to be rendered at 60 fps.
        //  Otherwise add a cpu timer and exit this loop when it is time to render.
        mjtNum simstart = d->time;
        while( d->time - simstart < 1.0/60.0 )
        {
            mj_step(m, d);
        }

        if (d->time>=simend)
        {
           fclose(fid);
           break;
         }

       // get framebuffer viewport
        mjrRect viewport = {0, 0, 0, 0};
        glfwGetFramebufferSize(window, &viewport.width, &viewport.height);

        //opt.frame = mjFRAME_WORLD; //mjFRAME_BODY
        //opt.flags[mjVIS_COM]  = 1 ; //mjVIS_JOINT;
        opt.flags[mjVIS_JOINT]  = 1 ;
          // update scene and render
        mjv_updateScene(m, d, &opt, NULL, &cam, mjCAT_ALL, &scn);
        mjr_render(viewport, &scn, &con);
        //int body = 2;
        //cam.lookat[0] = d->xpos[3*body+0];
        cam.lookat[0] = d->qpos[0];
        //printf("{%f, %f, %f, %f, %f, %f};\n",cam.azimuth,cam.elevation, cam.distance,cam.lookat[0],cam.lookat[1],cam.lookat[2]);

        // swap OpenGL buffers (blocking call due to v-sync)
        glfwSwapBuffers(window);

        // process pending GUI events, call GLFW callbacks
        glfwPollEvents();

    }

    // free visualization storage
    mjv_freeScene(&scn);
    mjr_freeContext(&con);

    // free MuJoCo model and data, deactivate
    mj_deleteData(d);
    mj_deleteModel(m);
    mj_deactivate();

    // terminate GLFW (crashes with Linux NVidia drivers)
    #if defined(__APPLE__) || defined(_WIN32)
        glfwTerminate();
    #endif

    return 1;
}

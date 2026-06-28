
#ifdef WIIMOTE_MOD_ACTIVATED

#include <wiiuse.h>
#include <thread>

#include "qcommon/qcommon.h"
#include "qcommon/q_shared.h"
#include "rd-common/tr_types.h"

#include "sys/sys_local.h"
#include "client/client.h"

// UI
void UI_UpdateWiimoteStatus();

#define MAX_WIIMOTES 4

static wiimote** i_wiimotes =  nullptr;
static wiimote* i_wiimote = nullptr;


static std::thread* con_thread = nullptr;

static int status = 0;

static int prevIrX = 0;
static int prevIrY = 0;

//------------------------------------------------------------------------------
void wiimote_init() {

  i_wiimotes =  wiiuse_init(MAX_WIIMOTES);

  Com_Printf("Wiimote initialization\n");

  status = 0;
  Cvar_SetValue("cl_wiimotestatus", 0.f);
  UI_UpdateWiimoteStatus();

}


//------------------------------------------------------------------------------
void wiimote_connect_thread() {

  Cvar_SetValue("cl_wiimotestatus", 1.f);
  UI_UpdateWiimoteStatus();

  status = 1;

  bool find = wiiuse_find(i_wiimotes, MAX_WIIMOTES, 5);
  Com_Printf("after wiimote found\n");

  if(find) {

    bool conn = wiiuse_connect(i_wiimotes, MAX_WIIMOTES);
    Com_Printf("after connection\n");

    status = 2;
    Cvar_SetValue("cl_wiimotestatus", 2.f);
    UI_UpdateWiimoteStatus();

    if(conn) {

      Com_Printf("wiimote connection res: \n");

      // usleep(300000);
      // Sys_Sleep(3000); 

      for(int i = 0; i < MAX_WIIMOTES; i ++) {
    		if(i_wiimotes[i] && WIIMOTE_IS_CONNECTED(i_wiimotes[i]) ) {
    			i_wiimote = i_wiimotes[i];
          break;
    		}
      }

      wiiuse_set_ir_vres(i_wiimote, 640, 480);

      wiiuse_set_leds(i_wiimote, WIIMOTE_LED_2);
      wiiuse_set_leds(i_wiimote, WIIMOTE_LED_3);

      // set ir resolution
      // wiiuse_set_ir(i_wiimote, 1);

      // activate motions
      // wiiuse_motion_sensing(i_wiimote, 1);

      Cvar_SetValue("cl_wiimotestatus", 3.f);
      status = 3;
      UI_UpdateWiimoteStatus();
      
    }

    if(!find || !conn) {
      Com_Printf("Wiimote connection not successful :(\n");
      Cvar_SetValue("cl_wiimotestatus", 0.f);
      UI_UpdateWiimoteStatus();
    }

  }

  if(!find) {
    Com_Printf("Wiimote connection not successful :(\n");
    Cvar_SetValue("cl_wiimotestatus", 0.f);
    UI_UpdateWiimoteStatus();
  }


}


//------------------------------------------------------------------------------
void wiimote_connect() {

  if(con_thread != nullptr) {
    if(con_thread->joinable()) {
      con_thread->join();
      delete con_thread;
      con_thread = nullptr;
    }
  }

  if(con_thread == nullptr) {
    con_thread = new std::thread(wiimote_connect_thread);
  }

}

//------------------------------------------------------------------------------
void wiimote_buttonEvents() {

  struct nunchuk_t* nc = (nunchuk_t*)&i_wiimote->exp.nunchuk;

  // handle button events here

  // button pressed
  if (IS_JUST_PRESSED(i_wiimote, WIIMOTE_BUTTON_A)) {
    Sys_QueEvent( 0, SE_KEY, A_WIIMOTEA, qtrue, 0, NULL );
  }
  if (IS_JUST_PRESSED(i_wiimote, WIIMOTE_BUTTON_B)) {
    Sys_QueEvent( 0, SE_KEY, A_WIIMOTEB, qtrue, 0, NULL );
  }
  if (IS_JUST_PRESSED(i_wiimote, WIIMOTE_BUTTON_UP)) {
    Sys_QueEvent( 0, SE_KEY, A_WIIMOTEUP, qtrue, 0, NULL );
  }
  if (IS_JUST_PRESSED(i_wiimote, WIIMOTE_BUTTON_DOWN)) {
    Sys_QueEvent( 0, SE_KEY, A_WIIMOTEDOWN, qtrue, 0, NULL );
  }
  if (IS_JUST_PRESSED(i_wiimote, WIIMOTE_BUTTON_LEFT)) {
    Sys_QueEvent( 0, SE_KEY, A_WIIMOTELEFT, qtrue, 0, NULL );
  }
  if (IS_JUST_PRESSED(i_wiimote, WIIMOTE_BUTTON_RIGHT)) {
    Sys_QueEvent( 0, SE_KEY, A_WIIMOTERIGHT, qtrue, 0, NULL );
  }
  if (IS_JUST_PRESSED(i_wiimote, WIIMOTE_BUTTON_PLUS)) {
    Sys_QueEvent( 0, SE_KEY, A_WIIMOTEPLUS, qtrue, 0, NULL );
  }
  if (IS_JUST_PRESSED(i_wiimote, WIIMOTE_BUTTON_MINUS)) {
    Sys_QueEvent( 0, SE_KEY, A_WIIMOTEMINUS, qtrue, 0, NULL );
  }
  if (IS_JUST_PRESSED(i_wiimote, WIIMOTE_BUTTON_HOME)) {
    Sys_QueEvent( 0, SE_KEY, A_WIIMOTEHOME, qtrue, 0, NULL );
  }
  if (IS_JUST_PRESSED(i_wiimote, WIIMOTE_BUTTON_ONE)) {
    Sys_QueEvent( 0, SE_KEY, A_WIIMOTE1, qtrue, 0, NULL );
  }
  if (IS_JUST_PRESSED(i_wiimote, WIIMOTE_BUTTON_TWO)) {
    Sys_QueEvent( 0, SE_KEY, A_WIIMOTE2, qtrue, 0, NULL );
  }
  if (IS_JUST_PRESSED(nc, NUNCHUK_BUTTON_C)) {
    Sys_QueEvent( 0, SE_KEY, A_NUNCHUKC, qtrue, 0, NULL );
  }
  if (IS_JUST_PRESSED(nc, NUNCHUK_BUTTON_Z)) {
    Sys_QueEvent( 0, SE_KEY, A_NUNCHUKZ, qtrue, 0, NULL );
  }

  // Button realeased
  if (IS_RELEASED(i_wiimote, WIIMOTE_BUTTON_A)) {
    Sys_QueEvent( 0, SE_KEY, A_WIIMOTEA, qfalse, 0, NULL );
  }
  if (IS_RELEASED(i_wiimote, WIIMOTE_BUTTON_B)) {
    Sys_QueEvent( 0, SE_KEY, A_WIIMOTEB, qfalse, 0, NULL );
  }
  if (IS_RELEASED(i_wiimote, WIIMOTE_BUTTON_UP)) {
    Sys_QueEvent( 0, SE_KEY, A_WIIMOTEUP, qfalse, 0, NULL );
  }
  if (IS_RELEASED(i_wiimote, WIIMOTE_BUTTON_DOWN)) {
    Sys_QueEvent( 0, SE_KEY, A_WIIMOTEDOWN, qfalse, 0, NULL );
  }
  if (IS_RELEASED(i_wiimote, WIIMOTE_BUTTON_LEFT)) {
    Sys_QueEvent( 0, SE_KEY, A_WIIMOTELEFT, qfalse, 0, NULL );
  }
  if (IS_RELEASED(i_wiimote, WIIMOTE_BUTTON_RIGHT)) {
    Sys_QueEvent( 0, SE_KEY, A_WIIMOTERIGHT, qfalse, 0, NULL );
  }
  if (IS_RELEASED(i_wiimote, WIIMOTE_BUTTON_PLUS)) {
    Sys_QueEvent( 0, SE_KEY, A_WIIMOTEPLUS, qfalse, 0, NULL );
  }
  if (IS_RELEASED(i_wiimote, WIIMOTE_BUTTON_MINUS)) {
    Sys_QueEvent( 0, SE_KEY, A_WIIMOTEMINUS, qfalse, 0, NULL );
  }
  if (IS_RELEASED(i_wiimote, WIIMOTE_BUTTON_HOME)) {
    Sys_QueEvent( 0, SE_KEY, A_WIIMOTEHOME, qfalse, 0, NULL );
  }
  if (IS_RELEASED(i_wiimote, WIIMOTE_BUTTON_ONE)) {
    Sys_QueEvent( 0, SE_KEY, A_WIIMOTE1, qfalse, 0, NULL );
  }
  if (IS_RELEASED(i_wiimote, WIIMOTE_BUTTON_TWO)) {
    Sys_QueEvent( 0, SE_KEY, A_WIIMOTE2, qfalse, 0, NULL );
  }
  if (IS_RELEASED(nc, NUNCHUK_BUTTON_C)) {
    Sys_QueEvent( 0, SE_KEY, A_NUNCHUKC, qfalse, 0, NULL );
  }
  if (IS_RELEASED(nc, NUNCHUK_BUTTON_Z)) {
    Sys_QueEvent( 0, SE_KEY, A_NUNCHUKZ, qfalse, 0, NULL );
  }

}

//------------------------------------------------------------------------------
void wiimote_pollEvents() {

  if(status == 3) {
    while(wiiuse_poll(i_wiimotes, MAX_WIIMOTES)) {
      if(i_wiimote->event == WIIUSE_EVENT) {

        wiimote_buttonEvents();

        // handle wiimote ir

        int irX = i_wiimote->ir.x;
        int irY = i_wiimote->ir.y;

        int irDx = irX - prevIrX;
        int irDy = irY - prevIrY;

        prevIrX = irX;
        prevIrY = irY;

        // Sys_QueEvent( 0, SE_WIIMOTEIR, irDx, irDy, 0, NULL );

        // nunchuk joystick
        // struct nunchuk_t* nc = (nunchuk_t*)&i_wiimote->exp.nunchuk;
        // int axis = 0;
        // Sys_QueEvent( 0, SE_JOYSTICK_AXIS, axis, nc->js.x*300, 0, NULL );
        // axis = 1;
        // Sys_QueEvent( 0, SE_JOYSTICK_AXIS, axis, nc->js.y*300, 0, NULL );

        // Com_Printf("wiimote joystick: %f, %f\n", nc->js.x, nc->js.y);



        // handle wiimote motion

        // handle nunchuk motion

      }
    }
  }



}

//------------------------------------------------------------------------------
void wiimote_getIr(int* x, int* y) {
  if(status == 3) {
    *x = i_wiimote->ir.x;
    *y = i_wiimote->ir.y;
    // Com_Printf("wiimote ir wiiuse: %d, %d\n", *x, *y);
  } else {
    *x = 0;
    *y = 0;
    // TODO TOREMOVE
    // wiiuse_set_ir(i_wiimote, 1);

  }
 
}

//------------------------------------------------------------------------------
void wiimote_clean() {

  wiiuse_cleanup(i_wiimotes, MAX_WIIMOTES);

  status = 0;
  Cvar_SetValue("cl_wiimotestatus", 0.f);

  Com_Printf("Wiimote cleaning\n");

  if(con_thread != nullptr) {
    if(con_thread->joinable()) {
      con_thread->join();
    }
    delete con_thread;
    con_thread = nullptr;
  }

}

#endif // WIIMOTE_MOD_ACTIVATED
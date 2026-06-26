#pragma once

#ifdef WIIMOTE_MOD_ACTIVATED

void wiimote_init();

void wiimote_connect();

void wiimote_pollEvents();

void wiimote_getIr(int* x, int* y);

void wiimote_clean();

#endif // WIIMOTE_MOD_ACTIVATED
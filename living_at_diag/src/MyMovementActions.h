#ifndef _MY_MOVEMENT_ACTIONS_
#define _MY_MOVEMENT_ACTIONS_

// Action implementation

void followCorridor(string robot_name, string params, bool *run);
void followPerson(string robot_name, string params, bool *run);
void getCloser(string robot_name, string params, bool *run);
void gotopose(string robot_name, string params, bool *run);
void goAndLookAt(string robot_name, string params, bool *run);
void home(string robot_name, string params, bool *run);
void turn(string robot_name, string params, bool *run);

#endif


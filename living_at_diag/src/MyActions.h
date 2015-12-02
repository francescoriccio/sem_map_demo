#ifndef _MY_ACTIONS_
#define _MY_ACTIONS_

// Action implementation

void evaluateCondition(string robot_name, string params, bool *run);
void forget(string robot_name, string params, bool *run);
void generatePlan(string robot_name, string params, bool *run);
void groundAtom(string robot_name, string params, bool *run);
void groundGoTo(string robot_name, string params, bool *run);
void insertAction(string robot_name, string params, bool *run);
void memQueryLoop(string robot_name, string params, bool *run);
void removeAndFeedback(string robot_name, string params, bool *run);
void removeAction(string robot_name, string params, bool *run);
void replaceAction(string robot_name, string params, bool *run);
void say(string robot_name, string params, bool *run);
void takePicture(string robot_name, string params, bool *run);
void waitForUser(string robot_name, string params, bool *run);

#endif


:- dynamic combinedPlan/1, plan/3, basicPlan/1, pcl/2.

%since this plan names are PNP subplans, combine_pnp will take every atom and capitalize it (i.e. goTo -> GoTo)
%plan has a list of input parameters and a list of output parameters
%basicPlans are the PNP subplans that have been written by a human while combinedPlans are the ones generated through HRI.

plan(followMe, [], []).
plan(forget, [obj], []).
plan(forgetPlan, [planName], []).
plan(getCloser, [], []).
plan(goTo, [obj], []).
plan(home, [], []).
plan(learnPlan, [], []).
plan(look, [dir, back], []).
plan(memorize, [], []).
plan(recognize, [], []).
plan(say, [text], []).
plan(takePicture, [], [pictureName]).
plan(tellObjInSight, [], []).
plan(turn, [dir], []).
plan(update, [obj], []).
plan(waitForUser, [], []).
plan(pickUp, [object], []).
plan(drop, [object], []).
plan(sendEmail,[file, person],[]).

basicPlan(followMe).
basicPlan(forget).
basicPlan(forgetPlan).
basicPlan(getCloser).
basicPlan(goTo).
basicPlan(home).
basicPlan(learnPlan).
basicPlan(look).
basicPlan(memorize).
basicPlan(recognize).
basicPlan(say).
basicPlan(takePicture).
basicPlan(tellObjInSight).
basicPlan(turn).
basicPlan(update).
basicPlan(waitForUser).
basicPlan(pickUp).
basicPlan(drop).
basicPlan(sendEmail).


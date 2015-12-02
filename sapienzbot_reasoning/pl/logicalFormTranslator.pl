%##################################################
% Constants
%##################################################

char(concatenation, X) :- X = '_'. %multiple word objects are called like fire_extinguisher

%##################################################
% grammar
%##################################################
getAtom(List, Answer, RX, RY) :- retractall(robotPos(_,_)), assert(robotPos(RX,RY)), s(Answer, List, []).

s(ObjAtom) --> n(ObjAtom), rel(ObjAtom).
s(ObjAtom) --> pp(ObjAtom), rel(ObjAtom).

pp(S)    --> [P], n(S), {p(P)}.
n(S)     --> [D], n(S), {det(D)}.
n(S)     --> [A], {objectType(S,A)}.
n(S)     --> [A], {roomType(S,A)}.
n(S)     --> [S], {roomType(S,_)}.
n(S)     --> [A], [B], {char(concatenation, X), atom_concat(A, X, Temp), atom_concat(Temp, B, Type),  objectType(S, Type)}.

% needed to parse n (e.g. to closet)
rel(_)   --> [].

% needed to parse n near n (e.g. bed near couch)
rel(S)   --> [Rel], n(ObjAtom2), {call(Rel,S,ObjAtom2), rel(Rel)}.

% needed to parse n rel p n (e.g. bed close to couch)
rel(S)   --> [Rel], [P], n(ObjAtom2), {atom_concat(Rel, P,ToRel), catch(call(ToRel,S,ObjAtom2), E, return_error(E)), rel(ToRel), p(P)}.

% needed to parse n p rel p n (e.g. bed in front of to couch)
rel(S)   --> [P1], [Rel], [P], n(ObjAtom2), {atom_concat(Rel, P,ToRel), catch(call(ToRel,S,ObjAtom2), E, return_error(E)), rel(ToRel), p(P), p(P1)}.

% needed to parse n p det rel p n (e.g. bed to the right of couch)
rel(S)   --> [P1], [D], [Rel], [P], n(ObjAtom2), {atom_concat(Rel, P,ToRel), catch(call(ToRel,S,ObjAtom2), E, return_error(E)), rel(ToRel), p(P), p(P1), det(D)}.

return_error(_) :- false.

det(the).
det(a).
det(an).

p(to).
p(on).
p(at).
p(of).
p(in).
p(near).

rel(nextto).
rel(behind).
rel(near).
rel(in).
rel(closeto).
rel(leftof).
rel(rightof).
rel(frontof).

synonym(socket,socket).
synonym(plug,socket).
synonym(power_socket,socket).

isA(stationery_closet,closet).
isA(documentation_closet,closet).
isA(utility_closet,closet).
isA(storage_closet,closet).
isA(book_closet,closet).

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Support grounding functions
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

targetType(Atom, Type, room):- roomType(Atom, Type). 
targetType(Atom, Type, object):- objectType(Atom, Type).
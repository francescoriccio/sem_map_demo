constant(squared_ditance_treshold, X) :- X is 25.
constant(squared_ditance_lower_treshold, X) :- X is 4.

constant(threshold_near, X) :- X is 10.
constant(threshold_next_to, X) :- X is 2.

%nearest(X,Y,N,objectType,Atom).
%N = set to 0
%X,Y = the starting coordinates of the search (the coordinates of the robot)
%objectType = the type of object to search for
%Atom = the variable to be bind with the atom of the searched object

nearest(X,Y,N,Type,Atom) :- NewStartX is X-N, NewEndX is X+N,
                        NewStartY is Y-N, NewEndY is Y-N,
                        findInRow(NewStartX,NewStartY,NewEndX,NewEndY,Type,Atom), !.
nearest(X,Y,N,Type,Atom) :- NewStartX is X+N, NewEndX is X+N,
                        NewStartY is Y-N, NewEndY is Y+N,
                        findInRow(NewStartX,NewStartY,NewEndX,NewEndY,Type,Atom), !. 
nearest(X,Y,N,Type,Atom) :- NewStartX is X-N, NewEndX is X+N,
                        NewStartY is Y+N, NewEndY is Y+N,
                        findInRow(NewStartX,NewStartY,NewEndX,NewEndY,Type,Atom), !. 
nearest(X,Y,N,Type,Atom) :- NewStartX is X-N, NewEndX is X-N, 
                        NewStartY is Y-N, NewEndY is Y+N, 
                        findInRow(NewStartX,NewStartY,NewEndX,NewEndY,Type,Atom), !. 
nearest(X,Y,N,Type,Atom) :- N1 is N+1,
                        nearest(X,Y,N1,Type,Atom), !.

findInRow(StartX,StartY,_,_,Type,Atom) :- checkCell(StartX,StartY,Type,Atom).
findInRow(StartX,StartY,EndX,EndY,Type,Atom) :- StartY == EndY, StartX < EndX, StartX1 is StartX+1, 
                                      findInRow(StartX1,StartY,EndX,EndY,Type,Atom).

findInRow(StartX,StartY,EndX,EndY,Type,Atom) :- StartX == EndX, StartY < EndY, StartY1 is StartY+1,
                                      findInRow(StartX,StartY1,EndX,EndY,Type,Atom).

checkCell(X,Y,Type,Atom) :- objectType(Atom,Type), objectPos(Atom,X,Y,_).

%objA behind objB from robot's point of view
behind(ObjAtom, ObjAtom2) :- robotPos(RX,RY), objectType(ObjAtom, ObjType), objectType(ObjAtom2, ObjType2), behind(ObjType, ObjType2, ObjAtom, ObjAtom2,RX,RY).
behind(ObjA, ObjB, AtomA, AtomB, RX, RY) :- objectType(AtomB,ObjB), objectType(AtomA,ObjA), AtomA \= AtomB,
                                  objectPos(AtomA,XA,YA,_), objectPos(AtomB,XB,YB,_), constant(squared_ditance_treshold, THR), constant(squared_ditance_lower_treshold, LTHR),
                                  ( (RX < XB, XB =< XA, (XA - XB)*(XA - XB) =< THR, (YA-YB)*(YA-YB) =< LTHR);
                                    (RX > XB, XB >= XA, (XA - XB)*(XA - XB) =< THR, (YA-YB)*(YA-YB) =< LTHR);
                                    (RY < YB, YB =< YA, (YB - YA)*(YB - YA) =< THR, (XA-XB)*(XA-XB) =< LTHR);
                                    (RY > YB, YB >= YA, (YB - YA)*(YB - YA) =< THR, (XA-XB)*(XA-XB) =< LTHR)).

%objA infront objB from robot's point of view
frontof(ObjAtom, ObjAtom2) :- robotPos(RX,RY), objectType(ObjAtom, ObjType), objectType(ObjAtom2, ObjType2), infront(ObjType, ObjType2, ObjAtom, ObjAtom2,RX,RY).
infront(ObjA, ObjB, AtomA, AtomB, RX, RY) :- objectType(AtomB,ObjB), objectType(AtomA,ObjA), AtomA \= AtomB,
                                            objectPos(AtomA,XA,YA,_), objectPos(AtomB,XB,YB,_), constant(squared_ditance_treshold, THR), constant(squared_ditance_lower_treshold, LTHR),
                                            ( (RX < XA, XA =< XB, (XA - XB)*(XA - XB) =< THR, (YA-YB)*(YA-YB) =< LTHR);
                                              (RX > XA, XA >= XB, (XA - XB)*(XA - XB) =< THR, (YA-YB)*(YA-YB) =< LTHR);
                                              (RY < YA, YA =< YB, (YB - YA)*(YB - YA) =< THR, (XA-XB)*(XA-XB) =< LTHR);
                                              (RY > YA, YA >= YB, (YB - YA)*(YB - YA) =< THR, (XA-XB)*(XA-XB) =< LTHR)).  

%objects near to eachother: objects that are in cells with a distance =< 3
closeto(AtomA, AtomB) :- near(AtomA, AtomB).
near(ObjAtom, ObjAtom2) :-  objectType(ObjAtom, ObjType), objectType(ObjAtom2, ObjType2), near(ObjType, ObjType2, ObjAtom, ObjAtom2).
near(ObjA, ObjB, AtomA, AtomB) :- objectType(AtomB,ObjB), objectType(AtomA,ObjA), AtomA \= AtomB,
                                  objectPos(AtomA,XA,YA,_), objectPos(AtomB,XB,YB,_),  constant(squared_ditance_treshold, THR),
                                  C1 is (XA-XB)*(XA-XB), C2 is (YA-YB)*(YA-YB), C1+C2 =< THR.

nextto(ObjAtom, ObjAtom2) :-  objectType(ObjAtom, ObjType), objectType(ObjAtom2, ObjType2), next(ObjType, ObjType2, ObjAtom, ObjAtom2).
next(ObjA, ObjB, AtomA, AtomB) :- objectType(AtomB,ObjB), objectType(AtomA,ObjA), AtomA \= AtomB,
                                  objectPos(AtomA,XA,YA,_), objectPos(AtomB,XB,YB,_), constant(squared_ditance_lower_treshold, LTHR),
                                  (XA-XB)*(XA-XB) =< LTHR, (YA-YB)*(YA-YB) =< LTHR.

%objA leftof objB from robot's point of view
leftof(ObjAtom, ObjAtom2) :-  robotPos(RX,RY), objectType(ObjAtom, ObjType), objectType(ObjAtom2, ObjType2), leftof(ObjType, ObjType2, ObjAtom, ObjAtom2, RX, RY).
leftof(ObjA, ObjB, AtomA, AtomB, RX, RY) :- objectType(AtomB,ObjB), objectType(AtomA,ObjA), AtomA \= AtomB,
                                            objectPos(AtomA,XA,YA,_), objectPos(AtomB,XB,YB,_), constant(squared_ditance_treshold, THR), constant(squared_ditance_lower_treshold, LTHR),
                                            ( (min(XA,XB,C), RX < C, YA =< YB, (XA - XB)*(XA - XB) =< LTHR, (YA-YB)*(YA-YB) =< THR);
                                              (max(XA,XB,C), RX > C, YA >= YB, (XA - XB)*(XA - XB) =< LTHR, (YA-YB)*(YA-YB) =< THR);
                                              (min(YA,YB,C), RY < C, XA >= XB, (XA - XB)*(XA - XB) =< THR, (YA-YB)*(YA-YB) =< LTHR);
                                              (max(YA,YB,C), RY > C, XA =< XB, (XA - XB)*(XA - XB) =< THR, (YA-YB)*(YA-YB) =< LTHR)).

%objA rightof objB from robot's point of view
rightof(ObjAtom, ObjAtom2) :- robotPos(RX,RY), objectType(ObjAtom, ObjType), objectType(ObjAtom2, ObjType2), rightof(ObjType, ObjType2, ObjAtom, ObjAtom2,RX,RY).
rightof(ObjA, ObjB, AtomA, AtomB, RX, RY) :- objectType(AtomB,ObjB), objectType(AtomA,ObjA), AtomA \= AtomB, constant(squared_ditance_lower_treshold, LTHR),
                                            objectPos(AtomA,XA,YA,_), objectPos(AtomB,XB,YB,_), constant(squared_ditance_treshold, THR),
                                            ( (min(XA,XB,C), RX < C, YA >= YB, (XA - XB)*(XA - XB) =< LTHR, (YA-YB)*(YA-YB) =< THR);
                                              (max(XA,XB,C), RX > C, YA =< YB, (XA - XB)*(XA - XB) =< LTHR, (YA-YB)*(YA-YB) =< THR);
                                              (min(YA,YB,C), RY < C, XA =< XB, (XA - XB)*(XA - XB) =< THR, (YA-YB)*(YA-YB) =< LTHR);
                                              (max(YA,YB,C), RY > C, XA >= XB, (XA - XB)*(XA - XB) =< THR, (YA-YB)*(YA-YB) =< LTHR)).

min(A,B,C) :- C is B, B=<A.
min(A,B,C) :- C is A, A<B.

max(A,B,C) :- C is A, B=<A.
max(A,B,C) :- C is B, A<B.

objectOf(A,B,Atom,Batom) :- objectType(Atom,A), objectPos(Atom,X,Y,_), roomType(Batom,B), cellCoordIsPartOf(X,Y,Batom).
objectOf(A,B,Atom,Batom) :- synonym(A,SynA), objectType(Atom,SynA), objectPos(Atom,X,Y,_), roomType(Batom,B), cellCoordIsPartOf(X,Y,Batom).
roomOf(A,B,Atom) :- roomType(Atom,A), roomSpec(Atom,B).

allObjectsInRoom(XR,YR,L) :- findall(X, objectInRoom(XR,YR,X), L).
objectInRoom(XR, YR, [O,XO,YO]) :- objectPos(O,XO,YO,_), cellCoordIsPartOf(XR,YR,Room), cellCoordIsPartOf(XO,YO,Room).
:- dynamic objectPos/4, objectType/2, roomType/2, roomSpec/2.

connected(X,Y) :- edge(X,Y) ; edge(Y,X); X==Y.

path(A,B,Path) :- travel(A,B,[A],Q), reverse(Q,Path).

travel(A,B,P,[B|P]) :- connected(A,B).
travel(A,B,Visited,Path) :- connected(A,C), C \== B, \+member(C,Visited), travel(C,B,[C|Visited],Path).  

objectOf(ObjectType,Room,ObjectAtom) :- objectType(ObjectAtom,ObjectType), objectPos(ObjectAtom, X, Y, _), cellCoordIsPartOf(X,Y,Room).

objectType(X,printer) :- objectType(X,laserprinter).

roomType(X,room):-roomType(X,office).

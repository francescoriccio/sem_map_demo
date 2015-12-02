inFrontOf(Object,pos(R,CTarg)) :- objectPos(Object,R,CObj,_), CTarg is CObj+1,
			cellCoordIsPartOf(R,CTarg,Room), cellCoordIsPartOf(R,CObj,Room), 
			not(objectPos(_,R,CTarg,_)).		

inFrontOf(Object,pos(R,CTarg)) :- objectPos(Object,R,CObj,_), CTarg is CObj-1,
			cellCoordIsPartOf(R,CTarg,Room), cellCoordIsPartOf(R,CObj,Room), 
			not(objectPos(_,R,CTarg,_)).	

inFrontOf(Object,pos(RTarg,C)) :- objectPos(Object,RObj,C,_), RTarg is RObj+1,
			cellCoordIsPartOf(RTarg,C,Room), cellCoordIsPartOf(RObj,C,Room), 
			not(objectPos(_,RTarg,C,_)).	
			
inFrontOf(Object,pos(RTarg,C)) :- objectPos(Object,RObj,C,_), RTarg is RObj-1,
			cellCoordIsPartOf(RTarg,C,Room), cellCoordIsPartOf(RObj,C,Room), 
			not(objectPos(_,RTarg,C,_)).
			

%next_to = near
next(ObjA,to(ObjB,_),AtomA,AtomB) :- near(ObjA,ObjB,AtomA,AtomB).
%check the same square
near(ObjA,ObjB,AtomA,AtomB) :- objectType(AtomB,ObjB), objectPos(AtomB,X,Y,_), objectType(AtomA,ObjA), objectPos(AtomA,X,Y,_).
%check the upper-left square
near(ObjA,ObjB,AtomA,AtomB) :- objectType(AtomB,ObjB), objectPos(AtomB,X,Y,_), X1 is X-1, Y1 is Y-1, objectType(AtomA,ObjA), objectPos(AtomA,X1,Y1,_),
				cellCoordIsPartOf(X,Y,AtomRoom), cellCoordIsPartOf(X1,Y1,AtomRoom).
%check the upper-center square
near(ObjA,ObjB,AtomA,AtomB) :- objectType(AtomB,ObjB), objectPos(AtomB,X,Y,_), X1 is X, Y1 is Y-1, objectType(AtomA,ObjA), objectPos(AtomA,X1,Y1,_),
				cellCoordIsPartOf(X,Y,AtomRoom), cellCoordIsPartOf(X1,Y1,AtomRoom).
%check the upper-right square
near(ObjA,ObjB,AtomA,AtomB) :- objectType(AtomB,ObjB), objectPos(AtomB,X,Y,_), X1 is X+1, Y1 is Y-1, objectType(AtomA,ObjA), objectPos(AtomA,X1,Y1,_),
				cellCoordIsPartOf(X,Y,AtomRoom), cellCoordIsPartOf(X1,Y1,AtomRoom).
%check the left square
near(ObjA,ObjB,AtomA,AtomB) :- objectType(AtomB,ObjB), objectPos(AtomB,X,Y,_), X1 is X-1, Y1 is Y, objectType(AtomA,ObjA), objectPos(AtomA,X1,Y1,_),
				cellCoordIsPartOf(X,Y,AtomRoom), cellCoordIsPartOf(X1,Y1,AtomRoom).
%check the right square
near(ObjA,ObjB,AtomA,AtomB) :- objectType(AtomB,ObjB), objectPos(AtomB,X,Y,_), X1 is X+1, Y1 is Y, objectType(AtomA,ObjA), objectPos(AtomA,X1,Y1,_),
				cellCoordIsPartOf(X,Y,AtomRoom), cellCoordIsPartOf(X1,Y1,AtomRoom).
%check the lower-left square
near(ObjA,ObjB,AtomA,AtomB) :- objectType(AtomB,ObjB), objectPos(AtomB,X,Y,_), X1 is X-1, Y1 is Y+1, objectType(AtomA,ObjA), objectPos(AtomA,X1,Y1,_),
				cellCoordIsPartOf(X,Y,AtomRoom), cellCoordIsPartOf(X1,Y1,AtomRoom).
%check the lower-center square
near(ObjA,ObjB,AtomA,AtomB) :- objectType(AtomB,ObjB), objectPos(AtomB,X,Y,_), X1 is X, Y1 is Y+1, objectType(AtomA,ObjA), objectPos(AtomA,X1,Y1,_),
				cellCoordIsPartOf(X,Y,AtomRoom), cellCoordIsPartOf(X1,Y1,AtomRoom).
%check the lower-right square
near(ObjA,ObjB,AtomA,AtomB) :- objectType(AtomB,ObjB), objectPos(AtomB,X,Y,_), X1 is X+1, Y1 is Y+1, objectType(AtomA,ObjA), objectPos(AtomA,X1,Y1,_),
				cellCoordIsPartOf(X,Y,AtomRoom), cellCoordIsPartOf(X1,Y1,AtomRoom).

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

allOutOfMap(cell(X1,Y1),cell(X2,Y2),cell(X3,Y3),cell(X4,Y4)):- \+cellCoordIsPartOf(X1,Y1,_), \+cellCoordIsPartOf(X2,Y2,_), \+cellCoordIsPartOf(X3,Y3,_), \+cellCoordIsPartOf(X4,Y4,_).

allObjectsInRoom(XR,YR,L) :- findall(X, objectInRoom(XR,YR,X), L).
objectInRoom(XR, YR, [O,XO,YO]) :- objectPos(O,XO,YO,_), cellCoordIsPartOf(XR,YR,Room), cellCoordIsPartOf(XO,YO,Room).
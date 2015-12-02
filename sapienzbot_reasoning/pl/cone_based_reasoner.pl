% robot pose supposed to be given in real world coordinates
% thresholds are given in cell units. 
% when thresholds needed, used cell coords, reals otherwise

%##################################################
% Constants
%##################################################

constant(threshold_near, X) :- X is 5.
constant(threshold_next_to, X) :- X is 2.
constant(desired_distance_from_obj, X) :- X is 2.

%##################################################
% unary spatial relations
%##################################################

% cell in front of atom (metric map coordinates)
front_of_obj(Atom, X, Y) :- findall(X, front(Atom, X, _), LX), findall(Y, front(Atom, _, Y), LY), 
                            average(LX, X), average(LY, Y).
front(Atom, X, Y) :- objectAngle(Atom, Angle), constant(desired_distance_from_obj, D), objectPos(Atom, XA, YA, _),
			       (( 315 =< Angle, Angle < 45, XC is XA+D, YC is YA);
			        ( 45 =< Angle, Angle < 135, XC is XA, YC is YA-D);
			        ( 135 =< Angle, Angle < 225, XC is XA-D, YC is YA);
			        ( 225 =< Angle, Angle < 315, XC is XA, YC is YA+D)),
			       cellCenterInMap(XC, YC, X, Y), cellCoordIsPartOf(XC, YC, _).

% cell to the left of atom (metric map coordinates)
left_of_obj(Atom, X, Y) :- findall(X, left(Atom, X, _), LX), findall(Y, left(Atom, _, Y), LY), 
                            average(LX, X), average(LY, Y).
left(Atom, X, Y) :- objectAngle(Atom, Angle), constant(desired_distance_from_obj, D), objectPos(Atom, XA, YA, _),
			       (( 315 =< Angle, Angle < 45, XC is XA, YC is YA-D);
			        ( 45 =< Angle, Angle < 135, XC is XA-D, YC is YA);
			        ( 135 =< Angle, Angle < 225, XC is XA, YC is YA+D);
			        ( 225 =< Angle, Angle < 315, XC is XA+D, YC is YA)),
			       cellCenterInMap(XC, YC, X, Y), cellCoordIsPartOf(XC, YC, _).

% cell to the right of atom (metric map coordinates)
right_of_obj(Atom, X, Y) :- findall(X, right(Atom, X, _), LX), findall(Y, right(Atom, _, Y), LY), 
                            average(LX, X), average(LY, Y).
right(Atom, X, Y) :- objectAngle(Atom, Angle), constant(desired_distance_from_obj, D), objectPos(Atom, XA, YA, _),
			       (( 315 =< Angle, Angle < 45, XC is XA, YC is YA+D);
			        ( 45 =< Angle, Angle < 135, XC is XA+D, YC is YA);
			        ( 135 =< Angle, Angle < 225, XC is XA, YC is YA-D);
			        ( 225 =< Angle, Angle < 315, XC is XA-D, YC is YA)),
			       cellCenterInMap(XC, YC, X, Y), cellCoordIsPartOf(XC, YC, _).

% cell behind of atom (metric map coordinates)
behind_of_obj(Atom, X, Y) :- findall(X, behind(Atom, X, _), LX), findall(Y, behind(Atom, _, Y), LY), 
                            average(LX, X), average(LY, Y).
behind(Atom, X, Y) :- objectAngle(Atom, Angle), constant(desired_distance_from_obj, D), objectPos(Atom, XA, YA, _),
			       (( 315 =< Angle, Angle < 45, XC is XA-D, YC is YA);
			        ( 45 =< Angle, Angle < 135, XC is XA, YC is YA+D);
			        ( 135 =< Angle, Angle < 225, XC is XA+D, YC is YA);
			        ( 225 =< Angle, Angle < 315, XC is XA, YC is YA-D)),
			       cellCenterInMap(XC, YC, X, Y), cellCoordIsPartOf(XC, YC, _).

% nearest room cell from position (metric map coordinates)
nearest_cell_of_room_from(Room, XR, YR, ResultX, ResultY):- findall(Cell_X, cellCoordIsPartOf(Cell_X, _, Room), List_Cells_X), 
                                                 findall(Cell_Y, cellCoordIsPartOf(Cell_Y, _, Room), List_Cells_Y),
                                                 convertToReal(List_Cells_X, List_Cells_Y, [], [], List_Reals_X, List_Reals_Y),
                                                 nearest_target_in_coord_list_from(List_Reals_X, List_Reals_Y, XR, YR, ResultX, ResultY).

nearest_target_in_coord_list_from([A], [B], _, _, ResultX, ResultY):- ResultX = A, ResultY = B, !.
nearest_target_in_coord_list_from([AX,BX|TX], [AY,BY|TY], XR, YR, ResultX, ResultY):- 
                                                                ((( (AX-XR)**2 + (AY-YR)**2 ) =< ( (BX-XR)**2 + (BY-YR)**2 ), TResX = AX, TResY = AY);
                                                                (( (AX-XR)**2 + (AY-YR)**2 ) > ( (BX-XR)**2 + (BY-YR)**2 ), TResX = BX, TResY = BY)),
                                                                nearest_target_in_coord_list_from([TResX|TX], [TResY|TY], XR, YR, ResultX, ResultY).

% nearest atom in list from position X, Y (metric map coordinates)
nearest_atom_in_list_from([A], _, _, Result) :- Result = A, !.
nearest_atom_in_list_from([A,B|T], XR, YR, Result) :- nearest_centroid_between_two_atoms(A, B, XR, YR, TempResult), 
                                                   nearest_atom_in_list_from([TempResult|T], XR, YR, Result).

nearest_centroid_between_two_atoms(Atom1, Atom2, X, Y, Result):- (objectRealCentroid(Atom1, X1, Y1); roomRealCentroid(Atom1,X1, Y1)),
                                                                (objectRealCentroid(Atom2, X2, Y2); roomRealCentroid(Atom2,X2, Y2)),
                                                                ((( (X1-X)**2 + (Y1-Y)**2 ) =< ( (X2-X)**2 + (Y2-Y)**2 ), Result = Atom1);
                                                                (( (X1-X)**2 + (Y1-Y)**2 ) > ( (X2-X)**2 + (Y2-Y)**2 ), Result = Atom2)).
                                                   
% nearest target from position X, Y (metric map coordinates)
nearest_target_from(Type, XR, YR, A) :- nearest_obj_from(Type, XR, YR, A); nearest_room_from(Type, XR, YR, A).

% nearest object from position X, Y (metric map coordinates)
nearest_obj_from(Type, XR, YR, A) :- findall(X, objectType(X, Type), L), nearest_object_in_list(L, XR, YR, A), !.

nearest_object_in_list([A], _, _, Result) :- Result = A, !.
nearest_object_in_list([A,B|T], XR, YR, Result) :- nearest_centroid_between_two_obj(A, B, XR, YR, TempResult), 
						   nearest_object_in_list([TempResult|T], XR, YR, Result).

nearest_centroid_between_two_obj(Atom1, Atom2, X, Y, Result) :- objectRealCentroid(Atom1, X1, Y1), objectRealCentroid(Atom2, X2, Y2), 
				((( (X1-X)**2 + (Y1-Y)**2 ) =< ( (X2-X)**2 + (Y2-Y)**2 ), Result = Atom1);
				 (( (X1-X)**2 + (Y1-Y)**2 ) > ( (X2-X)**2 + (Y2-Y)**2 ), Result = Atom2)).
				 
% nearest room from position X, Y (metric map coordinates)
nearest_room_from(Type, XR, YR, A) :- findall(X, roomType(X, Type), L), nearest_room_in_list(L, XR, YR, A), !.

nearest_room_in_list([A], _, _, Result) :- Result = A, !.
nearest_room_in_list([A,B|T], XR, YR, Result) :- nearest_centroid_between_two_rooms(A, B, XR, YR, TempResult), 
                                                   nearest_room_in_list([TempResult|T], XR, YR, Result).

nearest_centroid_between_two_rooms(Atom1, Atom2, X, Y, Result) :- roomRealCentroid(Atom1, X1, Y1), roomRealCentroid(Atom2, X2, Y2), 
                                ((( (X1-X)**2 + (Y1-Y)**2 ) =< ( (X2-X)**2 + (Y2-Y)**2 ), Result = Atom1);
                                 (( (X1-X)**2 + (Y1-Y)**2 ) > ( (X2-X)**2 + (Y2-Y)**2 ), Result = Atom2)).

% farthest object from position X, Y (metric map coordinates)
farthest_obj_from(Type, XR, YR, A) :- findall(X, objectType(X, Type), L), farthest_object_in_list(L, XR, YR, A), !.

farthest_object_in_list([A], _, _, Result) :- Result = A, !.
farthest_object_in_list([A,B|T], XR, YR, Result) :- farthest_centroid_between_two_obj(A, B, XR, YR, TempResult), 
						   farthest_object_in_list([TempResult|T], XR, YR, Result).

farthest_centroid_between_two_obj(Atom1, Atom2, X, Y, Result) :- objectRealCentroid(Atom1, X1, Y1), objectRealCentroid(Atom2, X2, Y2), 
				((( (X1-X)**2 + (Y1-Y)**2 ) > ( (X2-X)**2 + (Y2-Y)**2 ), Result = Atom1);
				 (( (X1-X)**2 + (Y1-Y)**2 ) =< ( (X2-X)**2 + (Y2-Y)**2 ), Result = Atom2)).

% second nearest object from position X, Y (metric map coordinates)
second_nearest_obj_from(Type, XR, YR, A) :- findall(X, objectType(X, Type), L), second_nearest_object_in_list(L, XR, YR, A), !.

second_nearest_object_in_list([A,_], _, _, Result) :- Result = A, !.
second_nearest_object_in_list([A,B,C|T], XR, YR, Result) :- Head = [A,B,C], farthest_object_in_list(Head, XR, YR, Temp), 
							delete(Head, Temp, List), 
							firstElementInList(List, D), lastElementInList(List, E),
							nearest_centroid_between_two_obj(D, E, XR, YR, Closest),
							farthest_centroid_between_two_obj(D, E, XR, YR, Farthest),
						        second_nearest_object_in_list([Farthest,Closest|T], XR, YR, Result).

% second farthest object from position X, Y (metric map coordinates)
second_farthest_obj_from(Type, XR, YR, A) :- findall(X, objectType(X, Type), L), second_farthest_object_in_list(L, XR, YR, A), !.

second_farthest_object_in_list([A,_], _, _, Result) :- Result = A, !.
second_farthest_object_in_list([A,B,C|T], XR, YR, Result) :- Head = [A,B,C], nearest_object_in_list(Head, XR, YR, Temp), 
							delete(Head, Temp, List), 
							firstElementInList(List, D), lastElementInList(List, E),
							nearest_centroid_between_two_obj(D, E, XR, YR, Closest),
							farthest_centroid_between_two_obj(D, E, XR, YR, Farthest),
						        second_nearest_object_in_list([Closest, Farthest|T], XR, YR, Result).

%##################################################
% binary spatial relations about distances
%##################################################

% next(loc_obj, ref_obj, loc_atom, ref_atom)
nextto(ObjAtom, ObjAtom2) :- objectType(ObjAtom, ObjType), objectType(ObjAtom2, ObjType2), 
			     next(ObjType, ObjType2, ObjAtom, ObjAtom2).
next(Loc_Type, Ref_Type, Loc_Atom, Ref_Atom) :- objectType(Loc_Atom, Loc_Type), objectType(Ref_Atom, Ref_Type), 
						Loc_Atom \= Ref_Atom, constant(threshold_next_to, T_Next), 
						objectCellCentroid(Loc_Atom, XLoc,YLoc), objectCellCentroid(Ref_Atom, XRef, YRef),
						sqrt((XLoc-XRef)**2 + (YLoc-YRef)**2) =< T_Next.

% not next(loc_obj, ref_obj, loc_atom, ref_atom)
notnextto(ObjAtom, ObjAtom2) :- objectType(ObjAtom, ObjType), objectType(ObjAtom2, ObjType2), 
			     not_next(ObjType, ObjType2, ObjAtom, ObjAtom2).
not_next(Loc_Type, Ref_Type, Loc_Atom, Ref_Atom) :- objectType(Loc_Atom, Loc_Type), objectType(Ref_Atom, Ref_Type), 
						Loc_Atom \= Ref_Atom, constant(threshold_next_to, T_Next), 
						objectCellCentroid(Loc_Atom, XLoc,YLoc), objectCellCentroid(Ref_Atom, XRef, YRef),
						sqrt((XLoc-XRef)**2 + (YLoc-YRef)**2) > T_Next.

% near(loc_obj, ref_obj, loc_atom, ref_atom)
closeto(AtomA, AtomB) :- near(AtomA, AtomB).
near(ObjAtom, ObjAtom2) :- objectType(ObjAtom, ObjType), objectType(ObjAtom2, ObjType2), 
			   near(ObjType, ObjType2, ObjAtom, ObjAtom2).
near(Loc_Type, Ref_Type, Loc_Atom, Ref_Atom) :- objectType(Loc_Atom, Loc_Type), objectType(Ref_Atom, Ref_Type), 
						Loc_Atom \= Ref_Atom, constant(threshold_near, T_Near), 
						objectCellCentroid(Loc_Atom, XLoc, YLoc), objectCellCentroid(Ref_Atom, XRef, YRef),
						sqrt((XLoc-XRef)**2 + (YLoc-YRef)**2) =< T_Near.

% not near(loc_obj, ref_obj, loc_atom, ref_atom)
not_near(Loc_Type, Ref_Type, Loc_Atom, Ref_Atom) :- objectType(Loc_Atom, Loc_Type), objectType(Ref_Atom, Ref_Type), 
						Loc_Atom \= Ref_Atom, constant(threshold_near, T_Near), 
						objectCellCentroid(Loc_Atom, XLoc, YLoc), objectCellCentroid(Ref_Atom, XRef, YRef),
						sqrt((XLoc-XRef)**2 + (YLoc-YRef)**2) > T_Near.

% nearest(loc_obj, ref_obj, loc_atom, ref_atom)
nearest(Loc_Type, Ref_Type, Loc_Atom, Ref_Atom) :- objectType(Ref_Atom, Ref_Type), objectRealCentroid(Ref_Atom, XRef, YRef),
						((Loc_Type = Ref_Type, nearest_obj_from(Loc_Type, XRef, YRef, Loc_Atom));
						 (Loc_Type \= Ref_Type, second_nearest_obj_from(Loc_Type, XRef, YRef, Loc_Atom))).

% farthest(loc_obj, ref_obj, loc_atom, ref_atom)
farthest(Loc_Type, Ref_Type, Loc_Atom, Ref_Atom) :- objectType(Ref_Atom, Ref_Type), objectRealCentroid(Ref_Atom, XRef, YRef),
						((Loc_Type = Ref_Type, farthest_obj_from(Loc_Type, XRef, YRef, Loc_Atom));
						 (Loc_Type \= Ref_Type, second_farthest_obj_from(Loc_Type, XRef, YRef, Loc_Atom))).

%##################################################
% binary spatial relations about directions
%##################################################

% loc_obj in front of ref_obj adopting the normal of the ref_obj as the front direction 
frontof(ObjAtom, ObjAtom2) :- objectType(ObjAtom, ObjType), objectType(ObjAtom2, ObjType2), 
			      front(ObjType, ObjType2, ObjAtom, ObjAtom2).
front(Loc_Type, Ref_Type, Loc_Atom, Ref_Atom) :- objectType(Loc_Atom, Loc_Type), objectType(Ref_Atom, Ref_Type), Loc_Atom \= Ref_Atom,
						objectRealCentroid(Loc_Atom, XLoc, YLoc), objectRealCentroid(Ref_Atom, XRef, YRef),
						near(Loc_Type, Ref_Type, Loc_Atom, Ref_Atom),
						objectAngle(Ref_Atom, ARef), Temp is ARef - (atan2(YLoc - YRef, XLoc- XRef)*180.0/pi),
						normalizeAngle(Temp, RelAng), (RelAng > 315; RelAng =< 45).

% loc_obj on the left of ref_obj adopting the normal of the ref_obj as the front direction 
leftof(ObjAtom, ObjAtom2) :- objectType(ObjAtom, ObjType), objectType(ObjAtom2, ObjType2), 
                             left(ObjType, ObjType2, ObjAtom, ObjAtom2).
left(Loc_Type, Ref_Type, Loc_Atom, Ref_Atom) :- objectType(Loc_Atom, Loc_Type), objectType(Ref_Atom, Ref_Type), Loc_Atom \= Ref_Atom,
						objectRealCentroid(Loc_Atom, XLoc, YLoc), objectRealCentroid(Ref_Atom, XRef, YRef),
						near(Loc_Type, Ref_Type, Loc_Atom, Ref_Atom),
						objectAngle(Ref_Atom, ARef), Temp is ARef - (atan2(YLoc - YRef, XLoc- XRef)*180.0/pi),
						normalizeAngle(Temp, RelAng), (RelAng > 225, RelAng =< 315).

% loc_obj behind ref_obj adopting the normal of the ref_obj as the front direction 
behind(ObjAtom, ObjAtom2) :- objectType(ObjAtom, ObjType), objectType(ObjAtom2, ObjType2), 
			     behind(ObjType, ObjType2, ObjAtom, ObjAtom2).
behind(Loc_Type, Ref_Type, Loc_Atom, Ref_Atom) :- objectType(Loc_Atom, Loc_Type), objectType(Ref_Atom, Ref_Type), Loc_Atom \= Ref_Atom,
						objectRealCentroid(Loc_Atom, XLoc, YLoc), objectRealCentroid(Ref_Atom, XRef, YRef),
						near(Loc_Type, Ref_Type, Loc_Atom, Ref_Atom),
						objectAngle(Ref_Atom, ARef), Temp is ARef - (atan2(YLoc - YRef, XLoc- XRef)*180.0/pi),
						normalizeAngle(Temp, RelAng), (RelAng > 135, RelAng =< 225).

% loc_obj on the right of ref_obj adopting the normal of the ref_obj as the front direction 
rightof(ObjAtom, ObjAtom2) :- objectType(ObjAtom, ObjType), objectType(ObjAtom2, ObjType2), 
			      right(ObjType, ObjType2, ObjAtom, ObjAtom2).
right(Loc_Type, Ref_Type, Loc_Atom, Ref_Atom) :- objectType(Loc_Atom, Loc_Type), objectType(Ref_Atom, Ref_Type), Loc_Atom \= Ref_Atom,
						objectRealCentroid(Loc_Atom, XLoc, YLoc), objectRealCentroid(Ref_Atom, XRef, YRef),
						near(Loc_Type, Ref_Type, Loc_Atom, Ref_Atom),
						objectAngle(Ref_Atom, ARef), Temp is ARef - (atan2(YLoc - YRef, XLoc- XRef)*180.0/pi),
						normalizeAngle(Temp, RelAng), (RelAng > 45, RelAng =< 135).

%##################################################
% other spatial relations
%##################################################

% object inside a specific room
in(ObjAtom, RoomAtom) :- objectType(ObjAtom, ObjType), roomType(RoomAtom, RoomType), 
                        in(ObjType, RoomType, ObjAtom, RoomAtom).
in(ObjType, RoomType, ObjAtom, RoomAtom) :- objectType(ObjAtom, ObjType), roomType(RoomAtom, RoomType),
                                            objectCellCentroid(ObjAtom, XLoc, YLoc), X is round(XLoc), Y is round(YLoc),
                                            cellCoordIsPartOf(X, Y, RoomAtom).

%##################################################
% support functions
%##################################################

objectRealCentroid(Atom, X, Y) :- findall(Cell_X, objectPos(Atom, Cell_X, _, _), List_Cells_X), findall(Cell_Y, objectPos(Atom, _, Cell_Y, _), List_Cells_Y), 
                              convertToReal(List_Cells_X, List_Cells_Y, [], [], List_Reals_X, List_Reals_Y), 
                              average(List_Reals_X, X), average(List_Reals_Y, Y).

objectCellCentroid(Atom, X, Y) :- findall(Cell_X, objectPos(Atom, Cell_X, _, _), List_Cells_X), findall(Cell_Y, objectPos(Atom, _, Cell_Y, _), List_Cells_Y), 
                              average(List_Cells_X, X), average(List_Cells_Y, Y).

roomRealCentroid(Atom, X, Y) :- findall(Cell_X, cellCoordIsPartOf(Cell_X, _, Atom), List_Cells_X), findall(Cell_Y, cellCoordIsPartOf( _, Cell_Y, Atom), List_Cells_Y), 
                              convertToReal(List_Cells_X, List_Cells_Y, [], [], List_Reals_X, List_Reals_Y), 
                              average(List_Reals_X, X), average(List_Reals_Y, Y).

roomCellCentroid(Atom, X, Y) :- findall(Cell_X, cellCoordIsPartOf(Cell_X, _, Atom), List_Cells_X), findall(Cell_Y, cellCoordIsPartOf( _, Cell_Y, Atom), List_Cells_Y), 
                              average(List_Cells_X, X), average(List_Cells_Y, Y).

get_cell_near_destination_room(XCR,YCR,Destination,X,Y):- cellCoordIsPartOf(XCR,YCR,Starting_Room), 
                    findall(X, (path(Starting_Room,Destination,A), length(A, Len), Len >=2, secondToLastElementInList(A, Door), stat_node(Door, X, _)), LX),
                    findall(Y, (path(Starting_Room,Destination,A), length(A, Len), Len >=2, secondToLastElementInList(A, Door), stat_node(Door, _, Y)), LY),
                    length(LX, Len), Len >=2, length(LY, Len), Len >=2, 
                    set(LX,LXT), set(LY,LYT), average(LXT,XC), average(LYT,YC), convertToReal(XC, YC, X, FIX), Y is FIX - 1, !.


secondToLastElementInList([A,_], A).
secondToLastElementInList([_,B|T],Elem):- secondToLastElementInList([B|T], Elem).
firstElementInList([Elem|_], Elem).
lastElementInList([Elem], Elem).
lastElementInList([_|Tail], Elem) :- last(Tail, Elem).

normalizeAngle(A, NormA) :- (A >= 0, A < 360, NormA is A, !); 
			    (A >= 360, normalizeAngle(A-360, NormA), !);
			    (A < 0, normalizeAngle(A+360, NormA), !).

min(A,B,C) :- (C is B, B=<A); (C is A, A<B).
max(A,B,C) :- (C is A, B=<A); (C is B, A<B).

average(List, Result) :- length(List, Len), sum(List, Sum), Result is Sum / Len.

sum([], 0).
sum([H|T], Sum) :- sum(T, Temp), Sum is Temp + H.

convertToReal(CellX, CellY, RealX, RealY) :- cellCenterInMap(CellX, CellY, RealX, RealY).
%Temp1 and Temp2 should be initialized to empty lists
convertToReal([EX], [EY], Temp1, Temp2, RealX, RealY) :- cellCenterInMap(EX, EY, X, Y), append([X], Temp1, RealX), append([Y], Temp2, RealY).
convertToReal([EX|RestX], [EY|RestY], Temp1, Temp2, RealX, RealY) :- cellCenterInMap(EX, EY, X, Y), 
                                                                    append([X], Temp1, PartialRealX), 
                                                                    append([Y], Temp2, PartialRealY),
                                                                    convertToReal(RestX, RestY, PartialRealX, PartialRealY, RealX, RealY).

allObjectsInRoom(XR,YR,L) :- findall(X, objectInRoom(XR,YR,X), L).
objectInRoom(XR, YR, [O,XO,YO]) :- objectPos(O,XO,YO,_), cellCoordIsPartOf(XR,YR,Room), cellCoordIsPartOf(XO,YO,Room).

% An empty list is a set.
set([], []).

% Put the head in the result,
% remove all occurrences of the head from the tail,
% make a set out of that.
set([H|T], [H|T1]) :- 
    remv(H, T, T2),
    set(T2, T1).

% Removing anything from an empty list yields an empty list.
remv(_, [], []).

% If the head is the element we want to remove,
% do not keep the head and
% remove the element from the tail to get the new list.
remv(X, [X|T], T1) :- remv(X, T, T1).

% If the head is NOT the element we want to remove,
% keep the head and
% remove the element from the tail to get the new tail.
remv(X, [H|T], [H|T1]) :-
    X \= H,
    remv(X, T, T1).
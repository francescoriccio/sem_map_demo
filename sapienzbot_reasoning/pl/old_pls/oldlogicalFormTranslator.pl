reduce(Arg^Expr, Arg, Expr).

getAtom(Sent,LF,Var) :- s(LF,Sent,[]), term_variables(LF,Var), call(LF).

s(S) --> pp(S).
s(S) --> np(S).
s(S) --> np1(S).
s(S) --> npNext(S).

pp(PP) --> in(IN), np1(NP), {reduce(IN,NP,PP)}.
pp(PP) --> in(IN), np(NP), {reduce(IN,NP,PP)}.


npSpc(NP) --> n(N1,person), n(N2,building_subpart), {reduce(Y^of(Y,N1,_,_),N2,NP)}.
npSpc(NP) --> n(_,person_spec), n(N1,person), n(N2,building_subpart), {reduce(Y^of(Y,N1,_,_),N2,NP)}.

np(NP) --> n(NP,_).
np(NP) --> npSpc(NP2), np(N), {reduce(Y^of(Y,NP2,_,_),N,NP)}.
np(NP) --> n(N1,person), np(N2), {reduce(Y^of(Y,N1,_,_),N2,NP)}.
np(NP) --> n(_,person_spec), n(N1,person), np(N2), {reduce(Y^of(Y,N1,_,_),N2,NP)}.
np(NP) --> n(N1,building_subpart), np(N2), {reduce(Y^of(Y,N1,_,_),N2,NP)}.
np(N2) --> n(_,person_spec), n(N2,person).
np(NP) --> n(N1,_), n(N2,_), {atom_concat(N1,'_',Temp)}, {atom_concat(Temp,N2,NP)}.
np(NP) --> n(N1,_), n(N2,_), n(N3,_), {atom_concat(N1,'_',Temp1)}, {atom_concat(Temp1,N2,Temp2)}, 
				     {atom_concat(Temp2,'_',Temp3)}, {atom_concat(Temp3,N3,NP)}.
np(NP) --> n(N,_), npNext(NP2), {reduce(NP2,N,NP)}.
%np(NP) --> jj(NP).
%np(NP) --> np(NP1), np(NP2), {reduce(NP2,NP1,NP)}.
npNext(NP) --> jj(JJ), pp(PP), {reduce(JJ,PP,NP)}.

np1(NP) --> np(NP2), pp(PP), {reduce(PP,NP2,NP)}.

in(X^Y^of(Y,X,_,_)) --> [of].
in(Y^to(Y,_)) --> [to].
in(Y^near(Y,_)) --> [near].
in(X^Y^near(Y,X,_,_)) --> [near].
in(Y^in(Y,_)) --> [in].
in(Y^into(Y,_)) --> [into].
%in(X^next(X,_)) --> [next].
%in(X^close(X,_)) --> [close].

jj(Y^next(Y,_)) --> [next].
jj(Y^next(Y,_)) --> [close].
jj(X^Y^next(Y,X,_,_)) --> [next].
jj(X^Y^next(Y,X,_,_)) --> [close].

n(N,Cat) --> [N], {n(N,Cat)}.

n(room,building_subpart).
n(corridor,building_subpart).
n(toilet,building_subpart).
n(hall,building_subpart).
n(hallway,building_subpart).
n(kitchen,building_subpart).
n(bedroom,building_subpart).
n(bathroom,building_subpart).

n(door,connecting_architecture).
n(stairs,connecting_architecture).
n(window,connecting_architecture).
n(exit,connecting_architecture).

n(fire,object). 
n(super,object).
n(extinguisher,object). 
n(emergency,object). 
n(chair,object). 
n(hydrant,object).
n(plug,object). 
n(table,furniture).
n(chair,furniture). 
n(socket,object). 
n(power,object). 
n(printer,object). 
n(closet,object). 
n(documentation,object). 
n(stationery,object). 
n(utility,object). 
n(book,object). 
n(bench,object). 
n(trash,object). 
n(bin,object). 
n(plant,object). 
n(shower,object).
n(couch,object).
n(chair,object).
n(sink,object).
n(fridge,object).
n(counter,object).
n(closet,object).
n(commode,object).
n(washer,object).
n(dishwasher,object).
n(dresser,object).
n(shelf,object).
n(bookshelf,object).
n(heater,object).
n(wardrobe,object).
n(sofa,object).
n(bed,object).
n(night,object).
n(stand,object).
n(microwave,object).
n(oven,object).
n(refrigerator,object).
n(desk,object).
n(armchair,object).
n(bowl,object).
n(wc,object).
n(bidet,object).
n(bide,object).
n(beede,object).
n(stove,object).
n(machine,object).
n(table,object).
n(bedside,object).
n(dining,object).
n(living,object).
n(wood,object).
n(sewing,object).
n(washing,object).
n(box,object).
n(jar,object).
n(bottle,object).
n(pillow,object).
n(lighter,object).
n(suitcase,object).
n(electric,object).
n(coffee,object).
n(cabinet,object).
n(boiler,object).
n(dish,object).
n(dishes,object).
n(toilet,object).
n(paper,object).
n(garbage,object).
n(towel,object).
n(umbrella,object).
n(can,object).
n(food,object).
n(fork,object).
n(knife,object).
n(glass,object).
n(cup,object).
n(basket,object).
n(bike,object).
n(bicycle,object).
n(headphones,object).
n(glasses,object).
n(money,object).
n(bag,object).
n(cigarette,object).
n(cigarettes,object).
n(book,object).
n(phone,object).
n(printer,object).
n(cell,object).
n(mobile,object).
n(telephone,object).
n(fax,object).
n(control,object).
n(remote,object).
n(laptop,object).
n(recorder,object).
n(router,object).
n(computer,object).
n(desktop,object).
n(system,object).
n(hifi,object).
n(fire,object).
n(pc,object).
n(stereo,object).
n(dryer,object).
n(hair,object).
n(tv,object).
n(television,object).
n(set,object).
n(dvd,object).
n(player,object).
n(main,object).
n(entrance,object).
n(place,object).
n(bar,object).
n(plug,object).
n(freezer, object).

n(daniele,person).
n(anna,person).
n(michela,person).
n(iacopo,person).

n(professor,person_spec).

%% KB %%
to(A,Atom) :- objectType(Atom,A).
to(A,Atom) :- synonym(A,SynA), objectType(Atom,SynA).
to(A,Atom) :- roomType(Atom,A).
to(of(A,B,Atom,Batom),Atom) :- of(A,B,Atom,Batom).
to(of(A,of(B,C,Batom,_),Atom,Batom),Atom) :- of(A,B,Atom,Batom), of(B,C,Batom,_).

near(A,Atom) :- to(A,Atom).

in(A,Atom) :- to(A,Atom).

into(A,Atom) :- to(A,Atom).

of(A,B,Atom,Batom) :- objectOf(A,B,Atom,Batom).
of(A,B,Atom,_) :- roomOf(A,B,Atom).
of(A,B,Atom,Catom) :- roomOf(C,B,Catom), objectOf(A,C,Atom,Catom).

objectOf(A,B,Atom,Batom) :- objectType(Atom,A), objectPos(Atom,X,Y,_), roomType(Batom,B), cellCoordIsPartOf(X,Y,Batom).
objectOf(A,B,Atom,Batom) :- synonym(A,SynA), objectType(Atom,SynA), objectPos(Atom,X,Y,_), roomType(Batom,B), cellCoordIsPartOf(X,Y,Batom).
roomOf(A,B,Atom) :- roomType(Atom,A), roomSpec(Atom,B).

synonym(socket,socket).
synonym(plug,socket).
synonym(power_socket,socket).

isA(stationery_closet,closet).
isA(documentation_closet,closet).
isA(utility_closet,closet).
isA(storage_closet,closet).
isA(book_closet,closet).
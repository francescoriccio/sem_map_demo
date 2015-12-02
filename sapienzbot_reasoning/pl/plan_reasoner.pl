const(afterGrade, 3).
const(beforeGrade, 3).
const(imAfterGrade, 1).
const(imBeforeGrade, 1).

plan(X):- combinedPlan(X); basicPlan(X).

isParent(X, Y):- planEdge(X, Y).
isChild(X, Y):- planEdge(Y, X).

isAncestor(X, Y):- isParent(X, Y).
isAncestor(X, Y):- isParent(X, Z), isAncestor(Z, Y), !.

isAncestorGrade(X, Y, GRADE):- GRADE >= 1, isParent(X, Y), !.
isAncestorGrade(X, Y, GRADE):- NEWGRADE is GRADE - 1, NEWGRADE >= 1, isParent(X, Z), isAncestorGrade(Z, Y, NEWGRADE), !.

isDescendent(X, Y):- isChild(X, Y).
isDescendent(X, Y):- isChild(X, Z), isDescendent(Z, Y), !.

isDescendentGrade(X, Y, GRADE):- GRADE >= 1, isChild(X, Y), !.
isDescendentGrade(X, Y, GRADE):- NEWGRADE is GRADE - 1, NEWGRADE >= 1, isChild(X, Z), isDescendentGrade(Z, Y, NEWGRADE), !.

after(X, Y, IDX, IDY):- plan(X), plan(Y), placeName(IDX, X), placeName(IDY, Y), const(afterGrade, GRADE), isDescendentGrade(IDX, IDY, GRADE).
before(X, Y, IDX, IDY):- plan(X), plan(Y), placeName(IDX, X), placeName(IDY, Y), const(beforeGrade, GRADE), isAncestorGrade(IDX, IDY, GRADE).

immediatlyAfter(X, Y, IDX, IDY):- plan(X), plan(Y), placeName(IDX, X), placeName(IDY, Y), const(imAfterGrade, GRADE), isDescendentGrade(IDX, IDY, GRADE).
immediatlyBefore(X, Y, IDX, IDY):- plan(X), plan(Y), placeName(IDX, X), placeName(IDY, Y), const(imBeforeGrade, GRADE), isAncestorGrade(IDX, IDY, GRADE).

while(X, Y, IDX, IDY):- plan(X), plan(Y),  placeName(IDX, X), placeName(IDY, Y), IDX \== IDY,
                        isDescendent(IDX, XA), isDescendent(IDY, YA), XA == YA,
                        isAncestor(IDX, XD), isAncestor(IDY, YD), XD == YD.

planEdge(a1, a2).
planEdge(a1, a3).
planEdge(a2, a4).
planEdge(a3, a4).

action(a1, turn).
action(a2, goTo).
action(a3, say).
action(a4, turn).

placeName(a1, turn).
placeName(a2, goTo).
placeName(a3, say).
placeName(a4, turn).

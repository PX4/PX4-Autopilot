{ Compute the sine using: sqrt(1 - cosine**2)
  This file verifies a unit that uses another unit
}

unit MySineUnit;

interface

function mysine(x : real) : real;

implementation

uses
   MyCosineUnit in '501-unit-cosine.pas';

function mysine(x : real) : real;
var
   mycos : real
begin
   mycos := mycosine(x);
   mysine := sqrt(1.0 - sqr(mycos))
end; { mysine }
end.


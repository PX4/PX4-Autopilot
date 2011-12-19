{ Compute the cosine using the expansion:
  cos(x) = 1 - x**2/(2*1) + x**4/(4*3*2*1) - ...
  This file verifies a simple unit that exports one function.
}

unit MyCosineUnit;

interface

function mycosine(x : real) : real;

implementation

function mycosine(x : real) : real;
const
   eps = 1e-14;

var
   sx, s, t : real;
   i, k	    : integer;

begin
   t := 1;
   k := 0;
   s := 1;
   sx := sqr(x);
   while abs(t) > eps*abs(s) do
   begin
      k := k + 2;
      t := -t * sx / (k * (k - 1));
      s := s + t;
   end;
   mycosine := s
end; { mycosine }
end.


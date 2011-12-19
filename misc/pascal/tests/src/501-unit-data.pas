{ This file tests only that data can be shared correctly between units }

unit MyDataUnit;

interface

var
   mycosx : real;
   mysinx : real;
   myone  : real;

procedure checkvars;

implementation

procedure checkvars;
begin
   myone := sqr(mycosx) + sqr(mysinx)
end; { checkvars }
end.


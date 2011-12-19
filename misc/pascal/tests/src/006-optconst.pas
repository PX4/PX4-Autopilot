{ A test of constant optimiztion }

program constopt;
var 
   i1, i2, i3 : integer;
   b1	      :  boolean
begin
   { integer operations:
   1. =, <>, <, <=, >, >=
   2. +, -, OR
   3. *, DIV, AND, SHL, SHR
   4. -, NOT
   }
     
   i1 := -2;                { -2 }
   i1 := 3 * 2;             {  6 }
   i1 := 3 * (-2);          { -6 }
   i1 := 3 div 2;           {  1 }
   i1 := 3 div (-2);        { -1 }
   i1 := 4 * 3 div (-2);    { -6 }
   i1 := 4 * (3 div (-2));  { -4 }
   i1 := (4 * 3) div (-2);  { -6 }

   i1 := 3 + 2;             {  5 }
   i1 := 3 - 2;             {  1 }
   i1 := 3 + (-2);          {  1 }
   i1 := 4 + 3 - 2;         {  5 }
   i1 := 4 + (3 - 2);       {  5 }
   i1 := (4 + 3) - 2;       {  5 }

   b1 := -2 = 3 + 2;                    { -2 =  5  0 }
   b1 := 3 * 2 <> 3 - 2;                {  6 <> 1 -1 }
   b1 := 3 * (-2) < 3 + (-2);           { -6 <  1 -1 }
   b1 := 3 div 2 <= 4 + 3 - 2;          {  1 <= 5 -1 }
   b1 := 3 div (-2) > 4 + (3 - 2);      { -1 >  5  0 }
   b1 := 4 * 3 div (-2) >= (4 + 3) - 2; { -6 >= 5  0 }

   { floating point operations }
   { to be provided }

   { string operations }
   { to be provided }
end.

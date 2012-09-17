program MyProgram;
 
uses
   MyCosineUnit in '501-unit-cosine.pas';
   MySineUnit in '501-unit-sine.pas';
   MyDataUnit in '501-unit-data.pas';

var
   x : real;
 
begin			
   write('Enter radians	: ');
   read(x);
   mycosx := mycosine(x);
   writeln('cos(', x, ')=', mycosx);
   mysinx := mysine(x);
   writeln('sin(', x, ')=', mysinx);
   checkvars;
   writeln('sin(', x, ')**2 + cos(', x, ')**2=', myone)
end.

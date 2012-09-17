{ a simple nested function }

program simplefunc(output);

function addmul(term1a, term1b, term2a, term2b: integer ) : integer;
   function factor(terma, termb: integer ) : integer;
   begin
      factor := terma + termb;
   end;
begin
   addmul := factor(term1a, term1b) * factor(term2a, term2b);
end;

begin
  writeln('(1 + 2) * (3 + 4) =', addmul(1, 2, 3, 4));
end.

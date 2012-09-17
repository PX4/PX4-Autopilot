{ assuming annual inflation rates of 7, 8, and 10 per cent,
  find the factor by which the frank, dollar, pound
  sterlinh, mark, or guilder will have been devalued in
  1, 2, ... n years.}

program inflation(output);

const
  n = 10;
var
  i : integer;
  w1, w2, w3 : real;

begin
  i := 0;
  w1 := 1.0;
  w2 := 1.0;
  w3 := 1.0;

  repeat
    i := i + 1;
    w1 := w1 * 1.07;
    w2 := w2 * 1.08;
    w3 := w3 * 1.10;
     writeln(i, '. ', w1, ', ', w2, ', ', w3);
  until i=n
end.

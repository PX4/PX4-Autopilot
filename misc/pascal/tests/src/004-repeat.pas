{ compute h(n) = 1 + 1/2 + 1/3 +...+ 1/n }

program egrepeat(input, output);

var
  n : integer;
  h : real;

begin
  read(n);
  writeln(n);
  h := 0;
  repeat
    h := h + 1/n;
    n := n - 1;
  until n=0;
  writeln(h);
end.

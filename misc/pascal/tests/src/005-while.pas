{ compute h(n) = 1 + 1/2 + 1/3 +...+ 1/n }

program egwhile(input, output);

var
  n : integer;
  h : real;

begin
  read(n);
  writeln(n);
  h := 0;
  while n>0 do
  begin
    h := h + 1/n;
    n := n - 1;
  end;
  writeln(h);
end.

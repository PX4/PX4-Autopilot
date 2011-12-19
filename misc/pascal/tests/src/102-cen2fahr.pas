{ example of constant definition part }

program convert(output);

const
  addin = 32;
  mulby = 1.8;
  low = 0;
  high = 39;
  seperator = '------------';

var
  degree : low..high;

begin
  writeln(seperator);
  for degree := low to high do
  begin
    writeln(degree, 'c', round(degree * mulby + addin), 'f');
  end;
  writeln(seperator)
end.

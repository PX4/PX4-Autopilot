{ writeln with a variety of arguments }

program check_writeln(output);

const
   floater	= 1.8;
   low		= 37;
   high		= 492;
   range_string	= ' range=';

var
   range : low..high;

begin
   range := (low + high) div 2;
   writeln('A', range_string, range, ' B floater=', floater);
end.

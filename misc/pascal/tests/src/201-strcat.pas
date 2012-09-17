PROGRAM stringcat;
VAR
   string1, string2  : string

FUNCTION inquote(instring : string) : string;
BEGIN
   inquote := '"' + instring + '"'
END;

BEGIN
   WRITELN(string1);

   string1 := 'Now ';
   WRITELN(inquote(string1));

   string2 := 'is the time ';
   WRITELN(inquote(string2));

   string1 := string1 + string2 + 'for all good men ';
   WRITELN(inquote(string1));

   string2 := 'to come ' + 'to ';
   WRITELN(inquote(string2));

   string1 := string1 + string2;
   WRITELN(inquote(string1));

   string2 := 'of their party';
   WRITELN(inquote(string2));

   string2 := 'the aid ' + string2;
   WRITELN(inquote(string2));

   string1 := string1 + string2 + '.';
   WRITELN(inquote(string1));
END.
